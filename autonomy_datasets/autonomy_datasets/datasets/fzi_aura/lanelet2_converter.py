# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

"""Generate Lanelet2 (OSM XML) maps of FZI-AURA scenes from OpenStreetMap.

FZI-AURA ships no map, but every sample carries the GNSS position of the vehicle. This module
fetches the OpenStreetMap roads around the GNSS track of a scene from the Overpass API and
converts them into Lanelet2 lanelets in the ``map`` frame the adapter publishes the ego poses in.

Georeferencing
--------------
The adapter publishes the ego poses in an east-north-up aligned ``map`` frame, whose origin is the
arbitrary origin of the released odometry. A scene is georeferenced by fitting the offset between
the GNSS fixes of its samples, projected to a local transverse Mercator plane centered at the
first fix, and the positions of the ego poses in ``map``. The east-north-up alignment makes a
translation sufficient; the residual stays in the order of a few decimeters and is dominated by
the lever arm of the GNSS antenna, which the release does not calibrate. The height of the
``map`` frame is equally arbitrary, so a map node takes the height of the ego pose closest to it.

Map content
-----------
OpenStreetMap draws a road as its centerline. Every road way becomes one lanelet per lane, laid
out from the ``lanes``, ``lanes:forward``, ``lanes:backward`` and ``oneway`` tags for right-hand
traffic, whose boundaries are synthesized by offsetting the centerline by multiples of the
configured lane width. Pedestrian crossings mapped as ``footway=crossing`` ways become
``crosswalk`` lanelets. The centerlines of different roads end at a shared junction node, so the
lanelets of different roads are not connected at junctions.

The generated maps contain information from OpenStreetMap, © OpenStreetMap contributors, made
available under the Open Database License (ODbL) 1.0.
"""

import json
import math
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple
from urllib.error import HTTPError
from urllib.parse import urlencode
from urllib.request import Request, urlopen

import numpy as np
from autonomy_datasets.datasets.lanelet2_osm import (
    geodetic_to_utm,
    Lanelet2OsmBuilder,
    utm_central_meridian_deg,
    UTM_K0,
    utm_to_geodetic,
    WGS84_A,
)
from rclpy.logging import get_logger

LOGGER = get_logger("autonomy_datasets.fzi_aura")

DEFAULT_OVERPASS_URL = "https://overpass-api.de/api/interpreter"

_HTTP_TIMEOUT_SECONDS = 90
# Time the Overpass API may spend on a query before it aborts it [s]
_OVERPASS_QUERY_TIMEOUT_SECONDS = 60
# Delays before retrying a query the Overpass API rejected because it is busy [s]. The public
# instances balance queries across servers, so a retry often reaches one that is not overloaded.
_OVERPASS_RETRY_DELAYS_SECONDS = (5.0, 15.0, 30.0)
# HTTP status codes of an Overpass API that is busy: too many requests, bad gateway, service
# unavailable, gateway timeout
_OVERPASS_BUSY_STATUS_CODES = (429, 502, 503, 504)

# OSM highway classes that carry motorized traffic, mapped to the Lanelet2 subtype and location of
# their lanelets. OpenStreetMap does not distinguish urban from rural roads below the trunk class.
_ROAD_CLASSES: Dict[str, Tuple[str, str]] = {
    "motorway": ("highway", "nonurban"),
    "motorway_link": ("highway", "nonurban"),
    "trunk": ("road", "nonurban"),
    "trunk_link": ("road", "nonurban"),
    "primary": ("road", "urban"),
    "primary_link": ("road", "urban"),
    "secondary": ("road", "urban"),
    "secondary_link": ("road", "urban"),
    "tertiary": ("road", "urban"),
    "tertiary_link": ("road", "urban"),
    "unclassified": ("road", "urban"),
    "residential": ("road", "urban"),
    "living_street": ("play_street", "urban"),
    "service": ("road", "urban"),
    "road": ("road", "urban"),
}

# Highway classes and junctions OpenStreetMap implies to be one-way unless tagged otherwise.
_IMPLIED_ONEWAY_HIGHWAYS = ("motorway", "motorway_link")
_IMPLIED_ONEWAY_JUNCTIONS = ("roundabout", "circular")

# Width of a crosswalk lanelet around its OSM centerline [m]
_CROSSWALK_WIDTH = 4.0

# Upper bound of the lane count of a single road way, guarding against mistagged ways.
_MAX_LANES = 8

# Minimum distance between two consecutive points of a way [m]; closer points are merged.
_MIN_POINT_DISTANCE = 0.01


class SceneGeoreference:
    """Relates WGS84 coordinates to the ``map`` frame of a scene.

    Fitted to the GNSS fixes the vehicle recorded at the samples of a scene and the positions of
    its ego poses in ``map`` at the same samples.
    """

    def __init__(self, fixes: np.ndarray, positions: np.ndarray) -> None:
        """Fit the georeference.

        Args:
            fixes: ``(N, 2)`` GNSS fixes as latitude and longitude in WGS84 degrees; NaN where the
                sample holds no fix.
            positions: ``(N, 3)`` positions of the ego poses in ``map`` at the same samples.

        Raises:
            ValueError: If no sample holds a GNSS fix.
        """
        valid = np.all(np.isfinite(fixes), axis=1) & np.any(fixes != 0.0, axis=1) & np.all(np.isfinite(positions), axis=1)
        if not np.any(valid):
            raise ValueError("no sample holds a GNSS fix")
        self._fixes = fixes[valid]
        self._positions = positions[valid]
        self._reference_lat, self._reference_lon = (float(value) for value in self._fixes[0])
        self._reference_easting, self._reference_northing = geodetic_to_utm(
            self._reference_lat, self._reference_lon, self._reference_lon
        )

        local = np.array([self._to_local(lat, lon) for lat, lon in self._fixes])
        self._offset = np.mean(self._positions[:, :2] - local, axis=0)
        self.residual = float(np.sqrt(np.mean(np.sum((local + self._offset - self._positions[:, :2]) ** 2, axis=1))))

        # The map is projected with lanelet2's UtmProjector, so its nodes only line up with 'map'
        # when their lat/lon are the inverse of that projection. The origin is chosen such that
        # the resulting lat/lon match the true position at the first fix of the scene.
        lon0 = utm_central_meridian_deg(self._reference_lon)
        easting, northing = geodetic_to_utm(self._reference_lat, self._reference_lon, lon0)
        self.origin_lat, self.origin_lon = utm_to_geodetic(easting - self._offset[0], northing - self._offset[1], lon0)

    def _to_local(self, lat: float, lon: float) -> Tuple[float, float]:
        """Project a WGS84 coordinate to the plane tangent at the first fix (x=east, y=north).

        The transverse Mercator projection is centered at the meridian of the first fix, where it
        is true to north and, once the UTM scale factor is removed, true to scale.
        """
        easting, northing = geodetic_to_utm(lat, lon, self._reference_lon)
        return (easting - self._reference_easting) / UTM_K0, (northing - self._reference_northing) / UTM_K0

    def to_map(self, lat: float, lon: float) -> Tuple[float, float, float]:
        """Return the position of a WGS84 coordinate in ``map``, at the height of the closest ego pose."""
        x, y = np.add(self._to_local(lat, lon), self._offset)
        closest = int(np.argmin(np.sum((self._positions[:, :2] - (x, y)) ** 2, axis=1)))
        return float(x), float(y), float(self._positions[closest, 2])

    def bounding_box(self, margin: float) -> Tuple[float, float, float, float]:
        """Return the ``(south, west, north, east)`` bounds of the GNSS track, extended by a margin in meters."""
        margin_lat = math.degrees(margin / WGS84_A)
        margin_lon = margin_lat / math.cos(math.radians(self._reference_lat))
        lat, lon = self._fixes[:, 0], self._fixes[:, 1]
        return (
            float(lat.min()) - margin_lat,
            float(lon.min()) - margin_lon,
            float(lat.max()) + margin_lat,
            float(lon.max()) + margin_lon,
        )


def fetch_osm_roads(bbox: Tuple[float, float, float, float], overpass_url: str = DEFAULT_OVERPASS_URL) -> List[Dict[str, Any]]:
    """Fetch the road and crossing ways within a bounding box from the Overpass API.

    Args:
        bbox: ``(south, west, north, east)`` bounds in WGS84 degrees.
        overpass_url: Interpreter endpoint of the Overpass API.

    Returns:
        The OSM ways, each with its ``tags`` and the lat/lon ``geometry`` of its nodes.

    Raises:
        OSError: If the Overpass API cannot be reached or rejects the query.
        ValueError: If the Overpass API responds with invalid JSON.
    """
    box = ",".join(f"{value:.7f}" for value in bbox)
    highways = "|".join(_ROAD_CLASSES)
    query = (
        f"[out:json][timeout:{_OVERPASS_QUERY_TIMEOUT_SECONDS}];"
        f'(way["highway"~"^({highways})$"]({box});'
        f'way["footway"="crossing"]({box}););'
        "out geom;"
    )
    request = Request(
        overpass_url,
        data=urlencode({"data": query}).encode(),
        headers={"User-Agent": "autonomy_datasets"},
    )

    def query_ways() -> List[Dict[str, Any]]:
        with urlopen(request, timeout=_HTTP_TIMEOUT_SECONDS) as response:
            return [element for element in json.load(response).get("elements", []) if element.get("type") == "way"]

    for retry_delay in _OVERPASS_RETRY_DELAYS_SECONDS:
        try:
            return query_ways()
        except HTTPError as error:
            if error.code not in _OVERPASS_BUSY_STATUS_CODES:
                raise
            LOGGER.info(f"Overpass API is busy (HTTP {error.code}); retrying in {retry_delay:.0f}s")
            time.sleep(retry_delay)
    return query_ways()


def osm_roads_to_lanelet2_osm(
    ways: Sequence[Dict[str, Any]],
    georeference: SceneGeoreference,
    lane_width: float = 3.5,
) -> str:
    """Convert OSM road and crossing ways into a Lanelet2 OSM XML string in the ``map`` frame of a scene.

    Args:
        ways: OSM ways with their ``tags`` and node ``geometry``, as returned by :func:`fetch_osm_roads`.
        georeference: Georeference of the scene the map is generated for.
        lane_width: Assumed lane width in meters for synthesizing the lane boundaries.

    Returns:
        The Lanelet2 map serialized as an OSM XML string.
    """
    builder = Lanelet2OsmBuilder(georeference.origin_lat, georeference.origin_lon)
    for way in ways:
        tags = way.get("tags", {})
        points = _way_points(way, georeference)
        if points is None:
            continue
        if tags.get("footway") == "crossing":
            _add_crosswalk_lanelet(builder, points)
        elif tags.get("highway") in _ROAD_CLASSES:
            _add_road_lanelets(builder, points, tags, lane_width)
    return builder.to_string()


def _way_points(way: Dict[str, Any], georeference: SceneGeoreference) -> Optional[np.ndarray]:
    """Return the ``(N, 3)`` points of an OSM way in ``map``, or None if it does not form a line."""
    points: List[Tuple[float, float, float]] = []
    for node in way.get("geometry") or []:
        if not node:
            continue
        point = georeference.to_map(node["lat"], node["lon"])
        if points and math.dist(point[:2], points[-1][:2]) < _MIN_POINT_DISTANCE:
            continue
        points.append(point)
    return np.array(points) if len(points) >= 2 else None


def _offset_polyline(points: np.ndarray, offset: float) -> np.ndarray:
    """Offset a polyline to its left by ``offset`` meters, or to its right if negative.

    Each vertex is shifted along the bisector of its adjacent segments, scaled so that the offset
    holds perpendicular to both segments and the lanes keep their width around bends.
    """
    directions = np.diff(points[:, :2], axis=0)
    directions /= np.linalg.norm(directions, axis=1, keepdims=True)
    segment_normals = np.column_stack([-directions[:, 1], directions[:, 0]])
    # the segment each vertex takes its normal from if its adjacent segments point in opposite directions
    vertex_segment_normals = np.vstack([segment_normals, segment_normals[-1:]])
    bisectors = np.vstack([segment_normals[:1], segment_normals[:-1] + segment_normals[1:], segment_normals[-1:]])
    lengths = np.linalg.norm(bisectors, axis=1, keepdims=True)
    bisectors = np.where(lengths > 1e-6, bisectors / np.maximum(lengths, 1e-6), vertex_segment_normals)
    # limits the miter at sharp bends to twice the offset
    cos_half_angle = np.maximum(np.sum(bisectors * vertex_segment_normals, axis=1), 0.5)
    shifted = points.copy()
    shifted[:, :2] += offset * bisectors / cos_half_angle[:, None]
    return shifted


def _lane_count(value: Optional[str]) -> Optional[int]:
    """Parse an OSM lane count tag; None if it is missing or invalid."""
    try:
        count = int(str(value).split(";")[0].strip())
    except ValueError:
        return None
    return count if 0 < count <= _MAX_LANES else None


def _lane_layout(tags: Dict[str, str]) -> Tuple[int, int, bool]:
    """Return the lanes of a road way along and against its direction, and whether they are one-way.

    A two-way road tagged with a single lane is laid out as one lanelet that is passable in both
    directions.
    """
    lanes = _lane_count(tags.get("lanes"))
    oneway = tags.get("oneway")
    implied_oneway = tags.get("highway") in _IMPLIED_ONEWAY_HIGHWAYS or tags.get("junction") in _IMPLIED_ONEWAY_JUNCTIONS
    if oneway == "-1":
        return 0, lanes or 1, True
    if oneway in ("yes", "true", "1") or (oneway is None and implied_oneway):
        return lanes or (2 if tags.get("highway") == "motorway" else 1), 0, True

    forward = _lane_count(tags.get("lanes:forward"))
    backward = _lane_count(tags.get("lanes:backward"))
    if forward is None and backward is None:
        if lanes == 1:
            return 1, 0, False
        backward = (lanes or 2) // 2
        forward = (lanes or 2) - backward
    elif forward is None:
        forward = max(lanes - backward, 1) if lanes else 1
    elif backward is None:
        backward = max(lanes - forward, 1) if lanes else 1
    return forward, backward, True


def _add_road_lanelets(builder: Lanelet2OsmBuilder, points: np.ndarray, tags: Dict[str, str], lane_width: float) -> None:
    """Add the lanelets of an OSM road way, laid out around its centerline for right-hand traffic."""
    forward, backward, one_way = _lane_layout(tags)
    lanes = forward + backward
    subtype, location = _ROAD_CLASSES[tags["highway"]]
    lanelet_tags = [("type", "lanelet"), ("subtype", subtype), ("location", location), ("one_way", "yes" if one_way else "no")]

    # Boundary k runs (lanes / 2 - k) lane widths left of the centerline: boundaries 0 to backward
    # enclose the lanes against the way direction, boundaries backward to lanes the lanes along it.
    boundary_nodes = [
        [builder.add_node(x, y, z) for x, y, z in _offset_polyline(points, (lanes / 2 - k) * lane_width)]
        for k in range(lanes + 1)
    ]

    def add_boundary(k: int, reverse: bool) -> int:
        subtype = "solid" if k in (0, lanes) else "dashed"
        node_ids = boundary_nodes[k][::-1] if reverse else boundary_nodes[k]
        return builder.add_way(node_ids, [("type", "line_thin"), ("subtype", subtype)])

    # A lanelet drives along its bounds, so the lanes against the way direction get reversed
    # boundaries of their own, which share the nodes of the lanes along it.
    along = {k: add_boundary(k, reverse=False) for k in range(backward, lanes + 1)} if forward else {}
    against = {k: add_boundary(k, reverse=True) for k in range(backward + 1)} if backward else {}
    for k in range(backward, lanes):
        builder.add_relation([("way", along[k], "left"), ("way", along[k + 1], "right")], lanelet_tags)
    for k in range(backward):
        builder.add_relation([("way", against[k + 1], "left"), ("way", against[k], "right")], lanelet_tags)


def _add_crosswalk_lanelet(builder: Lanelet2OsmBuilder, points: np.ndarray) -> None:
    """Add a crosswalk lanelet around the centerline of an OSM crossing way."""
    boundaries = []
    for offset in (_CROSSWALK_WIDTH / 2, -_CROSSWALK_WIDTH / 2):
        node_ids = [builder.add_node(x, y, z) for x, y, z in _offset_polyline(points, offset)]
        boundaries.append(builder.add_way(node_ids, [("type", "line_thin"), ("subtype", "dashed")]))
    builder.add_relation(
        [("way", boundaries[0], "left"), ("way", boundaries[1], "right")],
        [("type", "lanelet"), ("subtype", "crosswalk"), ("location", "urban"), ("one_way", "no")],
    )
