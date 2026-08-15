# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

"""Convert nuScenes map-expansion maps to Lanelet2 (OSM XML) maps.

The nuScenes dataset ships its maps in a custom semantic format (the
``map_expansion`` layers). Downstream ROS components such as the
``lanelet2_map_server`` expect a Lanelet2 map serialized as an OSM XML
string. This module bridges the two by discretizing the nuScenes lane graph
and pedestrian crossings into Lanelet2 lanelets.

Lanes (``lane`` and ``lane_connector``) carry a centerline but no explicit
left/right boundary references in nuScenes, so the boundaries are synthesized
by offsetting the discretized centerline perpendicular to the heading by half
the configured lane width. Pedestrian crossings (``ped_crossing``) are emitted
as ``crosswalk`` lanelets built from their (quadrilateral) polygon.

Nodes carry both ``local_x``/``local_y`` tags (the native nuScenes metric map
frame, x=east, y=north) and ``lat``/``lon`` attributes derived from a coarse
per-location geographic origin, so the result is usable by both metric
and geographic Lanelet2 loaders.
"""

import math
import xml.etree.ElementTree as ET
from typing import Any, List, Optional, Tuple

# Approximate WGS84 geographic origins for the nuScenes map locations. These
# anchor the (otherwise purely local) nuScenes metric map frame so that the
# resulting Lanelet2 map can also be loaded by geographic projectors. The
# values are intentionally coarse; the authoritative geometry lives in the
# ``local_x``/``local_y`` tags.
_LOCATION_ORIGINS = {
    "boston-seaport": (42.336, -71.058),
    "singapore-onenorth": (1.2882, 103.7891),
    "singapore-hollandvillage": (1.3098, 103.7935),
    "singapore-queenstown": (1.2934, 103.7843),
}

# Default geographic origin used when a location is unknown.
_DEFAULT_ORIGIN = (0.0, 0.0)

# WGS84 ellipsoid and UTM constants. The Lanelet2 map server loads the map with
# lanelet2's ``UtmProjector``, so node coordinates must be the geodetic (lat/lon)
# *inverse* of that exact projection for the map to line up with the metric ego
# frame. A naive spherical/equirectangular projection is off by the UTM scale
# factor (~0.2 %), which grows to several meters far from the map origin (the
# nuScenes local origin is a map corner, so the ego is often kilometers away).
_WGS84_A = 6378137.0
_WGS84_F = 1.0 / 298.257223563
_UTM_K0 = 0.9996
_UTM_FALSE_EASTING = 500000.0


def _utm_central_meridian_deg(lon_deg: float) -> float:
    """Return the central meridian (deg) of the standard UTM zone for a longitude."""
    zone = int(math.floor((lon_deg + 180.0) / 6.0)) + 1
    return (zone - 1) * 6.0 - 180.0 + 3.0


def _geodetic_to_utm(lat_deg: float, lon_deg: float, lon0_deg: float) -> Tuple[float, float]:
    """Transverse Mercator (UTM) forward, WGS84, using the Snyder series.

    Returns ``(easting, northing)`` for the zone whose central meridian is
    ``lon0_deg``. The false northing is intentionally omitted; only differences
    from the origin are used, so the constant cancels.
    """
    e2 = _WGS84_F * (2.0 - _WGS84_F)
    ep2 = e2 / (1.0 - e2)
    lat = math.radians(lat_deg)
    dlon = math.radians(lon_deg - lon0_deg)
    sin_lat, cos_lat, tan_lat = math.sin(lat), math.cos(lat), math.tan(lat)
    n = _WGS84_A / math.sqrt(1.0 - e2 * sin_lat**2)
    t = tan_lat**2
    c = ep2 * cos_lat**2
    a = dlon * cos_lat
    m = _WGS84_A * (
        (1 - e2 / 4 - 3 * e2**2 / 64 - 5 * e2**3 / 256) * lat
        - (3 * e2 / 8 + 3 * e2**2 / 32 + 45 * e2**3 / 1024) * math.sin(2 * lat)
        + (15 * e2**2 / 256 + 45 * e2**3 / 1024) * math.sin(4 * lat)
        - (35 * e2**3 / 3072) * math.sin(6 * lat)
    )
    easting = (
        _UTM_K0 * n * (a + (1 - t + c) * a**3 / 6 + (5 - 18 * t + t**2 + 72 * c - 58 * ep2) * a**5 / 120) + _UTM_FALSE_EASTING
    )
    northing = _UTM_K0 * (
        m
        + n
        * tan_lat
        * (a**2 / 2 + (5 - t + 9 * c + 4 * c**2) * a**4 / 24 + (61 - 58 * t + t**2 + 600 * c - 330 * ep2) * a**6 / 720)
    )
    return easting, northing


def _utm_to_geodetic(easting: float, northing: float, lon0_deg: float) -> Tuple[float, float]:
    """Transverse Mercator (UTM) inverse, WGS84, using the Snyder series.

    Inverse of :func:`_geodetic_to_utm` for the zone with central meridian
    ``lon0_deg``.
    """
    e2 = _WGS84_F * (2.0 - _WGS84_F)
    ep2 = e2 / (1.0 - e2)
    x = easting - _UTM_FALSE_EASTING
    m = northing / _UTM_K0
    mu = m / (_WGS84_A * (1 - e2 / 4 - 3 * e2**2 / 64 - 5 * e2**3 / 256))
    e1 = (1 - math.sqrt(1 - e2)) / (1 + math.sqrt(1 - e2))
    phi1 = (
        mu
        + (3 * e1 / 2 - 27 * e1**3 / 32) * math.sin(2 * mu)
        + (21 * e1**2 / 16 - 55 * e1**4 / 32) * math.sin(4 * mu)
        + (151 * e1**3 / 96) * math.sin(6 * mu)
        + (1097 * e1**4 / 512) * math.sin(8 * mu)
    )
    sin_phi1, cos_phi1, tan_phi1 = math.sin(phi1), math.cos(phi1), math.tan(phi1)
    n1 = _WGS84_A / math.sqrt(1 - e2 * sin_phi1**2)
    t1 = tan_phi1**2
    c1 = ep2 * cos_phi1**2
    r1 = _WGS84_A * (1 - e2) / (1 - e2 * sin_phi1**2) ** 1.5
    d = x / (n1 * _UTM_K0)
    lat = phi1 - (n1 * tan_phi1 / r1) * (
        d**2 / 2
        - (5 + 3 * t1 + 10 * c1 - 4 * c1**2 - 9 * ep2) * d**4 / 24
        + (61 + 90 * t1 + 298 * c1 + 45 * t1**2 - 252 * ep2 - 3 * c1**2) * d**6 / 720
    )
    lon = (
        math.radians(lon0_deg)
        + (d - (1 + 2 * t1 + c1) * d**3 / 6 + (5 - 2 * c1 + 28 * t1 - 3 * c1**2 + 8 * ep2 + 24 * t1**2) * d**5 / 120) / cos_phi1
    )
    return math.degrees(lat), math.degrees(lon)


class _UtmLocalProjector:
    """Maps nuScenes local metric coordinates to WGS84 lat/lon.

    The lat/lon are chosen so that lanelet2's ``UtmProjector`` (anchored at the
    same ``(origin_lat, origin_lon)``) maps them back to the original local
    coordinates, keeping the map aligned with the metric ego frame. Both the
    origin and the nodes use the origin's UTM zone, matching how ``UtmProjector``
    forces points into the origin's zone.
    """

    def __init__(self, origin_lat: float, origin_lon: float) -> None:
        self._lon0 = _utm_central_meridian_deg(origin_lon)
        self._origin_easting, self._origin_northing = _geodetic_to_utm(origin_lat, origin_lon, self._lon0)

    def to_geo(self, x: float, y: float) -> Tuple[float, float]:
        """Project a local metric point (x=east, y=north) to ``(lat, lon)``."""
        return _utm_to_geodetic(self._origin_easting + x, self._origin_northing + y, self._lon0)


def get_location_origin(location: str) -> Tuple[float, float]:
    """Return the (lat, lon) geographic origin anchoring a nuScenes location.

    Args:
        location: The nuScenes location name (e.g. ``"boston-seaport"``).

    Returns:
        The ``(origin_lat, origin_lon)`` origin in WGS84 degrees.
    """
    return _LOCATION_ORIGINS.get(location, _DEFAULT_ORIGIN)


class _OsmBuilder:
    """Incrementally builds a Lanelet2 OSM XML document.

    Element ids are unique across all primitive types (nodes, ways, relations),
    which satisfies the Lanelet2 requirement of per-type unique ids.
    """

    def __init__(self, origin_lat: float, origin_lon: float) -> None:
        self._origin_lat = origin_lat
        self._origin_lon = origin_lon
        self._projector = _UtmLocalProjector(origin_lat, origin_lon)
        self._next_id = 1
        self._root = ET.Element("osm", {"version": "0.6", "generator": "autonomy_datasets"})

    def _new_id(self) -> int:
        element_id = self._next_id
        self._next_id += 1
        return element_id

    def _local_to_geo(self, x: float, y: float) -> Tuple[float, float]:
        """Project a local metric coordinate (x=east, y=north) to lat/lon."""
        return self._projector.to_geo(x, y)

    def add_node(self, x: float, y: float, z: float = 0.0) -> int:
        """Add an OSM node for a local metric point and return its id."""
        node_id = self._new_id()
        lat, lon = self._local_to_geo(x, y)
        node = ET.SubElement(
            self._root,
            "node",
            {"id": str(node_id), "lat": f"{lat:.12f}", "lon": f"{lon:.12f}"},
        )
        _add_tag(node, "local_x", f"{x:.4f}")
        _add_tag(node, "local_y", f"{y:.4f}")
        _add_tag(node, "ele", f"{z:.4f}")
        return node_id

    def add_way(self, node_ids: List[int], tags: List[Tuple[str, str]]) -> int:
        """Add an OSM way referencing the given nodes and return its id."""
        way_id = self._new_id()
        way = ET.SubElement(self._root, "way", {"id": str(way_id)})
        for node_id in node_ids:
            ET.SubElement(way, "nd", {"ref": str(node_id)})
        for key, value in tags:
            _add_tag(way, key, value)
        return way_id

    def add_relation(self, members: List[Tuple[str, int, str]], tags: List[Tuple[str, str]]) -> int:
        """Add an OSM relation and return its id.

        Args:
            members: List of ``(member_type, ref_id, role)`` tuples.
            tags: List of ``(key, value)`` tag tuples.
        """
        relation_id = self._new_id()
        relation = ET.SubElement(self._root, "relation", {"id": str(relation_id)})
        for member_type, ref_id, role in members:
            ET.SubElement(relation, "member", {"type": member_type, "ref": str(ref_id), "role": role})
        for key, value in tags:
            _add_tag(relation, key, value)
        return relation_id

    def to_string(self) -> str:
        """Serialize the document to an XML string with a declaration."""
        return ET.tostring(self._root, encoding="unicode", xml_declaration=True)


def _add_tag(parent: ET.Element, key: str, value: str) -> None:
    ET.SubElement(parent, "tag", {"k": key, "v": value})


def _offset_centerline(
    centerline: List[Tuple[float, float, float]], half_width: float
) -> Tuple[List[Tuple[float, float]], List[Tuple[float, float]]]:
    """Offset a discretized centerline to the left and right boundaries.

    Args:
        centerline: List of ``(x, y, yaw)`` poses along the lane centerline.
        half_width: Half the lane width in meters.

    Returns:
        Tuple of ``(left_points, right_points)`` as lists of ``(x, y)``.
    """
    left_points: List[Tuple[float, float]] = []
    right_points: List[Tuple[float, float]] = []
    for x, y, yaw in centerline:
        # Left normal of heading yaw is (-sin(yaw), cos(yaw)).
        nx, ny = -math.sin(yaw), math.cos(yaw)
        left_points.append((x + half_width * nx, y + half_width * ny))
        right_points.append((x - half_width * nx, y - half_width * ny))
    return left_points, right_points


def _add_boundary_way(builder: _OsmBuilder, points: List[Tuple[float, float]], subtype: str) -> int:
    node_ids = [builder.add_node(x, y) for x, y in points]
    return builder.add_way(node_ids, [("type", "line_thin"), ("subtype", subtype)])


def _polygon_points(nusc_map: Any, polygon_token: str) -> List[Tuple[float, float]]:
    """Return the exterior ring of a nuScenes polygon as ``(x, y)`` points."""
    polygon = nusc_map.get("polygon", polygon_token)
    points: List[Tuple[float, float]] = []
    for node_token in polygon["exterior_node_tokens"]:
        node = nusc_map.get("node", node_token)
        points.append((float(node["x"]), float(node["y"])))
    return points


def _add_lane_lanelet(
    builder: _OsmBuilder,
    centerline: List[Tuple[float, float, float]],
    half_width: float,
) -> Optional[int]:
    """Add a road lanelet from a discretized centerline. Returns relation id."""
    if len(centerline) < 2:
        return None
    left_points, right_points = _offset_centerline(centerline, half_width)
    left_way = _add_boundary_way(builder, left_points, "dashed")
    right_way = _add_boundary_way(builder, right_points, "dashed")
    return builder.add_relation(
        members=[("way", left_way, "left"), ("way", right_way, "right")],
        tags=[
            ("type", "lanelet"),
            ("subtype", "road"),
            ("location", "urban"),
            ("one_way", "yes"),
        ],
    )


def _add_crosswalk_lanelet(builder: _OsmBuilder, polygon: List[Tuple[float, float]]) -> Optional[int]:
    """Add a crosswalk lanelet from a quadrilateral ped-crossing polygon.

    The polygon corners are ordered around the ring, so opposite edges form the
    left and right boundaries of the crossing. Polygons that are not (close to)
    quadrilaterals are skipped to avoid emitting malformed lanelets.
    """
    # Drop a duplicated closing node if present.
    if len(polygon) >= 2 and polygon[0] == polygon[-1]:
        polygon = polygon[:-1]
    if len(polygon) != 4:
        return None
    p0, p1, p2, p3 = polygon
    left_way = _add_boundary_way(builder, [p0, p3], "dashed")
    right_way = _add_boundary_way(builder, [p1, p2], "dashed")
    return builder.add_relation(
        members=[("way", left_way, "left"), ("way", right_way, "right")],
        tags=[
            ("type", "lanelet"),
            ("subtype", "crosswalk"),
            ("location", "urban"),
            ("one_way", "no"),
        ],
    )


def nuscenes_map_to_lanelet2_osm(
    nusc_map: Any,
    location: str,
    lane_width: float = 3.0,
    resolution_meters: float = 1.0,
    include_crosswalks: bool = True,
) -> str:
    """Convert a ``NuScenesMap`` to a Lanelet2 OSM XML string.

    Args:
        nusc_map: A ``nuscenes.map_expansion.map_api.NuScenesMap`` instance.
        location: The nuScenes location name (used to pick a geographic origin).
        lane_width: Assumed lane width in meters for synthesizing boundaries.
        resolution_meters: Centerline discretization resolution in meters.
        include_crosswalks: Whether to emit pedestrian crossings as crosswalk
            lanelets.

    Returns:
        The Lanelet2 map serialized as an OSM XML string.
    """
    origin_lat, origin_lon = get_location_origin(location)
    builder = _OsmBuilder(origin_lat, origin_lon)
    half_width = lane_width / 2.0

    lane_tokens = [record["token"] for record in nusc_map.lane]
    lane_tokens += [record["token"] for record in nusc_map.lane_connector]
    for token in lane_tokens:
        try:
            centerline = nusc_map.discretize_lanes([token], resolution_meters)[token]
        except (KeyError, ValueError):
            continue
        _add_lane_lanelet(builder, centerline, half_width)

    if include_crosswalks:
        for record in nusc_map.ped_crossing:
            try:
                polygon = _polygon_points(nusc_map, record["polygon_token"])
            except KeyError:
                continue
            _add_crosswalk_lanelet(builder, polygon)

    return builder.to_string()
