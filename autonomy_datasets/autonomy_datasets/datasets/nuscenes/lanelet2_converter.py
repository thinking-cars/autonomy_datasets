# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

"""Convert nuScenes map-expansion maps to Lanelet2 (OSM XML) maps.

The nuScenes dataset ships its maps in a custom semantic format (the
``map_expansion`` layers). Downstream ROS components such as the
``lanelet2_map_server`` expect a Lanelet2 map serialized as an OSM XML
string. This module bridges the two by discretizing the nuScenes lane graph
and pedestrian crossings into Lanelet2 lanelets.

A nuScenes map covers a whole city district, which takes tens of MB as a
Lanelet2 map, while a scene covers a few hundred meters. A location's map is
therefore discretized once, and the map of a scene only holds the lanelets
overlapping the surroundings of its ego trajectory (see ``scene_patch``).

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
from typing import Any, Iterable, List, Optional, Sequence, Tuple

from autonomy_datasets.datasets.lanelet2_osm import Lanelet2OsmBuilder, LOCAL_MAP_MARGIN_METERS

# ``(min_x, min_y, max_x, max_y)`` bounds of an area of the nuScenes metric map frame in meters
Patch = Tuple[float, float, float, float]

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


def get_location_origin(location: str) -> Tuple[float, float]:
    """Return the (lat, lon) geographic origin anchoring a nuScenes location.

    Args:
        location: The nuScenes location name (e.g. ``"boston-seaport"``).

    Returns:
        The ``(origin_lat, origin_lon)`` origin in WGS84 degrees.
    """
    return _LOCATION_ORIGINS.get(location, _DEFAULT_ORIGIN)


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


def _add_boundary_way(builder: Lanelet2OsmBuilder, points: List[Tuple[float, float]], subtype: str) -> int:
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
    builder: Lanelet2OsmBuilder,
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


def _add_crosswalk_lanelet(builder: Lanelet2OsmBuilder, polygon: List[Tuple[float, float]]) -> Optional[int]:
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


def _bounds(points: Iterable[Sequence[float]]) -> Patch:
    """Return the bounds of the ``x`` and ``y`` coordinates of points."""
    xs, ys = zip(*((point[0], point[1]) for point in points))
    return min(xs), min(ys), max(xs), max(ys)


def _overlaps(bounds: Patch, patch: Optional[Patch]) -> bool:
    """Return whether bounds overlap a patch, which all bounds overlap if not given."""
    if patch is None:
        return True
    return bounds[0] <= patch[2] and patch[0] <= bounds[2] and bounds[1] <= patch[3] and patch[1] <= bounds[3]


def scene_patch(positions: Iterable[Sequence[float]], margin: float = LOCAL_MAP_MARGIN_METERS) -> Patch:
    """Return the patch of the map relevant for a scene: the bounds of its ego trajectory, extended by a margin.

    Args:
        positions: Positions of the ego vehicle in the nuScenes metric map frame, of which ``x`` and
            ``y`` are used.
        margin: Margin around the ego trajectory in meters.

    Returns:
        The ``(min_x, min_y, max_x, max_y)`` patch in meters.
    """
    min_x, min_y, max_x, max_y = _bounds(positions)
    return min_x - margin, min_y - margin, max_x + margin, max_y + margin


class NuScenesLanelet2Map:
    """Lanes and pedestrian crossings of a nuScenes map, converted to Lanelet2 maps of parts of it.

    The lanes are discretized once, so that the maps of the scenes recorded at a location are
    converted without discretizing the lanes again.
    """

    def __init__(
        self,
        nusc_map: Any,
        location: str,
        lane_width: float = 3.0,
        resolution_meters: float = 1.0,
        include_crosswalks: bool = True,
    ) -> None:
        """Discretize the lanes and pedestrian crossings of a nuScenes map.

        Args:
            nusc_map: A ``nuscenes.map_expansion.map_api.NuScenesMap`` instance.
            location: The nuScenes location name (used to pick a geographic origin).
            lane_width: Assumed lane width in meters for synthesizing boundaries.
            resolution_meters: Centerline discretization resolution in meters.
            include_crosswalks: Whether to emit pedestrian crossings as crosswalk
                lanelets.
        """
        self.origin_lat, self.origin_lon = get_location_origin(location)
        self._half_width = lane_width / 2.0

        # centerlines of the lanes, and the polygons of the pedestrian crossings, with their bounds
        self._lanes: List[Tuple[Patch, List[Tuple[float, float, float]]]] = []
        self._crosswalks: List[Tuple[Patch, List[Tuple[float, float]]]] = []

        lane_tokens = [record["token"] for record in nusc_map.lane]
        lane_tokens += [record["token"] for record in nusc_map.lane_connector]
        for token in lane_tokens:
            try:
                centerline = nusc_map.discretize_lanes([token], resolution_meters)[token]
            except (KeyError, ValueError):
                continue
            if len(centerline) >= 2:
                # the bounds of the boundaries, which are offset from the centerline
                min_x, min_y, max_x, max_y = _bounds(centerline)
                bounds = (
                    min_x - self._half_width,
                    min_y - self._half_width,
                    max_x + self._half_width,
                    max_y + self._half_width,
                )
                self._lanes.append((bounds, centerline))

        if include_crosswalks:
            for record in nusc_map.ped_crossing:
                try:
                    polygon = _polygon_points(nusc_map, record["polygon_token"])
                except KeyError:
                    continue
                if polygon:
                    self._crosswalks.append((_bounds(polygon), polygon))

    def to_osm(self, patch: Optional[Patch] = None) -> str:
        """Return the Lanelet2 map of the lanes and pedestrian crossings overlapping a patch of the map.

        Lanelets overlapping the patch are kept whole, so that they extend beyond it.

        Args:
            patch: ``(min_x, min_y, max_x, max_y)`` patch of the nuScenes metric map frame in
                meters, e.g. from ``scene_patch``. The whole map is converted if not given.

        Returns:
            The Lanelet2 map serialized as an OSM XML string.
        """
        builder = Lanelet2OsmBuilder(self.origin_lat, self.origin_lon)
        for bounds, centerline in self._lanes:
            if _overlaps(bounds, patch):
                _add_lane_lanelet(builder, centerline, self._half_width)
        for bounds, polygon in self._crosswalks:
            if _overlaps(bounds, patch):
                _add_crosswalk_lanelet(builder, polygon)
        return builder.to_string()


def nuscenes_map_to_lanelet2_osm(
    nusc_map: Any,
    location: str,
    lane_width: float = 3.0,
    resolution_meters: float = 1.0,
    include_crosswalks: bool = True,
    patch: Optional[Patch] = None,
) -> str:
    """Convert a ``NuScenesMap``, or a patch of it, to a Lanelet2 OSM XML string.

    Args:
        nusc_map: A ``nuscenes.map_expansion.map_api.NuScenesMap`` instance.
        location: The nuScenes location name (used to pick a geographic origin).
        lane_width: Assumed lane width in meters for synthesizing boundaries.
        resolution_meters: Centerline discretization resolution in meters.
        include_crosswalks: Whether to emit pedestrian crossings as crosswalk
            lanelets.
        patch: ``(min_x, min_y, max_x, max_y)`` patch of the map to convert, see
            ``NuScenesLanelet2Map.to_osm``; the whole map is converted if not given.

    Returns:
        The Lanelet2 map serialized as an OSM XML string.
    """
    return NuScenesLanelet2Map(nusc_map, location, lane_width, resolution_meters, include_crosswalks).to_osm(patch)
