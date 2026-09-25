# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

"""Shared building blocks for serializing Lanelet2 maps as OSM XML.

Downstream ROS components such as the ``lanelet2_map_server`` load the map with lanelet2's
``UtmProjector`` anchored at the map origin, so the dataset adapters generate their maps in the
metric ``map`` frame and project every node to the geodetic (lat/lon) *inverse* of that exact
projection. The map then lines up with the metric ego frame regardless of how coarse the origin
is. Nodes additionally carry their metric coordinates as ``local_x``/``local_y`` tags, so the
result is usable by both metric and geographic Lanelet2 loaders.
"""

import math
import xml.etree.ElementTree as ET
from typing import List, Tuple

# WGS84 ellipsoid and UTM constants. The Lanelet2 map server loads the map with
# lanelet2's ``UtmProjector``, so node coordinates must be the geodetic (lat/lon)
# *inverse* of that exact projection for the map to line up with the metric ego
# frame. A naive spherical/equirectangular projection is off by the UTM scale
# factor (~0.2 %), which grows to several meters far from the map origin (the
# nuScenes local origin is a map corner, so the ego is often kilometers away).
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
UTM_K0 = 0.9996
UTM_FALSE_EASTING = 500000.0


def utm_central_meridian_deg(lon_deg: float) -> float:
    """Return the central meridian (deg) of the standard UTM zone for a longitude."""
    zone = int(math.floor((lon_deg + 180.0) / 6.0)) + 1
    return (zone - 1) * 6.0 - 180.0 + 3.0


def geodetic_to_utm(lat_deg: float, lon_deg: float, lon0_deg: float) -> Tuple[float, float]:
    """Transverse Mercator (UTM) forward, WGS84, using the Snyder series.

    Returns ``(easting, northing)`` for the zone whose central meridian is
    ``lon0_deg``. The false northing is intentionally omitted; only differences
    from the origin are used, so the constant cancels.
    """
    e2 = WGS84_F * (2.0 - WGS84_F)
    ep2 = e2 / (1.0 - e2)
    lat = math.radians(lat_deg)
    dlon = math.radians(lon_deg - lon0_deg)
    sin_lat, cos_lat, tan_lat = math.sin(lat), math.cos(lat), math.tan(lat)
    n = WGS84_A / math.sqrt(1.0 - e2 * sin_lat**2)
    t = tan_lat**2
    c = ep2 * cos_lat**2
    a = dlon * cos_lat
    m = WGS84_A * (
        (1 - e2 / 4 - 3 * e2**2 / 64 - 5 * e2**3 / 256) * lat
        - (3 * e2 / 8 + 3 * e2**2 / 32 + 45 * e2**3 / 1024) * math.sin(2 * lat)
        + (15 * e2**2 / 256 + 45 * e2**3 / 1024) * math.sin(4 * lat)
        - (35 * e2**3 / 3072) * math.sin(6 * lat)
    )
    easting = UTM_K0 * n * (a + (1 - t + c) * a**3 / 6 + (5 - 18 * t + t**2 + 72 * c - 58 * ep2) * a**5 / 120) + UTM_FALSE_EASTING
    northing = UTM_K0 * (
        m
        + n
        * tan_lat
        * (a**2 / 2 + (5 - t + 9 * c + 4 * c**2) * a**4 / 24 + (61 - 58 * t + t**2 + 600 * c - 330 * ep2) * a**6 / 720)
    )
    return easting, northing


def utm_to_geodetic(easting: float, northing: float, lon0_deg: float) -> Tuple[float, float]:
    """Transverse Mercator (UTM) inverse, WGS84, using the Snyder series.

    Inverse of :func:`geodetic_to_utm` for the zone with central meridian
    ``lon0_deg``.
    """
    e2 = WGS84_F * (2.0 - WGS84_F)
    ep2 = e2 / (1.0 - e2)
    x = easting - UTM_FALSE_EASTING
    m = northing / UTM_K0
    mu = m / (WGS84_A * (1 - e2 / 4 - 3 * e2**2 / 64 - 5 * e2**3 / 256))
    e1 = (1 - math.sqrt(1 - e2)) / (1 + math.sqrt(1 - e2))
    phi1 = (
        mu
        + (3 * e1 / 2 - 27 * e1**3 / 32) * math.sin(2 * mu)
        + (21 * e1**2 / 16 - 55 * e1**4 / 32) * math.sin(4 * mu)
        + (151 * e1**3 / 96) * math.sin(6 * mu)
        + (1097 * e1**4 / 512) * math.sin(8 * mu)
    )
    sin_phi1, cos_phi1, tan_phi1 = math.sin(phi1), math.cos(phi1), math.tan(phi1)
    n1 = WGS84_A / math.sqrt(1 - e2 * sin_phi1**2)
    t1 = tan_phi1**2
    c1 = ep2 * cos_phi1**2
    r1 = WGS84_A * (1 - e2) / (1 - e2 * sin_phi1**2) ** 1.5
    d = x / (n1 * UTM_K0)
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


class UtmLocalProjector:
    """Maps local metric map coordinates to WGS84 lat/lon.

    The lat/lon are chosen so that lanelet2's ``UtmProjector`` (anchored at the
    same ``(origin_lat, origin_lon)``) maps them back to the original local
    coordinates, keeping the map aligned with the metric ego frame. Both the
    origin and the nodes use the origin's UTM zone, matching how ``UtmProjector``
    forces points into the origin's zone.
    """

    def __init__(self, origin_lat: float, origin_lon: float) -> None:
        """Initialize the projector with the geographic map origin."""
        self._lon0 = utm_central_meridian_deg(origin_lon)
        self._origin_easting, self._origin_northing = geodetic_to_utm(origin_lat, origin_lon, self._lon0)

    def to_geo(self, x: float, y: float) -> Tuple[float, float]:
        """Project a local metric point (x=east, y=north) to ``(lat, lon)``."""
        return utm_to_geodetic(self._origin_easting + x, self._origin_northing + y, self._lon0)


class Lanelet2OsmBuilder:
    """Incrementally builds a Lanelet2 OSM XML document.

    Element ids are unique across all primitive types (nodes, ways, relations),
    which satisfies the Lanelet2 requirement of per-type unique ids.
    """

    def __init__(self, origin_lat: float, origin_lon: float) -> None:
        """Initialize the builder with the geographic map origin."""
        self._origin_lat = origin_lat
        self._origin_lon = origin_lon
        self._projector = UtmLocalProjector(origin_lat, origin_lon)
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
