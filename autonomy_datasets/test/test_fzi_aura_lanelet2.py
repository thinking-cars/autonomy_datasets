# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

"""Unit tests for the Lanelet2 maps the FZI-AURA adapter generates from OpenStreetMap.

The tests run offline: the georeference is fitted to a synthetic GNSS track, and the conversion is
fed hand-written OSM ways instead of an Overpass API response.
"""

import math
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from autonomy_datasets.datasets.fzi_aura.lanelet2_converter import (
    _lane_layout,
    _offset_polyline,
    osm_roads_to_lanelet2_osm,
    SceneGeoreference,
)
from autonomy_datasets.datasets.lanelet2_osm import geodetic_to_utm, utm_central_meridian_deg, UtmLocalProjector, WGS84_A, WGS84_F

LANE_WIDTH = 3.5


class _PlanarGeoreference:
    """Stands in for a SceneGeoreference whose ``map`` frame takes longitude as x and latitude as y in meters."""

    origin_lat = 49.0
    origin_lon = 8.4

    @staticmethod
    def to_map(lat, lon):
        return float(lon), float(lat), 0.0


def _way(points, **tags):
    """Return an OSM way as returned by the Overpass API, with ``(x, y)`` points in the planar map frame."""
    return {"type": "way", "tags": tags, "geometry": [{"lat": y, "lon": x} for x, y in points]}


def _parse_lanelets(osm):
    """Return ``(tags, left_points, right_points)`` of every lanelet of a Lanelet2 OSM string."""
    root = ET.fromstring(osm)

    def tags(element):
        return {tag.get("k"): tag.get("v") for tag in element.iter("tag")}

    nodes = {}
    for node in root.iter("node"):
        node_tags = tags(node)
        nodes[node.get("id")] = (float(node_tags["local_x"]), float(node_tags["local_y"]))
    ways = {way.get("id"): np.array([nodes[nd.get("ref")] for nd in way.iter("nd")]) for way in root.iter("way")}
    lanelets = []
    for relation in root.iter("relation"):
        bounds = {member.get("role"): ways[member.get("ref")] for member in relation.iter("member")}
        lanelets.append((tags(relation), bounds["left"], bounds["right"]))
    return lanelets


def _direction(bound):
    """Return the unit direction from the first to the last point of a bound."""
    direction = bound[-1] - bound[0]
    return direction / np.linalg.norm(direction)


class TestSceneGeoreference(unittest.TestCase):
    """The georeference relates GNSS fixes to the arbitrary map frame of the ego poses."""

    def setUp(self):
        """Build a curved track of 200 samples near Karlsruhe in an east-north-up map frame.

        The origin of the map frame lies kilometers away, as the origin of the released odometry does.
        """
        self.reference = (48.894, 8.318)
        self.offset = np.array([-7810.5, -12909.5])
        east = np.linspace(0.0, 150.0, 200)
        north = 0.002 * (east - 75.0) ** 2
        lat0 = math.radians(self.reference[0])
        e2 = WGS84_F * (2.0 - WGS84_F)
        meridian_radius = WGS84_A * (1 - e2) / (1 - e2 * math.sin(lat0) ** 2) ** 1.5
        normal_radius = WGS84_A / math.sqrt(1 - e2 * math.sin(lat0) ** 2)
        self.fixes = np.column_stack(
            [
                self.reference[0] + np.degrees(north / meridian_radius),
                self.reference[1] + np.degrees(east / (normal_radius * math.cos(lat0))),
            ]
        )
        self.positions = np.column_stack([east + self.offset[0], north + self.offset[1], np.linspace(-1880.0, -1870.0, 200)])

    def test_fixes_map_to_ego_positions(self):
        """Every GNSS fix lands on the ego position recorded with it."""
        georeference = SceneGeoreference(self.fixes, self.positions)
        self.assertLess(georeference.residual, 0.01)
        for (lat, lon), position in zip(self.fixes[::20], self.positions[::20]):
            np.testing.assert_allclose(georeference.to_map(lat, lon), position, atol=0.01)

    def test_origin_reproduces_first_fix(self):
        """Projecting the map frame back with lanelet2's UtmProjector yields the true position of the first fix."""
        georeference = SceneGeoreference(self.fixes, self.positions)
        projector = UtmLocalProjector(georeference.origin_lat, georeference.origin_lon)
        lat, lon = projector.to_geo(*self.positions[0, :2])
        # 1e-7 degrees are about a centimeter
        self.assertAlmostEqual(lat, self.fixes[0, 0], places=7)
        self.assertAlmostEqual(lon, self.fixes[0, 1], places=7)

    def test_map_nodes_invert_utm_projector(self):
        """A map node's lat/lon project back onto its map coordinates with the origin's UTM zone."""
        georeference = SceneGeoreference(self.fixes, self.positions)
        osm = osm_roads_to_lanelet2_osm(
            [{"tags": {"highway": "residential"}, "geometry": [{"lat": lat, "lon": lon} for lat, lon in self.fixes[::50]]}],
            georeference,
        )
        lon0 = utm_central_meridian_deg(georeference.origin_lon)
        origin = np.array(geodetic_to_utm(georeference.origin_lat, georeference.origin_lon, lon0))
        for node in ET.fromstring(osm).iter("node"):
            local = {tag.get("k"): float(tag.get("v")) for tag in node.iter("tag")}
            projected = np.array(geodetic_to_utm(float(node.get("lat")), float(node.get("lon")), lon0)) - origin
            np.testing.assert_allclose(projected, (local["local_x"], local["local_y"]), atol=0.001)

    def test_nodes_take_height_of_closest_ego_pose(self):
        """Map nodes take the height of the closest ego pose, as the map frame's height is arbitrary."""
        georeference = SceneGeoreference(self.fixes, self.positions)
        self.assertAlmostEqual(georeference.to_map(*self.fixes[-1])[2], self.positions[-1, 2], places=6)

    def test_fixes_without_gnss_are_ignored(self):
        """Samples without a GNSS fix are left out of the fit; a scene without any cannot be georeferenced."""
        fixes = self.fixes.copy()
        fixes[:100] = np.nan
        fixes[100] = 0.0
        georeference = SceneGeoreference(fixes, self.positions)
        np.testing.assert_allclose(georeference.to_map(*self.fixes[150]), self.positions[150], atol=0.01)
        with self.assertRaises(ValueError):
            SceneGeoreference(np.full_like(self.fixes, np.nan), self.positions)

    def test_bounding_box_covers_track_with_margin(self):
        """The bounding box extends the GNSS track by the margin in every direction."""
        south, west, north, east = SceneGeoreference(self.fixes, self.positions).bounding_box(200.0)
        self.assertAlmostEqual((self.fixes[:, 0].min() - south) * 111_200.0, 200.0, delta=2.0)
        self.assertAlmostEqual((north - self.fixes[:, 0].max()) * 111_200.0, 200.0, delta=2.0)
        self.assertAlmostEqual((east - self.fixes[:, 1].max()) * 111_200.0 * math.cos(math.radians(48.894)), 200.0, delta=2.0)
        self.assertLess(west, self.fixes[:, 1].min())


class TestLaneLayout(unittest.TestCase):
    """The lane layout follows the OpenStreetMap tagging conventions."""

    def test_layouts(self):
        """Lanes along and against the way direction, and whether the lanelets are one-way."""
        cases = [
            ({"highway": "residential"}, (1, 1, True)),
            ({"highway": "residential", "lanes": "1"}, (1, 0, False)),
            ({"highway": "residential", "lanes": "invalid"}, (1, 1, True)),
            ({"highway": "primary", "lanes": "4"}, (2, 2, True)),
            ({"highway": "primary", "lanes": "3"}, (2, 1, True)),
            ({"highway": "primary", "lanes": "3", "lanes:backward": "2"}, (1, 2, True)),
            ({"highway": "primary", "lanes:forward": "2"}, (2, 1, True)),
            ({"highway": "motorway"}, (2, 0, True)),
            ({"highway": "motorway", "lanes": "3"}, (3, 0, True)),
            ({"highway": "motorway", "oneway": "no"}, (1, 1, True)),
            ({"highway": "secondary", "oneway": "yes", "lanes": "2"}, (2, 0, True)),
            ({"highway": "secondary", "oneway": "-1"}, (0, 1, True)),
            ({"highway": "tertiary", "junction": "roundabout"}, (1, 0, True)),
        ]
        for tags, layout in cases:
            with self.subTest(tags=tags):
                self.assertEqual(_lane_layout(tags), layout)


class TestRoadConversion(unittest.TestCase):
    """OSM roads and crossings become lanelets laid out for right-hand traffic."""

    def test_two_way_road(self):
        """A two-way road gets one lane per direction, each driving on the right of the centerline."""
        lanelets = _parse_lanelets(
            osm_roads_to_lanelet2_osm([_way([(0, 0), (100, 0)], highway="residential")], _PlanarGeoreference())
        )
        self.assertEqual(len(lanelets), 2)
        for tags, left, right in lanelets:
            self.assertEqual((tags["type"], tags["subtype"], tags["one_way"]), ("lanelet", "road", "yes"))
            np.testing.assert_allclose(_direction(left), _direction(right), atol=1e-9)
            # the left bound of a lane in right-hand traffic is the centerline
            np.testing.assert_allclose(np.abs(left[:, 1]), 0.0, atol=1e-9)
            heading_east = _direction(left)[0] > 0
            np.testing.assert_allclose(right[:, 1], -LANE_WIDTH if heading_east else LANE_WIDTH, atol=1e-9)

    def test_lanes_share_their_boundaries(self):
        """Adjacent lanes of the same direction share a boundary, so that lane changes are possible."""
        lanelets = _parse_lanelets(
            osm_roads_to_lanelet2_osm([_way([(0, 0), (100, 0)], highway="primary", lanes="4")], _PlanarGeoreference())
        )
        eastbound = sorted(
            (lanelet for lanelet in lanelets if _direction(lanelet[1])[0] > 0), key=lambda lanelet: -lanelet[1][0, 1]
        )
        self.assertEqual(len(eastbound), 2)
        np.testing.assert_allclose(eastbound[0][2], eastbound[1][1])
        self.assertAlmostEqual(eastbound[1][2][0, 1], -2 * LANE_WIDTH)

    def test_reversed_one_way_road(self):
        """A road tagged oneway=-1 is driven against the direction of its way."""
        lanelets = _parse_lanelets(
            osm_roads_to_lanelet2_osm([_way([(0, 0), (100, 0)], highway="tertiary", oneway="-1")], _PlanarGeoreference())
        )
        self.assertEqual(len(lanelets), 1)
        _, left, right = lanelets[0]
        self.assertLess(_direction(left)[0], 0)
        self.assertAlmostEqual(left[0, 1], -LANE_WIDTH / 2)
        self.assertAlmostEqual(right[0, 1], LANE_WIDTH / 2)

    def test_single_lane_two_way_road(self):
        """A two-way road with a single lane is one lanelet passable in both directions."""
        lanelets = _parse_lanelets(
            osm_roads_to_lanelet2_osm([_way([(0, 0), (100, 0)], highway="service", lanes="1")], _PlanarGeoreference())
        )
        self.assertEqual([tags["one_way"] for tags, _, _ in lanelets], ["no"])

    def test_motorway_and_crossing(self):
        """Motorways become highway lanelets, crossings crosswalks; other ways are ignored."""
        ways = [
            _way([(0, 0), (100, 0)], highway="motorway", lanes="3"),
            _way([(50, -10), (50, 10)], highway="footway", footway="crossing"),
            _way([(0, 20), (100, 20)], highway="cycleway"),
            _way([(0, 30)], highway="residential"),
        ]
        lanelets = _parse_lanelets(osm_roads_to_lanelet2_osm(ways, _PlanarGeoreference()))
        subtypes = sorted((tags["subtype"], tags["location"], tags["one_way"]) for tags, _, _ in lanelets)
        self.assertEqual(subtypes, [("crosswalk", "urban", "no")] + [("highway", "nonurban", "yes")] * 3)


class TestOffsetPolyline(unittest.TestCase):
    """Lane boundaries keep their distance to the centerline around bends."""

    def test_offset_keeps_distance_around_bend(self):
        """Every offset vertex keeps the offset to both segments adjacent to its centerline vertex."""
        points = np.array([(0.0, 0.0, 0.0), (10.0, 0.0, 0.0), (10.0, 10.0, 0.0)])
        for offset in (1.5, -1.5):
            shifted = _offset_polyline(points, offset)
            np.testing.assert_allclose(shifted[0, :2], (0.0, offset))
            np.testing.assert_allclose(shifted[1, :2], (10.0 - offset, offset))
            np.testing.assert_allclose(shifted[2, :2], (10.0 - offset, 10.0))

    def test_offset_of_reversing_polyline(self):
        """A polyline that doubles back on itself yields finite boundaries."""
        points = np.array([(0.0, 0.0, 0.0), (10.0, 0.0, 0.0), (0.0, 0.0, 0.0)])
        self.assertTrue(np.all(np.isfinite(_offset_polyline(points, 1.0))))
