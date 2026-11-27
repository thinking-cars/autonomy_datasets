# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

"""Unit tests for the Lanelet2 maps the nuScenes adapter generates for the surroundings of a scene.

The tests run offline: a stand-in for the ``NuScenesMap`` holds a few straight lanes and a
pedestrian crossing instead of the map expansion of a nuScenes location.
"""

import unittest
import xml.etree.ElementTree as ET

from autonomy_datasets.datasets.lanelet2_osm import LOCAL_MAP_MARGIN_METERS
from autonomy_datasets.datasets.nuscenes.lanelet2_converter import (
    nuscenes_map_to_lanelet2_osm,
    NuScenesLanelet2Map,
    scene_patch,
)


class _FakeNuScenesMap:
    """Stands in for a ``NuScenesMap`` with straight eastbound lanes and quadrilateral crossings."""

    def __init__(self, lanes, crossings=()):
        """Create the map from ``(token, start_x, end_x, y)`` lanes and ``(token, corners)`` crossings."""
        self._lanes = {token: (start_x, end_x, y) for token, start_x, end_x, y in lanes}
        self.lane = [{"token": token} for token in self._lanes]
        self.lane_connector = []
        self.ped_crossing = [{"token": token, "polygon_token": token} for token, _ in crossings]
        self._polygons = {token: [f"{token}_{index}" for index in range(len(corners))] for token, corners in crossings}
        self._nodes = {
            f"{token}_{index}": {"x": x, "y": y} for token, corners in crossings for index, (x, y) in enumerate(corners)
        }
        self.discretized_lanes = 0

    def discretize_lanes(self, tokens, resolution_meters):
        self.discretized_lanes += len(tokens)
        discretized = {}
        for token in tokens:
            start_x, end_x, y = self._lanes[token]
            count = int((end_x - start_x) / resolution_meters) + 1
            discretized[token] = [(start_x + index * resolution_meters, y, 0.0) for index in range(count)]
        return discretized

    def get(self, layer, token):
        if layer == "polygon":
            return {"exterior_node_tokens": self._polygons[token]}
        return self._nodes[token]


def _lanelets(osm: str):
    """Return the subtypes of the lanelets of a Lanelet2 map."""
    return [
        {tag.get("k"): tag.get("v") for tag in relation.iter("tag")}["subtype"]
        for relation in ET.fromstring(osm).iter("relation")
    ]


def _nodes_y(osm: str):
    """Return the ``local_y`` coordinates of the nodes of a Lanelet2 map."""
    return {
        float({tag.get("k"): tag.get("v") for tag in node.iter("tag")}["local_y"]) for node in ET.fromstring(osm).iter("node")
    }


class TestNuScenesLocalMap(unittest.TestCase):
    """A scene's map holds the lanelets near its ego trajectory only."""

    def setUp(self):
        """Create a map with a lane near the scene, a lane far away and a crossing near the scene."""
        self.nusc_map = _FakeNuScenesMap(
            lanes=[("near", 0.0, 50.0, 0.0), ("far", 0.0, 50.0, 3000.0)],
            crossings=[("crossing", [(10.0, -5.0), (14.0, -5.0), (14.0, 5.0), (10.0, 5.0)])],
        )
        self.lanelet2_map = NuScenesLanelet2Map(self.nusc_map, "boston-seaport")

    def test_scene_patch_extends_ego_trajectory_by_margin(self):
        """The patch is the bounding box of the ego positions, extended by the margin."""
        patch = scene_patch([[10.0, 20.0, 0.5], [30.0, -5.0, 0.4]])
        margin = LOCAL_MAP_MARGIN_METERS
        self.assertEqual(patch, (10.0 - margin, -5.0 - margin, 30.0 + margin, 20.0 + margin))

    def test_keeps_lanelets_overlapping_patch(self):
        """Lanes and crossings near the scene are kept, lanes far away from it are left out."""
        osm = self.lanelet2_map.to_osm(scene_patch([[20.0, 0.0, 0.0]]))
        self.assertEqual(_lanelets(osm), ["road", "crosswalk"])
        self.assertLess(max(_nodes_y(osm)), 100.0)

    def test_keeps_lanelets_overlapping_patch_whole(self):
        """A lanelet that only partly overlaps the patch is kept with all of its nodes."""
        osm = self.lanelet2_map.to_osm((45.0, -1.0, 60.0, 1.0))
        self.assertEqual(_lanelets(osm), ["road"])
        local_x = [
            float({tag.get("k"): tag.get("v") for tag in node.iter("tag")}["local_x"]) for node in ET.fromstring(osm).iter("node")
        ]
        self.assertEqual((min(local_x), max(local_x)), (0.0, 50.0))

    def test_keeps_lane_whose_boundary_overlaps_patch(self):
        """A lane is kept if only its synthesized boundary, offset from the centerline, reaches into the patch."""
        osm = self.lanelet2_map.to_osm((0.0, 1.2, 50.0, 10.0))
        self.assertIn("road", _lanelets(osm))

    def test_empty_patch_holds_no_lanelets(self):
        """A scene far away from every lane gets a map without lanelets."""
        self.assertEqual(_lanelets(self.lanelet2_map.to_osm(scene_patch([[10000.0, 10000.0, 0.0]]))), [])

    def test_discretizes_lanes_once_per_location(self):
        """The maps of the scenes of a location are converted from the lanes discretized once."""
        self.lanelet2_map.to_osm(scene_patch([[20.0, 0.0, 0.0]]))
        self.lanelet2_map.to_osm(scene_patch([[20.0, 3000.0, 0.0]]))
        self.assertEqual(self.nusc_map.discretized_lanes, 2)

    def test_converts_whole_map_without_patch(self):
        """Without a patch, the whole map is converted."""
        osm = nuscenes_map_to_lanelet2_osm(self.nusc_map, "boston-seaport")
        self.assertEqual(sorted(_lanelets(osm)), ["crosswalk", "road", "road"])


if __name__ == "__main__":
    unittest.main()
