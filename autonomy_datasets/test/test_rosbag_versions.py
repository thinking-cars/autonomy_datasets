# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

"""Unit tests for the version-specific rosbag directories.

Rosbags are stored in a subfolder named after the version of the dataset adapter that generated
them, so that a new adapter version generates its own rosbags instead of replaying or extending
the ones of an older version.
"""

import os
import shutil
import tempfile
import unittest

from autonomy_datasets.datasets.dataset import DatasetAdapter
from autonomy_datasets.datasets.driving.driving import DrivIngAdapter
from autonomy_datasets.datasets.nuscenes.nuscenes import NuscenesAdapter
from autonomy_datasets.datasets.nvidia_physicalai_av_dataset.nvidia_physicalai_av_dataset import (
    NvidiaPhysicalAiAvDatasetAdapter,
)
from autonomy_datasets.datasets.rosbag.rosbag import find_existing_rosbags, get_rosbag_root_dir
from autonomy_datasets.datasets.waymo_open_dataset.waymo_open_dataset import WaymoOpenDatasetAdapter

DATASET = "nuscenes"
SPLIT = "mini_val"
VERSION = "1.0.0"


class TestDatasetAdapterVersions(unittest.TestCase):
    """Every adapter must declare the version of its conversion without being instantiated."""

    def test_adapters_declare_a_version(self):
        """The version and its release notes are readable from the adapter class."""
        for adapter in (WaymoOpenDatasetAdapter, NuscenesAdapter, NvidiaPhysicalAiAvDatasetAdapter, DrivIngAdapter):
            with self.subTest(adapter=adapter.__name__):
                self.assertNotEqual(adapter.VERSION, DatasetAdapter.VERSION, "adapter does not declare its own version")
                self.assertIn(adapter.VERSION, adapter.RELEASE_NOTES, "version is missing from the release notes")


class TestVersionedRosbagDirectories(unittest.TestCase):
    """Rosbags are looked up in the subfolder of the requested adapter version."""

    def setUp(self):
        """Create a dataset directory holding rosbags of two versions and of the legacy layout."""
        self.dataset_path = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.dataset_path, ignore_errors=True)
        self.current_bag = self._create_bag(f"bags/{VERSION}/{DATASET}_{SPLIT}_00001_current")
        self.previous_bag = self._create_bag(f"bags/0.1.0/{DATASET}_{SPLIT}_00001_previous")
        self.legacy_bag = self._create_bag(f"bags/{DATASET}_{SPLIT}_00001_legacy")

    def _create_bag(self, relative_path):
        """Create an (empty) rosbag directory below the dataset directory."""
        bag_path = os.path.join(self.dataset_path, relative_path)
        os.makedirs(bag_path)
        return bag_path

    def test_rosbag_root_dir_is_version_specific(self):
        """The rosbag directory of a version is a subfolder of the unversioned one."""
        self.assertEqual(get_rosbag_root_dir(self.dataset_path), os.path.join(self.dataset_path, "bags"))
        self.assertEqual(get_rosbag_root_dir(self.dataset_path, VERSION), os.path.join(self.dataset_path, "bags", VERSION))

    def test_finds_only_rosbags_of_the_requested_version(self):
        """Rosbags of other versions and of the legacy layout are not returned."""
        self.assertEqual(find_existing_rosbags(self.dataset_path, DATASET, SPLIT, VERSION), [self.current_bag])
        self.assertEqual(find_existing_rosbags(self.dataset_path, DATASET, SPLIT, "0.1.0"), [self.previous_bag])
        self.assertEqual(find_existing_rosbags(self.dataset_path, DATASET, SPLIT), [self.legacy_bag])

    def test_finds_no_rosbags_for_an_unknown_version(self):
        """A version without rosbags reports none, so that they are generated in its subfolder."""
        self.assertEqual(find_existing_rosbags(self.dataset_path, DATASET, SPLIT, "2.0.0"), [])

    def test_finds_no_rosbags_of_another_split(self):
        """Rosbags remain scoped to their split within a version."""
        self.assertEqual(find_existing_rosbags(self.dataset_path, DATASET, "mini_train", VERSION), [])


if __name__ == "__main__":
    unittest.main()
