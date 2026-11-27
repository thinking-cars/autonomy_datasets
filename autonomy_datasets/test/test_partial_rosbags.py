# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

"""Unit tests for the partial rosbags directories of the processes sharing a dataset directory.

Each process writing rosbags claims its own subdirectory of the shared partial rosbags directory,
so that processes writing other splits or replaying the rosbags keep the rosbags it is writing,
while the subdirectories of processes that were killed are removed.
"""

import os
import shutil
import signal
import subprocess
import sys
import tempfile
import unittest

from autonomy_datasets.datasets.rosbag.rosbag import PartialRosbagsDir

# Claims a partial rosbags directory, reports its path and waits until it is killed
_CLAIMING_PROCESS = """
import sys, time
from autonomy_datasets.datasets.rosbag.rosbag import PartialRosbagsDir
partial_dir = PartialRosbagsDir(sys.argv[1])
print(partial_dir.claim(), flush=True)
time.sleep(600)
"""


class TestPartialRosbagsDir(unittest.TestCase):
    """Processes keep each other's rosbags in progress and remove those of stopped processes."""

    def setUp(self):
        """Create the shared partial rosbags directory of a test."""
        self.root = os.path.join(tempfile.mkdtemp(prefix="partial_rosbags_"), "partial")
        self.addCleanup(shutil.rmtree, os.path.dirname(self.root))

    def _claim(self) -> PartialRosbagsDir:
        """Claim a partial rosbags directory with a rosbag in progress, released after the test."""
        partial_dir = PartialRosbagsDir(self.root)
        self.addCleanup(partial_dir.release)
        os.makedirs(os.path.join(partial_dir.claim(), "rosbag_in_progress"))
        return partial_dir

    def test_claims_own_subdirectory(self):
        """Each process writes to its own subdirectory, which is claimed once per pass."""
        first, second = self._claim(), self._claim()
        self.assertNotEqual(first.path, second.path)
        self.assertEqual(os.path.dirname(first.path), self.root)
        self.assertEqual(first.claim(), first.path)

    def test_keeps_claimed_subdirectories_of_other_processes(self):
        """Removing abandoned rosbags keeps the rosbags other running processes are writing."""
        writing = self._claim()
        PartialRosbagsDir(self.root).remove_abandoned()
        self.assertTrue(os.path.isdir(os.path.join(writing.path, "rosbag_in_progress")))

    def test_release_removes_own_subdirectory_only(self):
        """Releasing a subdirectory at the end of a pass keeps the subdirectories of the other processes."""
        released, other = self._claim(), self._claim()
        released_path = released.path
        released.release()
        self.assertFalse(os.path.exists(released_path))
        self.assertTrue(os.path.isdir(other.path))

    def test_removes_subdirectory_of_killed_process(self):
        """The subdirectory of a process that was killed is not locked anymore and is removed."""
        env = dict(os.environ, PYTHONPATH=os.pathsep.join(sys.path))
        process = subprocess.Popen(
            [sys.executable, "-c", _CLAIMING_PROCESS, self.root], stdout=subprocess.PIPE, text=True, env=env
        )
        killed_path = process.stdout.readline().strip()
        self.assertTrue(os.path.isdir(killed_path))
        PartialRosbagsDir(self.root).remove_abandoned()
        self.assertTrue(os.path.isdir(killed_path), msg="the subdirectory of a running process was removed")
        process.send_signal(signal.SIGKILL)
        process.wait()
        process.stdout.close()
        PartialRosbagsDir(self.root).remove_abandoned()
        self.assertFalse(os.path.exists(killed_path))

    def test_removes_rosbags_of_shared_layout(self):
        """Rosbags written directly to the shared directory by older versions of the node are removed."""
        legacy_bag = os.path.join(self.root, "nuscenes_mini_val_00001_token")
        os.makedirs(legacy_bag)
        writing = self._claim()
        PartialRosbagsDir(self.root).remove_abandoned()
        self.assertFalse(os.path.exists(legacy_bag))
        self.assertTrue(os.path.isdir(writing.path))


if __name__ == "__main__":
    unittest.main()
