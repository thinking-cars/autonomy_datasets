#!/usr/bin/env python3
# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

"""List the skipped test cases (with their skip reason) in the given xUnit result files.

``colcon test-result`` only reports how many tests were skipped, not which ones. Pass it the
result files, e.g. ``list_skipped_tests.py $(colcon test-result --all --result-files-only)``.
"""

import sys
import xml.etree.ElementTree as ET


def main(paths):
    """Print the name, result file and skip reason of every skipped test case (nothing if none)."""
    header_printed = False
    for path in paths:
        for testcase in ET.parse(path).iter("testcase"):
            skipped = testcase.find("skipped")
            if skipped is None:
                continue
            if not header_printed:
                print("\nSkipped tests:")
                header_printed = True
            name = ".".join(filter(None, (testcase.get("classname"), testcase.get("name"))))
            reason = skipped.get("message") or (skipped.text or "").strip()
            print(f"- {name} ({path})")
            print(f"  {reason}")


if __name__ == "__main__":
    main(sys.argv[1:])
