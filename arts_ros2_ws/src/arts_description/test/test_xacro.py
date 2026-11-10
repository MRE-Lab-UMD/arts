"""The cart xacro must flatten, and the three arm prefixes must be present.

Skipped when ur_description is not on AMENT_PREFIX_PATH, so a modular
arts_ros2_ws build that only sourced the FLIR overlay still configures.
"""
import os
import subprocess
import xml.etree.ElementTree as ET

import pytest
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory


PREFIXES = ("psm_left_", "psm_right_", "ecm_")


def test_xacro_processes_and_prefixes_arms():
    try:
        get_package_share_directory("ur_description")
    except PackageNotFoundError:
        pytest.skip("ur_description not on AMENT_PREFIX_PATH")

    share = get_package_share_directory("arts_description")
    xacro_file = os.path.join(share, "urdf", "arts.urdf.xacro")
    result = subprocess.run(
        ["xacro", xacro_file], check=True, capture_output=True, text=True
    )
    root = ET.fromstring(result.stdout)

    joints = {j.get("name") for j in root.iter("joint")}
    links = {link.get("name") for link in root.iter("link")}

    assert "world" in links
    assert "pedestal" in links
    for prefix in PREFIXES:
        name = prefix.rstrip("_")
        assert f"{name}_mount" in links, f"missing mount link for {name}"
        assert f"{prefix}shoulder_pan_joint" in joints
        assert f"{prefix}base_joint" in joints
        assert f"{prefix}base_link" in links

    # Camera arm is the CB3 (ur5); tool arms are UR5e. The type is not in the
    # flattened joint names, but the three prefixes must not collide.
    assert len([j for j in joints if j.endswith("shoulder_pan_joint")]) == 3
