"""CycloneDDS IDL for the SLAMWARE chassis LiDAR topic (not shipped by unitree_sdk2py).

Copied from unitree_robotskills unitree_gpt6/g1d_r1d_navigation/src/navi_agent/services/
slamware_idl.py (LaserScan_ verified on the real robot at rt/slamware_ros_sdk_server_node/scan,
360 beams at 1 deg). This module must NOT use ``from __future__ import annotations``:
cyclonedds resolves the dotted prefix of string annotations, so ``types.float32`` would resolve
to the stdlib ``types`` module.
"""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
import cyclonedds.idl.types as types
from unitree_sdk2py.idl.std_msgs.msg.dds_ import Header_


@dataclass
@annotate.final
@annotate.autoid("sequential")
class LaserScan_(idl.IdlStruct, typename="sensor_msgs.msg.dds_.LaserScan_"):
    header: Header_
    angle_min: types.float32
    angle_max: types.float32
    angle_increment: types.float32
    time_increment: types.float32
    scan_time: types.float32
    range_min: types.float32
    range_max: types.float32
    ranges: types.sequence[types.float32]
    intensities: types.sequence[types.float32]
