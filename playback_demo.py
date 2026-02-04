"""
This script is for replaying demonstrations from .h5 files.

In order to run this script, you need to have the following running on the NUC

cd deoxys_control/deoxys && ./auto_scripts/auto_arm.sh config/charmander.yml
cd deoxys_control/deoxys && ./auto_scripts/auto_gripper.sh config/charmander.yml

This is currently not tracked, but if the parameters in charmander.yml on the NUC
have changed, this could result in an innacurate playback.
"""

import argparse
import os

import h5py
import yaml
from deoxys.experimental.motion_utils import reset_joints_to
from easydict import EasyDict

from openteach.components.operators.franka import (
    CONFIG_ROOT,
    CONTROL_FREQ,
    ROTATION_VELOCITY_LIMIT,
    STATE_FREQ,
    TRANSLATION_VELOCITY_LIMIT,
    FrankaArmOperator,
)
from openteach.constants import VR_FREQ

parser = argparse.ArgumentParser()

def replay_from_h5(args):
    filename = "/home/jeremiah/openteach/extracted_data/demonstration_test/demo_test.h5"

    # load network config
    with open(os.path.join(CONFIG_ROOT, "network.yaml"), "r") as f:
        network_cfg = EasyDict(yaml.safe_load(f))

    with h5py.File(filename, "r") as h5f:
        arm_action = h5f["arm_action"][:]
        gripper_action = h5f["gripper_action"][:]
        joint_pos = h5f["joint_pos"][:]
        controller_type = h5f.attrs["controller_type"]
        controller_cfg_json = h5f.attrs["controller_cfg_json"]
        controller_cfg = EasyDict(yaml.safe_load(controller_cfg_json))

        # make sure all global settings` are the same
        if h5f.attrs["CONTROL_FREQ"] != CONTROL_FREQ:
            raise ValueError("Playback control frequency does not match recorded control frequency.")
        if h5f.attrs["STATE_FREQ"] != STATE_FREQ:
            raise ValueError("Playback state frequency does not match recorded state frequency.")
        if h5f.attrs["ROTATION_VELOCITY_LIMIT"] != ROTATION_VELOCITY_LIMIT:
            raise ValueError("Playback rotation velocity limit does not match recorded rotation velocity limit.")
        if h5f.attrs["TRANSLATION_VELOCITY_LIMIT"] != TRANSLATION_VELOCITY_LIMIT:
            raise ValueError("Playback translation velocity limit does not match recorded translation velocity limit.")
        if h5f.attrs["VR_FREQ"] != VR_FREQ:
            raise ValueError("Playback VR frequency does not match recorded VR frequency.")
        if not controller_type == "OSC_POSE":
            raise NotImplementedError("Only` OSC_POSE controller is supported in playback.")

    operator = FrankaArmOperator(
        network_cfg["host_address"],
        None,
        None,
        None,
        use_filter=False,
        arm_resolution_port = None,
        teleoperation_reset_port = None,
        record=None,
        storage_location="recorded_playbacks",
    )
    operator.velocity_controller_cfg = controller_cfg
    # move robot to start position
    reset_joints_to(operator.robot_interface, joint_pos[0])
    for i in range(0, len(arm_action)):
        operator.timer.start_loop()
        playback_actions = (arm_action[i], gripper_action[i])
        operator.arm_control(None, None, playback_actions=playback_actions)
        operator.timer.end_loop()


if __name__ == "__main__":
    args = parser.parse_args()
    replay_from_h5(args)
