"""
This script is for replaying demonstrations from .h5 files.

In order to run this script, you need to have the following running on the NUC

cd deoxys_control/deoxys && ./auto_scripts/auto_arm.sh config/charmander.yml
cd deoxys_control/deoxys && ./auto_scripts/auto_gripper.sh config/charmander.yml
"""

import argparse
import os
import time

import h5py
import paramiko
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
parser.add_argument("--reverse", action="store_true", help="Reverse the demonstration playback.")

def check_nuc_hash_and_diff():
    """This is to ensure there are no changes on the NUC that would affect playback."""
    s = time.time()
    with open(os.path.join(CONFIG_ROOT, "deoxys.yml"), "r") as f:
        deoxys_cfg = EasyDict(yaml.safe_load(f))
    expected_hash = "99e2526ed37fa9be5732750ebcc8f162892b49a6"
    repo_path = "/home/ripl/deoxys_control"  # path on the NUC
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        client.connect(
            hostname=deoxys_cfg.NUC.IP,
            username="ripl",
            timeout=5,
            banner_timeout=5,
            auth_timeout=5,
            allow_agent=True,
            look_for_keys=True,
        )
        status_cmd = f"cd {repo_path} && git status --porcelain"
        hash_cmd = f"cd {repo_path} && git rev-parse HEAD"
        _, status_stdout, status_stderr = client.exec_command(status_cmd, timeout=5)
        status_output = status_stdout.read().decode("utf-8", errors="replace").strip()
        status_error = status_stderr.read().decode("utf-8", errors="replace").strip()
        if status_error:
            raise RuntimeError(f"Failed to check git status on NUC: {status_error}")
        if status_output:
            raise RuntimeError("NUC deoxys_control has uncommitted changes.")
        _, hash_stdout, hash_stderr = client.exec_command(hash_cmd, timeout=5)
        current_hash = hash_stdout.read().decode("utf-8", errors="replace").strip()
        hash_error = hash_stderr.read().decode("utf-8", errors="replace").strip()
        if hash_error:
            raise RuntimeError(f"Failed to get git hash on NUC: {hash_error}")
        if current_hash != expected_hash:
            raise RuntimeError(
                f"NUC deoxys_control hash mismatch: expected {expected_hash}, got {current_hash}"
            )
    finally:
        client.close()
    print("NUC deoxys_control repository is clean and matches expected hash.")
    print(f"NUC check took {time.time() - s:.2f} seconds.")


def replay_from_h5(args):
    filename = "/home/jeremiah/openteach/extracted_data/demonstration_test3/demo_test3.h5"

    # load network config
    with open(os.path.join(CONFIG_ROOT, "network.yaml"), "r") as f:
        network_cfg = EasyDict(yaml.safe_load(f))

    check_nuc_hash_and_diff()

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

    demo_number = os.path.basename(filename).split(".")[0][5:]
    recording_name = demo_number + "_playback2"
    if args.reverse:
        if not controller_cfg["is_delta"]:
            raise NotImplementedError
        recording_name += "_reversed"
        arm_action = -arm_action[::-1]
        gripper_action = gripper_action[::-1]
        joint_pos = joint_pos[::-1]

    operator = FrankaArmOperator(
        network_cfg["host_address"],
        None,
        None,
        None,
        use_filter=False,
        arm_resolution_port = None,
        teleoperation_reset_port = None,
        record=recording_name,
    )

    operator.velocity_controller_cfg = controller_cfg
    # move robot to start position
    reset_joints_to(operator.robot_interface, joint_pos[0])
    for i in range(0, len(arm_action)):
        operator.timer.start_loop()
        playback_actions = (arm_action[i], gripper_action[i])
        operator.arm_control(None, None, playback_actions=playback_actions)
        operator.timer.end_loop()
    operator.save_obs_cmd_history()


if __name__ == "__main__":
    args = parser.parse_args()
    replay_from_h5(args)
