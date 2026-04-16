"""
This script is for replaying demonstrations from .h5 files.

In order to run this script, you need to have the following running on the NUC

cd deoxys_control/deoxys && ./auto_scripts/auto_arm.sh config/charmander.yml
cd deoxys_control/deoxys && ./auto_scripts/auto_gripper.sh config/charmander.yml
"""

import argparse
import json
import os
import time

import h5py
import numpy as np
import paramiko
import yaml
from deoxys.experimental.motion_utils import reset_joints_to
from easydict import EasyDict

from openteach.components.operators.franka import (
    CONFIG_ROOT,
    CONTROL_FREQ,
    CONTROL_MODE_OPTIONS,
    ROTATION_VELOCITY_LIMIT,
    STATE_FREQ,
    TRANSLATION_VELOCITY_LIMIT,
    FrankaArmOperator,
)
from openteach.constants import VR_FREQ

parser = argparse.ArgumentParser()
parser.add_argument("--reverse", action="store_true", help="Reverse the demonstration playback.")
parser.add_argument("--demo_num", type=str, help="The demo number to replay.")
parser.add_argument("--suffix", type=str, help="Additional suffix after \"playback\".")
parser.add_argument(
    "--gripper_shift",
    type=int,
    default=None,
    help="Shift reversed gripper commands forward by this many control steps. Only valid with --reverse.",
)


def load_controller_cfg(controller_cfg_json):
    if isinstance(controller_cfg_json, np.bytes_):
        controller_cfg_json = controller_cfg_json.tobytes()
    if isinstance(controller_cfg_json, bytes):
        controller_cfg_json = controller_cfg_json.decode("utf-8")
    return EasyDict(json.loads(controller_cfg_json))


def get_control_mode(controller_cfg):
    control_mode = controller_cfg.get("control_mode")
    if control_mode is not None:
        if control_mode not in CONTROL_MODE_OPTIONS:
            raise ValueError(f"Invalid recorded control mode: {control_mode}")
        assert control_mode == "absolute_joint"
        return "absolute_joint_direct"
        return control_mode


def get_arm_control_kwargs(control_mode, arm_action, target_pose, gripper_cmd):
    if control_mode == "eef_delta_actions":
        return {"action": arm_action, "gripper_cmd": gripper_cmd}
    return {"target_pose": target_pose, "gripper_cmd": gripper_cmd}

def check_nuc_hash_and_diff():
    """This is to ensure there are no changes on the NUC that would affect playback."""
    s = time.time()
    with open(os.path.join(CONFIG_ROOT, "deoxys.yml"), "r") as f:
        deoxys_cfg = EasyDict(yaml.safe_load(f))
    expected_hash = "ad7eaee1ce5b033602906e9891ef124eb2eb30f9"
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
    filename = f"/home/jeremiah/openteach/extracted_data/demonstration_{args.demo_num}/demo_{args.demo_num}.h5"

    print()
    print("=" * 50)
    print(f"Replaying demonstration from {filename}...")
    print("=" * 50)
    print()

    # load network config
    with open(os.path.join(CONFIG_ROOT, "network.yaml"), "r") as f:
        network_cfg = EasyDict(yaml.safe_load(f))

    check_nuc_hash_and_diff()

    with h5py.File(filename, "r") as h5f:
        arm_action = h5f["arm_action"][:]
        gripper_action = h5f["gripper_action"][:]
        joint_pos = h5f["joint_pos"][:]
        controller_cfg_json = h5f.attrs["controller_cfg_json"]
        controller_cfg = load_controller_cfg(controller_cfg_json)
        control_mode = get_control_mode(controller_cfg)
        cartesian_pose_cmd = h5f["cartesian_pose_cmd"][:]

        if h5f.attrs["ROTATION_VELOCITY_LIMIT"] > ROTATION_VELOCITY_LIMIT:
            raise ValueError("Playback rotation velocity limit does not match recorded rotation velocity limit.")
        if h5f.attrs["TRANSLATION_VELOCITY_LIMIT"] > TRANSLATION_VELOCITY_LIMIT:
            raise ValueError("Playback translation velocity limit does not match recorded translation velocity limit.")
        assert CONTROL_FREQ == h5f.attrs["VR_FREQ"]
        assert STATE_FREQ > CONTROL_FREQ

    demo_number = os.path.basename(filename).split(".")[0][5:]
    recording_name = demo_number + "_playback"
    if args.reverse:
        recording_name += "_reversed"
        if control_mode == "eef_delta_actions":
            if not controller_cfg["is_delta"]:
                raise NotImplementedError
            arm_action = -arm_action[::-1]
        else:
            arm_action = arm_action[::-1]
            cartesian_pose_cmd = cartesian_pose_cmd[::-1]
        gripper_action = gripper_action[::-1]
        joint_pos = joint_pos[::-1]
        if args.gripper_shift:
            gripper_action[:-args.gripper_shift] = gripper_action[args.gripper_shift:]
    if args.suffix:
        recording_name += f"_{args.suffix}"

    print(f"Playback control mode: {control_mode}")
    print(f"Playback controller type: {controller_cfg.controller_type}")
    print(f"Recording playback as {recording_name}...\n")
    operator = FrankaArmOperator(
        network_cfg["host_address"],
        None,
        None,
        None,
        use_filter=False,
        arm_resolution_port = None,
        teleoperation_reset_port = None,
        record=recording_name,
        control_mode=control_mode,
        controller_cfg=controller_cfg,
    )
    # move robot to start position
    try:
        reset_joints_to(operator.robot_interface, joint_pos[0])
        for i in range(len(arm_action)):
            # control_kwargs = get_arm_control_kwargs(
            #     control_mode,
            #     arm_action[i],
            #     cartesian_pose_cmd[i],
            #     gripper_action[i],
            # )
            operator.arm_control(action=arm_action[i], gripper_cmd=gripper_action[i])
    except KeyboardInterrupt:
        print("KeyboardInterrupt detected. Saving playback history so far...")
    finally:
        operator.save_obs_cmd_history()


if __name__ == "__main__":
    args = parser.parse_args()
    if args.gripper_shift is not None and not args.reverse:
        parser.error("--gripper_shift can only be used with --reverse.")
    if args.gripper_shift is not None and args.gripper_shift < 0:
        parser.error("--gripper_shift must be non-negative.")
    if args.gripper_shift is None:
        args.gripper_shift = 0
    replay_from_h5(args)
