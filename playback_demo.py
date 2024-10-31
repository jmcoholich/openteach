"""
This script is for replaying demonstrations from .pkl files and from the RLDS files
for verification and debugging purposes.

In order to run this script, you need to have the following running on the NUC

cd deoxys_control/deoxys && ./auto_scripts/auto_arm.sh config/charmander.yml
cd deoxys_control/deoxys && ./auto_scripts/auto_gripper.sh config/charmander.yml

"""

# deoxys_control
from deoxys.franka_interface import FrankaInterface
import deoxys.proto.franka_interface.franka_controller_pb2 as franka_controller_pb2
from deoxys.utils.config_utils import get_default_controller_config
from deoxys.utils.log_utils import get_deoxys_example_logger
from examples.osc_control import move_to_target_pose, deltas_move
from deoxys.experimental.motion_utils import reset_joints_to
from deoxys.utils.transform_utils import quat2axisangle, mat2euler, mat2quat, quat_distance, quat2mat, euler2mat

# General
import numpy as np
import pickle as pkl
import h5py
import tensorflow_datasets as tfds
import math
import argparse
import cv2
from time import sleep
import sys
import importlib
from scipy.spatial.transform import Rotation as R
import os
import time

# add openteachcontrollers to path
sys.path.append("/home/ripl/openteachcontrollers/src/franka-arm-controllers/franka_arm")
from controller import FrankaController

parser = argparse.ArgumentParser()
parser.add_argument("demo", type=str, help="The name of the demonstration to visualize")


def vectquat2axisangle(quat):
    """
    Converts quaternion to axis-angle format.
    Returns a unit vector direction scaled by its angle in radians.

    Args:
        quat (np.array): (N, 4) vec4 float angles in (x,y,z,w) format

    Returns:
        np.array: (N, 3) axis-angle exponential coordinates in (ax,ay,az) format
    """
    # clip quaternion
    quat[:, 3] = np.clip(quat[:, 3], -1.0, 1.0)


    den = np.sqrt(1.0 - quat[:, 3] * quat[:, 3])

    mask = np.isclose(den, 0.0)

    rpy = (quat[:, :3] * 2.0 * np.arccos(quat[:, 3])[:, None]) / den[:, None]
    rpy[mask] = np.zeros(3)

    return rpy

def dataloader(args):
    franka_controller = FrankaController(record=False, control_freq=60)
    reset_joint_positions = [
            0.09162008114028396,
            -0.19826458111314524,
            -0.01990020486871322,
            -2.4732269941140346,
            -0.01307073642274261,
            2.30396583422025,
            0.8480939705504309,
        ]

    # Initialize robot
    # logger = get_deoxys_example_logger()  # logger for debugging
    # robot_interface = FrankaInterface("/home/ripl/deoxys_control/deoxys/config/charmander.yml", use_visualizer=False)  # hardcoded path to config file, probably should change
    # controller_type = "OSC_POSE"  # controls end effector in 6 dimensions, need to use serpeate controller for gripper
    # controller_cfg = get_default_controller_config(controller_type=controller_type)


    # Load demonstration data
    sys.path.append("/home/ripl/rlds_dataset_builder")
    # ds = tfds.load("franka_pick_coke_single", split='train')
    ds = tfds.load("franka_pick_coke", split='train')

    # move robot to start position
    reset_joints_to(franka_controller.robot_interface, reset_joint_positions)

    for episode in ds.take(2):
        for i, st in enumerate(episode['steps']):
            deltas = st['action'].numpy()  # the action are deltas for (x, y, z, r, p, y, gripper)
            deltas[-1] = -1 if deltas[-1] == 0 else 1  # binarize gripper actions
            # curr_pos, curr_quat = franka_controller.get_cartesian_position()
            curr_pos, curr_quat = rlds_state2pos_quat(st['observation']['state'].numpy())
            eef_pos = curr_pos.reshape((3,)) + deltas[:3]
            curr_rot_mat = quat2mat(curr_quat)
            delta_rot_mat = euler2mat(deltas[3:6])
            target_rot_mat = delta_rot_mat @ curr_rot_mat
            eef_quat = mat2quat(target_rot_mat)

            cartesian_pose = np.concatenate((eef_pos, eef_quat))
            franka_controller.cartesian_control(cartesian_pose)
            franka_controller.set_gripper_position(deltas[-1])
            time.sleep(1.0/15)

def rlds_state2pos_quat(state):
    rpy = state[3:6]
    return state[:3], mat2quat(euler2mat(rpy))

def main(args):
    downsample_factor = 1
    franka_controller = FrankaController(record=False, control_freq=15 // downsample_factor)

    home = os.path.expanduser("~")
    # Load demonstration data
    filename = f"{home}/openteach/extracted_data/pick_coke/demonstration_{args.demo}/demo_{args.demo}.pkl"
    # arm_cmd_file = f"/home/ripl/openteach/extracted_data/pick_coke/demonstration_coke18/franka_arm_tcp_commands.h5"
    with open(filename, 'rb') as dbfile:
        db = pkl.load(dbfile)

    # binarize gripper actions (-1, 1)
    gripper_actions = np.where(db['gripper_cmd'] <= 0, 1, -1)
    assert len(gripper_actions) == len(db['eef_pose'])

    # move robot to start position
    reset_joints_to(franka_controller.robot_interface, db['joint_angles'][0])

    for i in range(0, len(gripper_actions) - downsample_factor, downsample_factor):
        delta_pos = db['eef_pos'][i + downsample_factor] - db['eef_pos'][i]
        if i == 0:
            curr_pos, curr_quat = franka_controller.get_cartesian_position()
            curr_pos = curr_pos.reshape((3,)) + delta_pos
        else:
            curr_pos += delta_pos

        # curr_pos = db['eef_pos'][i]
        cartesian_pose = np.concatenate((
            curr_pos,
            db['eef_quat'][i]
        ))

        franka_controller.cartesian_control(cartesian_pose)
        franka_controller.set_gripper_position(gripper_actions[i])

if __name__ == "__main__":
    args = parser.parse_args()
    main(args)
    # dataloader(args)
