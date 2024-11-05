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
from openteach.utils.timer import FrequencyTimer
from easydict import EasyDict

parser = argparse.ArgumentParser()
parser.add_argument("demo", type=str, help="The name of the demonstration to visualize")

DEFAULT_CONTROLLER = EasyDict({
    'controller_type': 'OSC_POSE',
    'is_delta': True,
    'traj_interpolator_cfg': {
        'traj_interpolator_type': 'LINEAR_POSE',
        'time_fraction': 0.3
    },
    'Kp': {
        'translation': [250.0, 250.0, 250.0],
        'rotation': [250.0, 250.0, 250.0]
    },
    'action_scale': {
        'translation': 0.5,
        'rotation': 1.0
    },
    'residual_mass_vec': [0.0, 0.0, 0.0, 0.0, 0.1, 0.5, 0.5],
    'state_estimator_cfg': {
        'is_estimation': False,
        'state_estimator_type': 'EXPONENTIAL_SMOOTHING',
        'alpha_q': 0.9,
        'alpha_dq': 0.9,
        'alpha_eef': 1.0,
        'alpha_eef_vel': 1.0
    }
})

# def vectquat2axisangle(quat):
#     """
#     Converts quaternion to axis-angle format.
#     Returns a unit vector direction scaled by its angle in radians.

#     Args:
#         quat (np.array): (N, 4) vec4 float angles in (x,y,z,w) format

#     Returns:
#         np.array: (N, 3) axis-angle exponential coordinates in (ax,ay,az) format
#     """
#     # clip quaternion
#     quat[:, 3] = np.clip(quat[:, 3], -1.0, 1.0)


#     den = np.sqrt(1.0 - quat[:, 3] * quat[:, 3])

#     mask = np.isclose(den, 0.0)

#     rpy = (quat[:, :3] * 2.0 * np.arccos(quat[:, 3])[:, None]) / den[:, None]
#     rpy[mask] = np.zeros(3)

#     return rpy

def replay_from_rlds(args):
    robot_interface = FrankaInterface(
        os.path.join('/home/ripl/openteach/configs', 'deoxys.yml'), use_visualizer=False,
        control_freq=60,
        state_freq=200
    )
    reset_joint_positions = [
            0.09162008114028396,
            -0.19826458111314524,
            -0.01990020486871322,
            -2.4732269941140346,
            -0.01307073642274261,
            2.30396583422025,
            0.8480939705504309,
        ]
    reset_joints_to(robot_interface, reset_joint_positions)

    # Load demonstration data
    sys.path.append("/home/ripl/rlds_dataset_builder")
    # ds = tfds.load("franka_pick_coke_single", split='train')
    ds = tfds.load("franka_pick_coke", split='train')

    timer = FrequencyTimer(15)
    for episode in ds.take(1):
        for st in episode['steps']:
            timer.start_loop()
            deltas = st['action'].numpy()  # the action are deltas for (x, y, z, r, p, y, gripper)
            # convert rpy to exponential axis-angle
            deltas[3:6] = quat2axisangle(mat2quat(euler2mat(deltas[3:6])))

            robot_interface.control(
                    controller_type='OSC_POSE',
                    action=deltas[:6],
                    controller_cfg=DEFAULT_CONTROLLER,
                )
            robot_interface.gripper_control(deltas[6])
            timer.end_loop()


def replay_from_pkl(args):
    home = os.path.expanduser("~")
    # Load demonstration data
    filename = f"{home}/openteach/extracted_data/demonstration_{args.demo}/demo_{args.demo}.pkl"
    # arm_cmd_file = f"/home/ripl/openteach/extracted_data/pick_coke/demonstration_coke18/franka_arm_tcp_commands.h5"
    with open(filename, 'rb') as dbfile:
        db = pkl.load(dbfile)
    robot_interface = FrankaInterface(
        os.path.join('/home/ripl/openteach/configs', 'deoxys.yml'), use_visualizer=False,
        control_freq=60,
        state_freq=200
    )
    timer = FrequencyTimer(15)

    # move robot to start position
    reset_joints_to(robot_interface, db['joint_pos'][0])
    for i in range(0, len(db["arm_action"])):
        timer.start_loop()
        robot_interface.control(
                controller_type=db["controller_type"],
                action=db["arm_action"][i],
                controller_cfg=db["controller_cfg"],
            )

        robot_interface.gripper_control(db["gripper_action"][i])
        timer.end_loop()


if __name__ == "__main__":
    args = parser.parse_args()
    # replay_from_pkl(args)
    replay_from_rlds(args)
