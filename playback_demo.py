# deoxys_control
from deoxys.franka_interface import FrankaInterface
import deoxys.proto.franka_interface.franka_controller_pb2 as franka_controller_pb2
from deoxys.utils.config_utils import get_default_controller_config
from deoxys.utils.log_utils import get_deoxys_example_logger
from examples.osc_control import move_to_target_pose, deltas_move
from deoxys.experimental.motion_utils import reset_joints_to
from deoxys.utils.transform_utils import quat2axisangle, mat2euler, mat2quat, quat_distance
from deoxys.utils import transform_utils

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
    logger = get_deoxys_example_logger()  # logger for debugging
    robot_interface = FrankaInterface("/home/ripl/deoxys_control/deoxys/config/charmander.yml", use_visualizer=False)  # hardcoded path to config file, probably should change
    controller_type = "OSC_POSE"  # controls end effector in 6 dimensions, need to use serpeate controller for gripper
    controller_cfg = get_default_controller_config(controller_type=controller_type)


    # Load demonstration data
    sys.path.append("/home/ripl/rlds_dataset_builder")
    ds = tfds.load("franka_pick_coke_single", split='train')

    # move robot to start position
    reset_joints_to(robot_interface, reset_joint_positions)

    for episode in ds.take(1):
        for i, st in enumerate(episode['steps']):
            # first move to starting position
            if i == 0:
                last_eef = np.concatenate((robot_interface.last_eef_quat_and_pos[1].flatten(), quat2axisangle(robot_interface.last_eef_quat_and_pos[0])))
                eef = st['observation']['state'].numpy()
                deltas = eef - last_eef

                move_to_target_pose(
                        robot_interface,
                        controller_type,
                        controller_cfg,
                        target_delta_pose=deltas,
                        num_steps=40,
                        num_additional_steps=1,
                        interpolation_method="linear"
                        )
            else:
                deltas = st['action'].numpy()
                # deltas[:3] = [i/.05 for i in deltas[:3]]
                deltas[-1] = -1 if deltas[-1] == 0 else 1  # binarize gripper actions
                # print("deltas:", deltas)
                # robot_interface.control(
                #     controller_type=controller_type,
                #     action=deltas,
                #     controller_cfg=controller_cfg,
                # )

                move_to_target_pose(
                            robot_interface,
                            controller_type,
                            controller_cfg,
                            target_delta_pose=deltas[:6],
                            num_steps=40,
                            num_additional_steps=1,
                            interpolation_method="linear"
                            )
                robot_interface.gripper_control(deltas[6])
                # sleep(1)

def main(args):
    # Initialize robot
    logger = get_deoxys_example_logger()  # logger for debugging
    robot_interface = FrankaInterface("/home/ripl/deoxys_control/deoxys/config/charmander.yml", use_visualizer=False)  # hardcoded path to config file, probably should change
    controller_type = "OSC_POSE"  # controls end effector in 6 dimensions, need to use serpeate controller for gripper
    controller_cfg = get_default_controller_config(controller_type=controller_type)


    # Load demonstration data
    filename = f"/home/ripl/openteach/extracted_data/pick_coke/demonstration_{args.demo}/demo_{args.demo}.pkl"
    # arm_cmd_file = f"/home/ripl/openteach/extracted_data/pick_coke/demonstration_coke18/franka_arm_tcp_commands.h5"
    with open(filename, 'rb') as dbfile:
        db = pkl.load(dbfile)

    # with h5py.File(arm_cmd_file, "r") as f:
    #     tpc_cmd = f['arm_tcp_commands'][()]

    # binarize gripper actions (-1, 1)
    gripper_actions = np.where(db['gripper_cmd'] <= 0, 1, -1)

    # move robot to start position
    reset_joints_to(robot_interface, db['joint_angles'][0])

    for i, action in enumerate(gripper_actions):
        # skip starting position
        N = 1  # number of steps to skip
        if i == 0 or i+N+1 >= len(gripper_actions) or i%N != 0:
            continue


        # can replay demos well when we use the robots actual current postion to calulate deltas
        # current_pos, current_quat = (robot_interface.last_eef_quat_and_pos[1].flatten(), robot_interface.last_eef_quat_and_pos[0])

        # when we use the robots recorded position to calculate deltas, the accured error is noticable
        current_pos, current_quat = (db['eef_pose'][i][:3, 3], mat2quat(db['eef_pose'][i][:3, :3]))  # this is more accurate to how we are training

        # get the goal position
        target_pos = db['eef_pose'][i+N][:3, 3]

        # calculate the deltas needed to go from current to target position
        delta_pos = target_pos - current_pos
        target_quat = mat2quat(db['eef_pose'][i+N][:3, :3])
        if np.dot(target_quat, current_quat) < 0.0:
            current_quat = -current_quat
        delta_rpy = quat2axisangle(target_quat) - quat2axisangle(current_quat)

        deltas = np.concatenate((delta_pos, delta_rpy))

        # deltas = tpc_cmd[i]  # use commanded deltas instead of calculated deltas

        print(f"Calculated: {deltas}\n")
        move_to_target_pose(
                    robot_interface,
                    controller_type,
                    controller_cfg,
                    target_delta_pose=deltas,
                    num_steps=10,
                    num_additional_steps=1,
                    interpolation_method="linear"
                    )

        # need to be careful how we actually calculate the deltas here when diretly publishing to the robot
        # deltas[:3] = [i/.05 for i in deltas[:3]]  # counter action scale so the deltas that are pblished to robot are the same as calculated here
        # robot_interface.control(
        #     controller_type=controller_type,
        #     action=deltas,
        #     controller_cfg=controller_cfg,
        # )
        robot_interface.gripper_control(gripper_actions[i])


if __name__ == "__main__":
    main(parser.parse_args())
    # dataloader(parser.parse_args())