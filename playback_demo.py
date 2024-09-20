# deoxys_control
from deoxys.franka_interface import FrankaInterface
import deoxys.proto.franka_interface.franka_controller_pb2 as franka_controller_pb2
from deoxys.utils.config_utils import get_default_controller_config
from deoxys.utils.log_utils import get_deoxys_example_logger
from examples.osc_control import move_to_target_pose
from deoxys.experimental.motion_utils import reset_joints_to
from deoxys.utils.transform_utils import quat2axisangle, mat2euler

# General
import numpy as np
import pickle as pkl
import math
import argparse
import cv2
from time import sleep
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



def main(args):
    # Initialize robot
    logger = get_deoxys_example_logger()  # logger for debugging
    robot_interface = FrankaInterface("/home/ripl/deoxys_control/deoxys/config/charmander.yml", use_visualizer=False)  # hardcoded path to config file, probably should change
    controller_type = "OSC_POSE"  # controls end effector in 6 dimensions, need to use serpeate controller for gripper
    controller_cfg = get_default_controller_config(controller_type=controller_type)


    # Load demonstration data
    filename = f"/home/ripl/openteach/extracted_data/demonstration_{args.demo}/demo_{args.demo}.pkl"
    with open(filename, 'rb') as dbfile:
        db = pkl.load(dbfile)

    # Convert demonstration data to 6DOF actions
    pos = db['eef_pose'][:, :3, 3]  # (x, y, z)
    rpy = np.array([mat2euler(db['eef_pose'][i, :3, :3]) for i in range(db['eef_pose'].shape[0])])  # (roll, pitch, yaw)

    actions = np.concatenate((pos, rpy), axis=1)
    gripper_actions = np.where(db['gripper_cmd'] <= 0, 1, -1)
    # breakpoint()
    # move robot to start position
    reset_joints_to(robot_interface, db['joint_angles'][0])

    # mostly works, jumpy it has a tolerance threshold
    # for action in db['joint_angles']:
    #     reset_joints_to(robot_interface, action)

    # action scale probably plays a large role here and I don't know how to choose the correct value
    for i, action in enumerate(actions):
        # skip starting position
        if i == 0:
            continue

        # get last eef position and calulate deltas to goal position
        last_eef = np.concatenate((robot_interface.last_eef_quat_and_pos[1].flatten(), quat2axisangle(robot_interface.last_eef_quat_and_pos[0])))
        deltas = action - last_eef

        move_to_target_pose(
                    robot_interface,
                    controller_type,
                    controller_cfg,
                    target_delta_pose=deltas,
                    num_steps=1,
                    num_additional_steps=1,
                    interpolation_method="linear",
                    verbose=False
                    )
        robot_interface.gripper_control(gripper_actions[i])

if __name__ == "__main__":
    main(parser.parse_args())