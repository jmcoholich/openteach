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
from deoxys.utils.transform_utils import quat2axisangle, mat2euler, mat2quat, quat_distance, quat2mat, euler2mat, axisangle2quat, quat_multiply

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
from matplotlib import pyplot as plt

parser = argparse.ArgumentParser()
# parser.add_argument("demo", type=str, help="The name of the demonstration to visualize")

DEFAULT_CONTROLLER = EasyDict({
    'controller_type': 'OSC_POSE',
    'is_delta': False,
    'traj_interpolator_cfg': {
        'traj_interpolator_type': 'LINEAR_POSE',
        'time_fraction': 0.3
    },
    'Kp': {
        'translation': [250.0, 250.0, 250.0],
        'rotation': [250.0, 250.0, 250.0]
    },
    'action_scale': {
        'translation': 1.0,
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

CMD_ACTION_CONTROLLER = EasyDict({
    'controller_type': 'OSC_POSE',
    'is_delta': False,
    'traj_interpolator_cfg': {
        'traj_interpolator_type': 'LINEAR_POSE',
        'time_fraction': 0.3
    },
    'Kp': {
        'translation': [250.0, 250.0, 250.0],
        'rotation': [250.0, 250.0, 250.0]
    },
    'action_scale': {
        'translation': 1.0,
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

def replay_from_rlds(args):
    robot_interface = FrankaInterface(
        os.path.join('/home/ripl/openteach/configs', 'deoxys.yml'), use_visualizer=False,
        control_freq=5,
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
    ds = tfds.load("franka_pick_coke_single", split='train')

    # timer = FrequencyTimer(15)
    for episode in ds.take(1):
        for st in episode['steps']:
            # breakpoint()
            # timer.start_loop()
            deltas = st['action'].numpy()  # the action are deltas for (x, y, z, r, p, y, gripper)
            cv2.imshow("image", st['observation']['image'].numpy()[:, :, ::-1])  # convert to BGR for cv2
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # convert rpy to exponential axis-angle
            deltas[3:6] = quat2axisangle(mat2quat(euler2mat(deltas[3:6])))
            print(deltas)
            robot_interface.control(
                    controller_type='OSC_POSE',
                    action=deltas[:6],
                    controller_cfg=DEFAULT_CONTROLLER,
                )
            robot_interface.gripper_control(deltas[6])
            # timer.end_loop()



def replay_from_pkl(args):
    home = os.path.expanduser("~")
    # Load demonstration data
    # filename = f"/home/ripl/openteach/extracted_data/cups_demonstrations/pickle_files/demo_cups_0.pkl"
    # filename = f"/home/ripl/openteach/extracted_data/sim_demo_coke/demo_pick_up_coke_SIM_448.pkl"
    # filename = f"/data3/rlbench_demos/slowest_grip/converted/pick_up_coke/demo_pick_up_coke_SIM_003.pkl"
    filename = f"/data3/rlbench_demos/new_waypoints/converted/stack_blocks_simple/demo_stack_blocks_simple_SIM_000.pkl"


    # arm_cmd_file = f"/home/ripl/openteach/extracted_data/pick_coke/demonstration_coke18/franka_arm_tcp_commands.h5"
    with open(filename, 'rb') as dbfile:
        db = pkl.load(dbfile)
    # breakpoint()
    pos = db['eef_pos'].squeeze() + db['arm_action'][:, :3]
    quat_rot_actions = [axisangle2quat(x) for x in db['arm_action'][:, 3:]]
    rot = np.array([
        quat2axisangle(quat_multiply(i, j)) for i,j in \
            zip(quat_rot_actions, db['eef_quat'])
            ])
    arm_action = np.hstack((pos, rot))

    # breakpoint()

    # images = []
    # for i in db['rgb_frames'][:, 2]: # grab from the right camera
    #     images.append(cv2.cvtColor(i, cv2.COLOR_BGR2RGB))
    #     cv2.imshow("Camera", i)  # convert back to BGR for cv2
    #     if cv2.waitKey(30) & 0xFF == ord('q'):
    #         break
    # cv2.destroyAllWindows()
    # image_strip = np.concatenate(images[::4], axis=1)
    # plt.figure()
    # plt.imshow(image_strip)
    # plt.show()
    # breakpoint()
    robot_interface = FrankaInterface(
        os.path.join('/home/ripl/openteach/configs', 'deoxys.yml'), use_visualizer=False,
        control_freq=5,  # setting control frequency here so we don't have to handle it with a timer
        state_freq=200
    )
    # timer = FrequencyTimer(15)

    # move robot to start position
    reset_joints_to(robot_interface, db['joint_pos'][0])
    for i in range(0, len(arm_action)):
        # timer.start_loop()

        deltas = arm_action[i]

        robot_interface.control(
                controller_type=DEFAULT_CONTROLLER["controller_type"],
                action=deltas,
                controller_cfg=DEFAULT_CONTROLLER,
            )

        robot_interface.gripper_control(db["gripper_action"][i])
        # timer.end_loop()


if __name__ == "__main__":
    args = parser.parse_args()
    replay_from_pkl(args)
    # replay_from_rlds(args)
