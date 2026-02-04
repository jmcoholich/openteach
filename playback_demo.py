"""
This script is for replaying demonstrations from .h5 files and from the RLDS files
for verification and debugging purposes.

In order to run this script, you need to have the following running on the NUC

cd deoxys_control/deoxys && ./auto_scripts/auto_arm.sh config/charmander.yml
cd deoxys_control/deoxys && ./auto_scripts/auto_gripper.sh config/charmander.yml

"""

# deoxys_control
import argparse
import os
import sys

import cv2
import h5py

# General
import numpy as np
from deoxys.experimental.motion_utils import reset_joints_to
from deoxys.franka_interface import FrankaInterface
from deoxys.utils.transform_utils import (
    axisangle2quat,
    euler2mat,
    mat2quat,
    quat2axisangle,
    quat_multiply,
)
from easydict import EasyDict

from openteach.constants import VR_FREQ

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


def replay_from_h5(args):
    home = os.path.expanduser("~")
    # Load demonstration data
    # filename = f"{home}/openteach/extracted_data/demonstration_1/demo_1.h5"
    filename = "/home/jeremiah/openteach/extracted_data/demonstration_test_reversal/demo_test_reversal.h5"

    with h5py.File(filename, "r") as h5f:
        eef_pos = np.atleast_2d(np.squeeze(np.array(h5f["eef_pos"])))
        eef_quat = np.atleast_2d(np.squeeze(np.array(h5f["eef_quat"])))
        arm_action = np.atleast_2d(np.squeeze(np.array(h5f["arm_action"])))
        gripper_action = np.atleast_1d(np.squeeze(np.array(h5f["gripper_action"]))).reshape(-1)
        joint_pos = None
        if "joint_pos" in h5f:
            joint_pos = np.atleast_2d(np.squeeze(np.array(h5f["joint_pos"])))
    # breakpoint()
    pos = eef_pos + arm_action[:, :3]
    quat_rot_actions = [axisangle2quat(x) for x in arm_action[:, 3:]]
    rot = np.array([
        quat2axisangle(quat_multiply(i, j)) for i,j in \
            zip(quat_rot_actions, eef_quat, strict=True)
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
        control_freq=VR_FREQ,  # setting control frequency here so we don't have to handle it with a timer
        state_freq=200
    )
    # timer = FrequencyTimer(15)

    # move robot to start position
    if joint_pos is not None:
        reset_joints_to(robot_interface, joint_pos[0])
    else:
        print("No joint_pos dataset in h5; skipping joint reset.")
    for i in range(0, len(arm_action)):
        # timer.start_loop()

        deltas = arm_action[i]

        robot_interface.control(
                controller_type=DEFAULT_CONTROLLER["controller_type"],
                action=deltas,
                controller_cfg=DEFAULT_CONTROLLER,
            )

        robot_interface.gripper_control(gripper_action[i])
        # timer.end_loop()


if __name__ == "__main__":
    args = parser.parse_args()
    replay_from_h5(args)
    # replay_from_rlds(args)
