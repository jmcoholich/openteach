import json
import time
from copy import deepcopy as copy
from pathlib import Path

import h5py
import numpy as np
import zmq
from deoxys.franka_interface import FrankaInterface
from deoxys.utils import transform_utils
from deoxys.utils.config_utils import YamlConfig, verify_controller_config
from scipy.spatial.transform import Rotation, Slerp

from openteach.components.operators.operator_base import Operator
from openteach.constants import *
from openteach.ik import geofik
from openteach.utils.files import *
from openteach.utils.network import ZMQKeypointSubscriber
from openteach.utils.timer import FrequencyTimer
from openteach.utils.vectorops import *

CONFIG_ROOT = os.path.join(os.path.expanduser("~"), "openteach/configs")

CONTROL_FREQ = 20
STATE_FREQ = 60

ROTATION_VELOCITY_LIMIT = 0.2
TRANSLATION_VELOCITY_LIMIT = 0.1

TELEOP_SCALE_PARAM = 1.0

FLIP_TELEOP = True  # teleop facing the robot

JUST_GO_STRAIGHT_UP = False # experimental option to slowly lift the arm straight up for unplugging
ABS_JOINT_LOWPASS_NEW_ACTION_WEIGHT = 0.9
ABS_JOINT_LOWPASS_JOINT_1_NEW_ACTION_WEIGHT = 0.4

CONTROL_MODE_OPTIONS = (
    "eef_delta_actions",
    "absolute_joint",
    "absolute_joint_direct",
    "absolute_eef_pose_to_delta",
    "absolute_eef_pose_direct",
)

def get_velocity_controller_config(config_root):
    controller_cfg = YamlConfig(
        os.path.join(config_root, "osc-pose-controller-velocity.yml")
    ).as_easydict()
    controller_cfg = verify_controller_config(controller_cfg)

    return controller_cfg

def get_position_controller_config(config_root):
    controller_cfg = YamlConfig(
        os.path.join(config_root, "osc-pose-controller-position.yml")
    ).as_easydict()
    controller_cfg = verify_controller_config(controller_cfg)

    return controller_cfg

def get_joint_impedance_controller(config_root):
    controller_cfg = YamlConfig(
        os.path.join(config_root, "joint-pos-controller-impedance.yml")
    ).as_easydict()
    controller_cfg = verify_controller_config(controller_cfg)

    return controller_cfg

np.set_printoptions(precision=5, suppress=True)
# Filter to smooth out the arm cartesian state
class Filter:
    def __init__(self, state, comp_ratio=0.6):
        self.pos_state = state[:3]
        self.ori_state = state[3:7]
        self.comp_ratio = comp_ratio

    def __call__(self, next_state):
        self.pos_state = self.pos_state[:3] * self.comp_ratio + next_state[:3] * (1 - self.comp_ratio)
        ori_interp = Slerp([0, 1], Rotation.from_quat(
            np.stack([self.ori_state, next_state[3:7]], axis=0)),)
        self.ori_state = ori_interp([1 - self.comp_ratio])[0].as_quat()
        return np.concatenate([self.pos_state, self.ori_state])

class FrankaArmOperator(Operator):
    _STATE_LOG_PROPERTIES = (
        "last_eef_pose",
        "last_eef_pose_d",
        "last_F_T_EE",
        "last_F_T_NE",
        "last_q",
        "last_tau_J",
        "last_dtau_J",
        "last_tau_J_d",
        "last_tau_ext_hat_filtered",
        "last_dq_d",
        "last_ddq_d",
        "last_q_d",
        "last_dq",
        "last_pose",
    )
    def __init__(
        self,
        host,
        transformed_keypoints_port,
        remote_message_port,
        gripper_message_port,
        use_filter=False,
        arm_resolution_port = None,
        teleoperation_reset_port = None,
        record=None,
        storage_location="extracted_data",
        control_mode=None,
        controller_cfg=None,
    ):
        if control_mode is None:
            raise RuntimeError(f"robot control mode needs to be specified. Valid options are: {CONTROL_MODE_OPTIONS}")
        if control_mode not in CONTROL_MODE_OPTIONS:
            raise ValueError(f"Invalid robot control mode: {control_mode}. Valid options are: {CONTROL_MODE_OPTIONS}")
        self.control_mode = control_mode
        self._last_abs_joint_action = None
        self.notify_component_start('franka arm operator')
        # Subscribers for the transformed hand keypoints
        self.record = record
        self.storage_location = storage_location

        # remote coords path oculus -> keypoint_transform -> franka
        if transformed_keypoints_port is None:
            self._transformed_hand_keypoint_subscriber = None
            self._transformed_arm_keypoint_subscriber = None
        else:
            self._transformed_hand_keypoint_subscriber = ZMQKeypointSubscriber(
                host=host,
                port=transformed_keypoints_port,
                topic='transformed_hand_coords'
            )
            # Subscribers for the transformed arm frame
            self._transformed_arm_keypoint_subscriber = ZMQKeypointSubscriber(
                host=host,
                port=transformed_keypoints_port,
                topic='transformed_hand_frame'
            )
        # Subscribers for the remote message
        if remote_message_port is None:
            self._remote_message_subscriber = None
        else:
            self._remote_message_subscriber = ZMQKeypointSubscriber(
                host=host,
                port=remote_message_port,
                topic='remote_msg'
            )
        # Subscribers for the gripper message
        if gripper_message_port is None:
            self._gripper_message_subscriber = None
        else:
            self._gripper_message_subscriber = ZMQKeypointSubscriber(
                host=host,
                port=gripper_message_port,
                topic='gripper_msg'
            )

        self.deoxys_obs_cmd_history = {}
        self.robot_interface = FrankaInterface(
                os.path.join(CONFIG_ROOT, 'deoxys.yml'), use_visualizer=False,
                control_freq=CONTROL_FREQ,
                state_freq=STATE_FREQ
            )
        if controller_cfg is not None:
            self.velocity_controller_cfg = None
            self.position_controller_cfg = None
            self.joint_impedance_controller_cfg = None
            controller_cfg = copy(controller_cfg)
            controller_cfg.pop("control_mode", None)
            self.controller_cfg = verify_controller_config(controller_cfg)
        else:
            self.velocity_controller_cfg = get_velocity_controller_config(
                config_root = CONFIG_ROOT
            )
            print("\nvelocity controller config", self.velocity_controller_cfg)

            self.position_controller_cfg = get_position_controller_config(
                config_root = CONFIG_ROOT
            )

            self.joint_impedance_controller_cfg = get_joint_impedance_controller(
                config_root = CONFIG_ROOT
            )
            if control_mode == "eef_delta_actions":  # formerly "playback" mode. Assumes delta eef actions are already provided.
                self.controller_cfg = self.velocity_controller_cfg
            elif control_mode == "absolute_joint" or control_mode == "absolute_joint_direct":
                self.controller_cfg = self.joint_impedance_controller_cfg
            elif control_mode == "absolute_eef_pose_to_delta":  # this is the old teleop pipeline
                self.controller_cfg = self.velocity_controller_cfg
            elif control_mode == "absolute_eef_pose_direct":  # this skips the outer loop and directly commands
                self.controller_cfg = self.position_controller_cfg
            else:
                raise ValueError("Not a valid control mode")

        # Initalizing the robot controller
        self.arm_teleop_state = ARM_TELEOP_STOP # We will start as the cont

        # Subscribers for the resolution scale and teleop state
        # TODO: See if we can remove this since we dont use them
        if arm_resolution_port is None:
            self._arm_resolution_subscriber = None
        else:
            self._arm_resolution_subscriber = ZMQKeypointSubscriber(
                host = host,
                port = arm_resolution_port,
                topic = 'button'
            )
        if teleoperation_reset_port is None:
            self._arm_teleop_state_subscriber = None
        else:
            self._arm_teleop_state_subscriber = ZMQKeypointSubscriber(
                host = host,
                port = teleoperation_reset_port,
                topic = 'pause'
            )
        # Robot Initial Frame
        self.robot_init_H = self.robot_interface.last_eef_pose
        self.hand_moving_H = None
        self.hand_init_t = None
        self.is_first_frame = True

        self.use_filter = use_filter
        if use_filter:
            robot_init_cart = self._homo2cart(self.robot_init_H)
            self.comp_filter = Filter(robot_init_cart, comp_ratio=0.8)

        self._timer = FrequencyTimer(VR_FREQ)

        self.gripper_state = None
        self.gripper_last_msg = False
        self.gripper_cmd = -1
        self.below_thresh = False

    @property
    def timer(self):
        return self._timer

    # @property
    # def robot(self):
    #     return self._robot

    @property
    def transformed_hand_keypoint_subscriber(self):
        return self._transformed_hand_keypoint_subscriber

    @property
    def transformed_arm_keypoint_subscriber(self):
        return self._transformed_arm_keypoint_subscriber

    @property
    def remote_message_subscriber(self):
        return self._remote_message_subscriber

    # Get the hand frame
    def _get_hand_frame(self):
        for i in range(10):
            data = self.transformed_arm_keypoint_subscriber.recv_keypoints(flags=zmq.NOBLOCK)
            if data is not None: break
        if data is None: return None
        return np.asanyarray(data).reshape(4, 3)

    def _get_remote_message(self):
        """Map from MetaQuest remote message to a homo mat for use in teleop."""
        for i in range(10):
            data = self.remote_message_subscriber.recv_keypoints()
            if data is not None:
                break
        if data is None:
            return None
        # pose is x, y, z, qx, qy, qz, qw
        # need to transform this to a (4,3) pose matrix
        remote_pose = np.array(data[0])
        # offset_R = np.array(data[1])  # this is the position points offset along the principle axes in the remote frame

        t = np.array(remote_pose[:3])
        R = transform_utils.quat2mat(remote_pose[3:])

        # remote_frame = np.vstack([t, R])
        homo_mat = np.zeros((4, 4))
        homo_mat[:3, :3] = R
        homo_mat[:3, 3] = t
        homo_mat[3, 3] = 1

        x_rot_90 = np.array([
            [1, 0, 0, 0],
            [0, 0, -1, 0],
            [0, 1, 0, 0],
            [0, 0, 0, 1],
            ])
        homo_mat = x_rot_90 @ homo_mat
        y_refl_mat = np.eye(3)
        y_refl_mat[1, 1] = -1
        z_refl_mat = np.eye(3)
        z_refl_mat[2, 2] = -1
        homo_mat = homo_mat.copy()
        # Reflect translation into the robot frame, but leave orientation in the
        # rotated frame. Reflecting the rotation block as well flips controller
        # roll in the relative-pose teleop path used by _controller_tracking().
        homo_mat[:3, :3] = z_refl_mat @ homo_mat[:3, :3] @ z_refl_mat
        homo_mat[:3, 3] = y_refl_mat @ homo_mat[:3, 3]

        return homo_mat

    # # Create a coordinate frame for the arm
    # def _get_dir_frame(self, base, offset):
    #     X = normalize_vector(offset[0] - base)
    #     Y = normalize_vector(offset[1] - base)
    #     Z = normalize_vector(base - offset[2])

    #     return [X, Y, Z]

    def _get_gripper_message(self):
        msg = self._gripper_message_subscriber.recv_keypoints()
        if not self.gripper_last_msg and msg:
            self.gripper_cmd *= -1
            self.gripper_last_msg = msg
        elif self.gripper_last_msg and not msg:
            self.gripper_last_msg = msg
        return self.gripper_cmd

    # Get the resolution scale mode (High or Low)
    def _get_resolution_scale_mode(self):
        data = self._arm_resolution_subscriber.recv_keypoints()
        res_scale = np.asanyarray(data).reshape(1)[0] # Make sure this data is one dimensional
        return res_scale

    # Get the teleop state (Pause or Continue)
    def _get_arm_teleop_state(self):
        reset_stat = self._arm_teleop_state_subscriber.recv_keypoints()
        reset_stat = np.asanyarray(reset_stat).reshape(1)[0] # Make sure this data is one dimensional
        return reset_stat

    # Converts a frame to a homogenous transformation matrix
    def _turn_frame_to_homo_mat(self, frame):
        t = frame[0]
        R = frame[1:]

        homo_mat = np.zeros((4, 4))
        homo_mat[:3, :3] = np.transpose(R)
        homo_mat[:3, 3] = t
        homo_mat[3, 3] = 1

        return homo_mat

    # Converts Homogenous Transformation Matrix to Cartesian Coords
    def _homo2cart(self, homo_mat):

        t = homo_mat[:3, 3]
        R = Rotation.from_matrix(
            homo_mat[:3, :3]).as_quat()

        cart = np.concatenate(
            [t, R], axis=0
        )

        return cart

    # Gets the Scaled Resolution pose
    def _get_scaled_cart_pose(self, moving_robot_homo_mat):
        # Get the cart pose without the scaling
        unscaled_cart_pose = self._homo2cart(moving_robot_homo_mat)

        # Get the current cart pose
        current_homo_mat = copy(self.robot_interface.last_eef_pose)
        current_cart_pose = self._homo2cart(current_homo_mat)

        translation_scale = 0.5
        rotation_scale = 0.5

        # Get the difference in translation between these two cart poses
        diff_in_translation = unscaled_cart_pose[:3] - current_cart_pose[:3]
        scaled_diff_in_translation = diff_in_translation * translation_scale
        # print('SCALED_DIFF_IN_TRANSLATION: {}'.format(scaled_diff_in_translation))

        target_rotation = Rotation.from_quat(unscaled_cart_pose[3:])
        current_rotation = Rotation.from_quat(current_cart_pose[3:])
        relative_rotation = target_rotation * current_rotation.inv()
        scaled_rotation = Rotation.from_rotvec(
            relative_rotation.as_rotvec() * rotation_scale
        ) * current_rotation

        scaled_cart_pose = np.zeros(7)
        scaled_cart_pose[3:] = scaled_rotation.as_quat()
        scaled_cart_pose[:3] = current_cart_pose[:3] + scaled_diff_in_translation # Get the scaled translation only

        return scaled_cart_pose

    # Reset the teleoperation and get the first frame
    def _reset_teleop(self):
        # Just updates the beginning position of the arm
        print('****** RESETTING TELEOP ****** ')
        self.robot_init_H = self.robot_interface.last_eef_pose
        first_hand_frame = self._get_hand_frame()
        while first_hand_frame is None:
            first_hand_frame = self._get_hand_frame()
        self.hand_init_H = self._turn_frame_to_homo_mat(first_hand_frame)
        self.hand_init_t = copy(self.hand_init_H[:3, 3])
        self.is_first_frame = False
        return first_hand_frame

        # Reset the teleoperation and get the first frame
    def _reset_controller_teleop(self):
        # Just updates the beginning position of the arm
        print('****** RESETTING TELEOP ****** ')
        self.robot_init_H = self.robot_interface.last_eef_pose
        self.hand_init_H = self._get_remote_message()
        while self.hand_init_H is None:
            self.hand_init_H = self._get_remote_message()
            time.sleep(0.1)
        self.is_first_frame = False

    # Apply the retargeted angles
    def _apply_retargeted_angles(self, log=False):

        # See if there is a reset in the teleop
        new_arm_teleop_state = self._get_arm_teleop_state()
        if self.is_first_frame or (self.arm_teleop_state == ARM_TELEOP_STOP and new_arm_teleop_state == ARM_TELEOP_CONT):
            moving_hand_frame = self._reset_teleop() # Should get the moving hand frame only once
        else:
            moving_hand_frame = self._get_hand_frame() # Should get the hand frame
        self.arm_teleop_state = new_arm_teleop_state

        # # Get the arm resolution
        # arm_teleoperation_scale_mode = self._get_resolution_scale_mode()
        # if arm_teleoperation_scale_mode == ARM_HIGH_RESOLUTION:
        #     self.resolution_scale = 1
        # elif arm_teleoperation_scale_mode == ARM_LOW_RESOLUTION:
        #     self.resolution_scale = 0.6

        if moving_hand_frame is None:
            return # It means we are not on the arm mode yet instead of blocking it is directly returning

        # Get the moving hand frame
        self.hand_moving_H = self._turn_frame_to_homo_mat(moving_hand_frame)

        # Transformation code
        H_HI_HH = copy(self.hand_init_H) # Homo matrix that takes P_HI  to P_HH - Point in Inital Hand Frame to Point in current hand Frame
        H_HT_HH = copy(self.hand_moving_H) # Homo matrix that takes P_HT to P_HH
        H_RI_RH = copy(self.robot_init_H) # Homo matrix that takes P_RI to P_RH

        # Rotation from allegro to franka
        H_A_R = np.array(
            [[1/np.sqrt(2), 1/np.sqrt(2), 0, 0],
             [-1/np.sqrt(2), 1/np.sqrt(2), 0, 0],
             [0, 0, 1, -0.06], # The height of the allegro mount is 6cm
             [0, 0, 0, 1]])

        H_HT_HI = np.linalg.pinv(H_HI_HH) @ H_HT_HH # Homo matrix that takes P_HT to P_HI
        H_RT_RH = H_RI_RH @ H_A_R @ H_HT_HI @ np.linalg.pinv(H_A_R) # Homo matrix that takes P_RT to P_RH
        # self.robot_moving_H = copy(H_RT_RH)

        # Use the resolution scale to get the final cart pose
        # final_pose = self._get_scaled_cart_pose(self.robot_moving_H)
        final_pose = self._homo2cart(copy(H_RT_RH))
        # Use a Filter
        # if self.use_filter:
        #     final_pose = self.comp_filter(final_pose)
        # Move the robot arm

        ## Add Gripper control. Gripper cmd should be in [-1, 1]
        gripper_cmd = self.get_gripper_state_from_hand_keypoints()
        # self.robot.set_gripper_state(gripper_cmd)
        # self.robot.arm_control(final_pose, gripper_cmd=gripper_cmd)
        self.arm_control(target_pose=final_pose, gripper_cmd=gripper_cmd)

    def _controller_tracking(self):
        new_arm_teleop_state = self._get_arm_teleop_state()
        if self.is_first_frame or (self.arm_teleop_state == ARM_TELEOP_STOP and new_arm_teleop_state == ARM_TELEOP_CONT):
            self._reset_controller_teleop()
            self.hand_moving_H = copy(self.hand_init_H)
        else:
            self.hand_moving_H = self._get_remote_message()
        self.arm_teleop_state = new_arm_teleop_state

        if self.hand_moving_H is None:
            return # It means we are not on the arm mode; return to avoid blocking

        flip_mat = np.eye(3)
        flip_mat_rot = np.eye(3)
        if FLIP_TELEOP:
            # make a z rotation matrix parameterized by a z rotation angle in degrees
            z_rot = 135 # degrees
            z_rot_rot = 180 # degrees
            flip_mat = np.array([
                [np.cos(np.radians(z_rot)), -np.sin(np.radians(z_rot)), 0],
                [np.sin(np.radians(z_rot)), np.cos(np.radians(z_rot)), 0],
                [0, 0, 1]
            ])
            flip_mat_rot = np.array([
                [np.cos(np.radians(z_rot_rot)), -np.sin(np.radians(z_rot_rot)), 0],
                [np.sin(np.radians(z_rot_rot)), np.cos(np.radians(z_rot_rot)), 0],
                [0, 0, 1]
            ])

        # Transformation code
        controller_origin_to_init = copy(self.hand_init_H)
        controller_origin_to_current = copy(self.hand_moving_H)
        robot_origin_to_init = copy(self.robot_init_H)

        robot_origin_to_current = np.eye(4)
        controller_relative_rotation = (
            controller_origin_to_current[:3, :3]
            @ np.linalg.pinv(controller_origin_to_init[:3, :3])
        )
        teleop_relative_rotation = flip_mat_rot @ controller_relative_rotation.T @ flip_mat_rot.T
        # teleop_relative_rotation = self._scale_down_rot(teleop_relative_rotation, [0.5, 0.5, 1.0])
        # The current controller frame mapping gets each physical motion onto the
        # correct robot axis, but with the opposite sign. Invert only the relative
        # rotation here so we keep the axis correspondence. Then rotate that
        # relative motion into the teleop viewpoint selected by flip_mat.
        robot_origin_to_current[:3, :3] = (
            robot_origin_to_init[:3, :3] @ teleop_relative_rotation
        )
        teleop_relative_translation = flip_mat @ (controller_origin_to_current[:3, 3] - controller_origin_to_init[:3, 3])
        # teleop_relative_translation = teleop_relative_translation * np.array([0.5, 0.5, 0.5])
        robot_origin_to_current[:3, 3] = robot_origin_to_init[:3, 3] + teleop_relative_translation
        if JUST_GO_STRAIGHT_UP:
            robot_origin_to_current[:3, :3] = (
                robot_origin_to_init[:3, :3]
            )
            self.robot_init_H[:3, 3] += np.array([0, 0, 0.005])
            robot_origin_to_current[:3, 3] = robot_origin_to_init[:3, 3]

        # Use the resolution scale to get the final cart pose
        final_pose = copy(self._get_scaled_cart_pose(robot_origin_to_current))
        # final_pose = self._homo2cart(copy(robot_origin_to_current))  # use this for unscaled actions

        # Add Gripper control. Gripper cmd should be in [-1, 1]
        gripper_cmd = self._get_gripper_message()
        self.arm_control(
            control_mode=self.control_mode,
            target_pose=final_pose,
            gripper_cmd=gripper_cmd,
            )


    @staticmethod
    def _scale_down_homo_mat(mat, scale_params):
        scaled_mat = np.array(mat, copy=True)
        scaled_mat[:3, 3] *= scale_params[:3]

        rotvec = Rotation.from_matrix(mat[:3, :3]).as_rotvec()
        scaled_mat[:3, :3] = Rotation.from_rotvec(rotvec * scale_params[3:]).as_matrix()

        return scaled_mat


    @staticmethod
    def _scale_down_rot(mat, scale_params):
        rotvec = Rotation.from_matrix(mat).as_rotvec()
        scaled_mat = Rotation.from_rotvec(rotvec * scale_params).as_matrix()
        return scaled_mat

    def arm_control(self, **kwargs):
        if self.robot_interface.state_buffer_size == 0:
            raise RuntimeError("No reading from franka interface. Is FCI enabled?")

        if self.control_mode == "eef_delta_actions":  # formerly "playback" mode. Assumes delta eef actions are already provided.
            if "action" not in kwargs:
                raise RuntimeError("no action passed to arm_control() for control mode eef_delta_actions")
            action = kwargs["action"]
            if isinstance(action, list):
                action = np.array(action)
        elif self.control_mode == "absolute_joint":
            action = self.get_abs_joint_angle_actions(kwargs["target_pose"])
            action = self._filter_abs_joint_action(action)
        elif self.control_mode == "absolute_eef_pose_to_delta":  # this is the old teleop pipeline
            action = self.get_abs_eef_pose_actions(kwargs["target_pose"])
        elif self.control_mode == "absolute_eef_pose_direct":  # this skips the outer loop and directly commands
            target_pos, target_quat = kwargs["target_pose"][:3], kwargs["target_pose"][3:]
            action = target_pos.squeeze().tolist() + transform_utils.quat2axisangle(target_quat).tolist()
        elif self.control_mode == "absolute_joint_direct":
            action = kwargs["action"]
        else:
            raise ValueError("Not a valid control mode")

        if action is None or self.controller_cfg is None:
            return

        self.update_logs(kwargs, action)

        self.robot_interface.control(
            controller_type=self.controller_cfg.controller_type,
            action=action,
            controller_cfg=self.controller_cfg,
        )

        if "gripper_cmd" in kwargs:
            self.robot_interface.gripper_control(kwargs["gripper_cmd"])

    def _filter_abs_joint_action(self, action):
        if action is None:
            return None
        action = np.asarray(action, dtype=np.float64)
        if (
            self._last_abs_joint_action is None
            or self._last_abs_joint_action.shape != action.shape
        ):
            self._last_abs_joint_action = action.copy()
            return action

        new_action_weights = np.full_like(action, ABS_JOINT_LOWPASS_NEW_ACTION_WEIGHT)
        if new_action_weights.size > 1:
            new_action_weights[1] = ABS_JOINT_LOWPASS_JOINT_1_NEW_ACTION_WEIGHT

        filtered_action = (
            new_action_weights * action
            + (1 - new_action_weights) * self._last_abs_joint_action
        )
        self._last_abs_joint_action = filtered_action.copy()
        return filtered_action

    def update_logs(self, kwargs, action):
        history = self.deoxys_obs_cmd_history
        interface = self.robot_interface
        index = len(history["index"]) if "index" in history else 0
        state_values = {
            property_name: getattr(interface, property_name)
            for property_name in self._STATE_LOG_PROPERTIES
        }
        current_quat, current_pos = interface.last_eef_quat_and_pos
        compound_values = {
            "last_eef_rot_and_pos": interface.last_eef_rot_and_pos,
            "last_eef_quat_and_pos": (current_quat, current_pos),
        }
        state_values.update({
            property_name: np.concatenate([
                np.asarray(value).reshape(-1)
                for value in property_value
            ])
            for property_name, property_value in compound_values.items()
        })

        log_values = {
            "cartesian_pose_cmd": kwargs.get("target_pose", None),
            "arm_action": action,
            "gripper_action": kwargs.get("gripper_cmd", None),
            "gripper_state": interface.last_gripper_q,
            "eef_quat": current_quat,
            "eef_pos": current_pos,
            "eef_pose": state_values["last_eef_pose"],
            "joint_pos": state_values["last_q"],
            "timestamp": time.time(),
            "index": index,
        }
        log_values.update(state_values)

        for key, value in log_values.items():
            history.setdefault(key, []).append(value)

    def get_abs_joint_angle_actions(self, target_pose):
        """Return a controller config and joint angle actions."""
        target_pose = np.array(target_pose, dtype=np.float32)
        target_pos, target_quat = target_pose[:3], target_pose[3:]
        target_mat = transform_utils.pose2mat(pose=(target_pos, target_quat))
        try:
            geofik_q = geofik.solve_geofik_swivel(self.robot_interface.last_q, target_mat)
            action = geofik_q
            return action
        except Exception as e:
            print("geofik_swivel_error", e)
            return None

    def get_abs_eef_pose_actions(self, target_pose):
        current_quat, current_pos = self.robot_interface.last_eef_quat_and_pos
        current_mat = transform_utils.pose2mat(pose=(current_pos.flatten(), current_quat.flatten()))

        target_pose = np.array(target_pose, dtype=np.float32)
        target_pos, target_quat = target_pose[:3], target_pose[3:]
        target_mat = transform_utils.pose2mat(pose=(target_pos, target_quat))

        pose_error = transform_utils.get_pose_error(target_pose=target_mat, current_pose=current_mat)

        if np.dot(target_quat, current_quat) < 0.0:
            current_quat = -current_quat
        quat_diff = transform_utils.quat_distance(target_quat, current_quat)
        axis_angle_diff = transform_utils.quat2axisangle(quat_diff)

        action_pos = pose_error[:3]
        action_axis_angle = axis_angle_diff.flatten()

        action_pos, _ = transform_utils.clip_translation(action_pos, TRANSLATION_VELOCITY_LIMIT)
        action_axis_angle, _ = transform_utils.clip_translation(action_axis_angle, ROTATION_VELOCITY_LIMIT)
        action = action_pos.tolist() + action_axis_angle.tolist()
        return action

    def stream(self):
        self.notify_component_start('franka control')
        print("Start controlling the robot hand using the Oculus Headset.\n")

        # Assume that the initial position is considered initial after 3 seconds of the start
        while True:
            try:
                # print(self.robot_interface.last_eef_pose)
                if self.robot_interface.last_eef_pose is not None:
                    self.timer.start_loop()

                    # Retargeting function
                    # self._apply_retargeted_angles(log=False)
                    self._controller_tracking()

                    self.timer.end_loop()
                # else:
                #     print('No robot state.. try activating deadman switch')
            except KeyboardInterrupt:
                print("KeyboardInterrupt detected. Stopping the teleoperator and saving the recorded data...")
                if self.record is not None and self.storage_location is not None:
                    self.save_obs_cmd_history()
                break
            except Exception as e:
                print(e)

        self.transformed_arm_keypoint_subscriber.stop()
        print('Stopping the teleoperator!')

    def save_obs_cmd_history(self):
        path = os.path.join(os.getcwd(), self.storage_location, f'deoxys_obs_cmd_history_{self.record}.h5')
        print('Saving the deoxys_obs_cmd_history to {}'.format(path))
        tmp_path = f"{path}.tmp"
        with h5py.File(tmp_path, 'w') as f:
            for key, value in self.deoxys_obs_cmd_history.items():
                dataset = self._history_values_to_dataset(key, value)
                if dataset.dtype.kind in ("O", "U"):
                    f.create_dataset(key, data=dataset, dtype=h5py.string_dtype(encoding="utf-8"))
                else:
                    f.create_dataset(key, data=dataset)
            controller_cfg = copy(self.controller_cfg)
            controller_cfg["control_mode"] = self.control_mode
            f.attrs["controller_type"] = self.controller_cfg.controller_type
            f.attrs["controller_cfg_json"] = json.dumps(controller_cfg, separators=(",", ":"), sort_keys=True)
            f.attrs["CONTROL_FREQ"] = CONTROL_FREQ
            f.attrs["STATE_FREQ"] = STATE_FREQ
            f.attrs["VR_FREQ"] = VR_FREQ
            f.attrs["ROTATION_VELOCITY_LIMIT"] = ROTATION_VELOCITY_LIMIT
            f.attrs["TRANSLATION_VELOCITY_LIMIT"] = TRANSLATION_VELOCITY_LIMIT
        os.replace(tmp_path, path)
        print('\nSaved successfully!\n')

    @staticmethod
    def _json_safe_history_value(value):
        if value is None:
            return None
        if isinstance(value, np.ndarray):
            return value.tolist()
        if isinstance(value, np.generic):
            return value.item()
        if isinstance(value, (list, tuple)):
            return [FrankaArmOperator._json_safe_history_value(item) for item in value]
        if isinstance(value, dict):
            return {
                key: FrankaArmOperator._json_safe_history_value(item)
                for key, item in value.items()
            }
        return value

    def _history_values_to_dataset(self, key, values):
        values = list(values)
        try:
            dataset = np.asarray(values)
            if dataset.dtype.kind != "O":
                return dataset
        except ValueError:
            pass

        template = None
        for value in values:
            if value is None:
                continue
            candidate = np.asarray(value)
            if candidate.dtype.kind in ("b", "i", "u", "f") and candidate.dtype.kind != "O":
                template = candidate
                break

        if template is None:
            print(f"Warning: saving {key} as strings because it has no numeric samples.")
            return np.asarray([
                json.dumps(self._json_safe_history_value(value), default=str)
                for value in values
            ], dtype=object)

        shape = template.shape
        output = np.full((len(values),) + shape, np.nan, dtype=np.float64)
        for i, value in enumerate(values):
            if value is None:
                continue
            candidate = np.asarray(value)
            if candidate.shape != shape or candidate.dtype.kind not in ("b", "i", "u", "f"):
                print(f"Warning: saving {key} as strings because sample {i} has an incompatible shape or dtype.")
                return np.asarray([
                    json.dumps(self._json_safe_history_value(value), default=str)
                    for value in values
                ], dtype=object)
            output[i] = candidate

        return output


###############################################################################
# Add gripper control
###############################################################################

    # Function to get gripper state from hand keypoints
    def get_gripper_state_from_hand_keypoints(self):
        transformed_hand_coords= self._transformed_hand_keypoint_subscriber.recv_keypoints()
        distance = np.linalg.norm(transformed_hand_coords[OCULUS_JOINTS['ring'][-1]]- transformed_hand_coords[OCULUS_JOINTS['thumb'][-1]])
        thresh = 0.05
        if self.gripper_state is None:
            self.gripper_state = not (self.robot_interface.last_gripper_q > 0.07)
        if distance < thresh and not self.below_thresh:
            # print random 4 digit number to check if the function is being called
            # print("distance less than thresh", random.randint(1000,9999))
            self.gripper_state = not self.gripper_state
            self.below_thresh = True
        elif distance > thresh:
            self.below_thresh = False
        gripper_cmd = np.array(self.gripper_state * 2 - 1)
        # print("gripper_cmd", gripper_cmd, random.randint(1000,9999))
        return gripper_cmd
