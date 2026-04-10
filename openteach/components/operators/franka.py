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

FLIP_TELEOP = True

JUST_GO_STRAIGHT_UP = False


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
    ):
        self.notify_component_start('franka arm operator')
        # Subscribers for the transformed hand keypoints
        self.record = record
        self.storage_location = storage_location
        debug_name = f"franka_debug_{record}.jsonl" if record is not None else "franka_debug.jsonl"
        self._debug_log_path = Path(os.getcwd()) / self.storage_location / debug_name

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
        self.velocity_controller_cfg = get_velocity_controller_config(
            config_root = CONFIG_ROOT
        )
        print("\nvelocity controller config", self.velocity_controller_cfg)

        self.position_controller_cfg = get_position_controller_config(
            config_root = CONFIG_ROOT
        )

        # Initalizing the robot controller
        # self._robot = FrankaArm()
        # self.resolution_scale = 1 # NOTE: Get this from a socket
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

    def _to_jsonable(self, value):
        if isinstance(value, np.ndarray):
            return value.tolist()
        if isinstance(value, (np.floating, np.integer)):
            return value.item()
        if isinstance(value, (list, tuple)):
            return [self._to_jsonable(item) for item in value]
        if isinstance(value, dict):
            return {key: self._to_jsonable(item) for key, item in value.items()}
        return value

    def _append_debug_log(self, event, **payload):
        self._debug_log_path.parent.mkdir(parents=True, exist_ok=True)
        record = {
            "timestamp": time.time(),
            "event": event,
            **{key: self._to_jsonable(value) for key, value in payload.items()},
        }
        with self._debug_log_path.open("a", encoding="utf-8") as f:
            f.write(json.dumps(record))
            f.write("\n")

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
        self._append_debug_log("raw_msg", data=data)
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
        self._append_debug_log("homo_mat", homo_mat=homo_mat)
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
        # homo_mat[:3, :3] = np.linalg.inv(homo_mat[:3, :3])
        self._append_debug_log("homo_mat_refl_y", homo_mat=homo_mat)

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

        # Get the difference in translation between these two cart poses
        diff_in_translation = unscaled_cart_pose[:3] - current_cart_pose[:3]
        scaled_diff_in_translation = diff_in_translation * 0.25  # translation_scale
        # print('SCALED_DIFF_IN_TRANSLATION: {}'.format(scaled_diff_in_translation))

        scaled_cart_pose = np.zeros(7)
        scaled_cart_pose[3:] = unscaled_cart_pose[3:] # Get the rotation directly
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
        self.arm_control(final_pose, gripper_cmd)

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

        # robot_init_to_current = np.linalg.pinv(controller_origin_to_init) @ controller_origin_to_current
        # robot_init_to_current = np.eye(4)
        # robot_init_to_current[:3, :3] = (np.linalg.pinv(controller_origin_to_init) @ controller_origin_to_current)[:3, :3]
        # robot_init_to_current[:3, 3] = controller_origin_to_current[:3, 3] - controller_origin_to_init[:3, 3]
        # robot_init_to_current = self._scale_down_homo_mat(robot_init_to_current, TELEOP_SCALE_PARAM)
        # robot_origin_to_current = robot_origin_to_init @ robot_init_to_current
        robot_origin_to_current = np.eye(4)
        controller_relative_rotation = (
            controller_origin_to_current[:3, :3]
            @ np.linalg.pinv(controller_origin_to_init[:3, :3])
        )
        teleop_relative_rotation = flip_mat_rot @ controller_relative_rotation.T @ flip_mat_rot.T
        # The current controller frame mapping gets each physical motion onto the
        # correct robot axis, but with the opposite sign. Invert only the relative
        # rotation here so we keep the axis correspondence. Then rotate that
        # relative motion into the teleop viewpoint selected by flip_mat.
        robot_origin_to_current[:3, :3] = (
            robot_origin_to_init[:3, :3] @ teleop_relative_rotation
        )
        robot_origin_to_current[:3, 3] = robot_origin_to_init[:3, 3] + flip_mat @ (controller_origin_to_current[:3, 3] - controller_origin_to_init[:3, 3])
        if JUST_GO_STRAIGHT_UP:
            robot_origin_to_current[:3, :3] = (
                robot_origin_to_init[:3, :3]
            )
            self.robot_init_H[:3, 3] += np.array([0, 0, 0.005])
            robot_origin_to_current[:3, 3] = robot_origin_to_init[:3, 3]

        self._append_debug_log(
            "controller_tracking_transform",
            # robot_init_to_current=robot_init_to_current,
            robot_origin_to_init=robot_origin_to_init,
            controller_origin_to_init=controller_origin_to_init,
            robot_origin_to_current=robot_origin_to_current,
        )
        # Use the resolution scale to get the final cart pose
        final_pose = copy(self._get_scaled_cart_pose(robot_origin_to_current))
        # final_pose = self._homo2cart(copy(robot_origin_to_current))  # use this for unscaled actions

        # Add Gripper control. Gripper cmd should be in [-1, 1]
        gripper_cmd = self._get_gripper_message()
        self.arm_control(final_pose, gripper_cmd)


    @staticmethod
    def _scale_down_homo_mat(mat, scale_param):
        scaled_mat = np.array(mat, copy=True)
        scaled_mat[:3, 3] *= scale_param

        rotvec = Rotation.from_matrix(mat[:3, :3]).as_rotvec()
        scaled_mat[:3, :3] = Rotation.from_rotvec(rotvec * scale_param).as_matrix()

        return scaled_mat


    def arm_control(self, cartesian_pose, gripper_cmd, playback_actions=None):
        current_quat, current_pos = self.robot_interface.last_eef_quat_and_pos
        if current_quat is None:
            raise RuntimeError("No reading from franka interface. Is FCI enabled?")
        current_mat = transform_utils.pose2mat(pose=(current_pos.flatten(), current_quat.flatten()))

        if playback_actions is not None:
            action, gripper_cmd = playback_actions
            # if isinstance(action, list):
            #     action = np.array(action)
            # action_pos, _ = transform_utils.clip_translation(action[:3], TRANSLATION_VELOCITY_LIMIT)
            # action_axis_angle, _ = transform_utils.clip_translation(action[3:], ROTATION_VELOCITY_LIMIT)
            # action = action_pos.tolist() + action_axis_angle.tolist()
            cartesian_pose = 0.0
        else:
            cartesian_pose = np.array(cartesian_pose, dtype=np.float32)
            target_pos, target_quat = cartesian_pose[:3], cartesian_pose[3:]
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

        if not self.deoxys_obs_cmd_history:
            self.deoxys_obs_cmd_history = {
                'cartesian_pose_cmd': [cartesian_pose],
                'arm_action': [action],
                'gripper_action': [gripper_cmd],
                'gripper_state': [self.robot_interface.last_gripper_q],
                'eef_quat': [current_quat],
                'eef_pos': [current_pos],
                'eef_pose': [current_mat],
                'joint_pos': [self.robot_interface.last_q],
                'timestamp': [time.time()],
                'index': [0],
            }
        else:
            self.deoxys_obs_cmd_history['cartesian_pose_cmd'].append(cartesian_pose)
            self.deoxys_obs_cmd_history['arm_action'].append(action)
            self.deoxys_obs_cmd_history['gripper_action'].append(gripper_cmd)
            self.deoxys_obs_cmd_history['gripper_state'].append(self.robot_interface.last_gripper_q)
            self.deoxys_obs_cmd_history['eef_quat'].append(current_quat)
            self.deoxys_obs_cmd_history['eef_pos'].append(current_pos)
            self.deoxys_obs_cmd_history['eef_pose'].append(current_mat)
            self.deoxys_obs_cmd_history['joint_pos'].append(self.robot_interface.last_q)
            self.deoxys_obs_cmd_history['timestamp'].append(time.time())
            self.deoxys_obs_cmd_history['index'].append(len(self.deoxys_obs_cmd_history['index']))

        self.robot_interface.control(
            controller_type=self.velocity_controller_cfg.controller_type,
            action=action,
            controller_cfg=self.velocity_controller_cfg,
        )

        if gripper_cmd is not None:
            self.robot_interface.gripper_control(gripper_cmd)

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
        with h5py.File(path, 'w') as f:
            for key, value in self.deoxys_obs_cmd_history.items():
                f.create_dataset(key, data=np.array(value))
            f.attrs["controller_type"] = self.velocity_controller_cfg.controller_type
            f.attrs["controller_cfg_json"] = json.dumps(self.velocity_controller_cfg, separators=(",", ":"), sort_keys=True)
            f.attrs["CONTROL_FREQ"] = CONTROL_FREQ
            f.attrs["STATE_FREQ"] = STATE_FREQ
            f.attrs["VR_FREQ"] = VR_FREQ
            f.attrs["ROTATION_VELOCITY_LIMIT"] = ROTATION_VELOCITY_LIMIT
            f.attrs["TRANSLATION_VELOCITY_LIMIT"] = TRANSLATION_VELOCITY_LIMIT
        print('\nSaved successfully!\n')


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
