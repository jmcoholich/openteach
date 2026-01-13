import numpy as np
import zmq
import time


from copy import deepcopy as copy
from openteach.constants import *
from openteach.utils.timer import FrequencyTimer
from openteach.utils.network import ZMQKeypointSubscriber
from openteach.utils.vectorops import *
from openteach.utils.files import *
# from openteach.robot.franka import FrankaArm
from scipy.spatial.transform import Rotation, Slerp
from .operator import Operator

from deoxys.franka_interface import FrankaInterface
from deoxys.utils import transform_utils
from deoxys.utils.config_utils import YamlConfig
from deoxys.utils.config_utils import verify_controller_config


import pickle as pkl

CONTROLLER_TYPE = "OSC_POSE"
CONFIG_ROOT = '/home/ripl/openteach/configs'

CONTROL_FREQ = 60
STATE_FREQ = 200

ROTATION_VELOCITY_LIMIT = 0.5 # 1
TRANSLATION_VELOCITY_LIMIT = 0.1 # 2


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

np.set_printoptions(precision=2, suppress=True)
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

        # TODO: See if we can remove these since we dont use them or
        # find out if it is better to copy the old pub/sub layout
        # remote coords path oculus -> keypoint_transform -> franka
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
        self._remote_message_subscriber = ZMQKeypointSubscriber(
            host=host,
            port=remote_message_port,
            topic='remote_msg'
        )
        # Subscribers for the gripper message
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
        self._arm_resolution_subscriber = ZMQKeypointSubscriber(  # TODO: See if we can remove this since we dont use them
            host = host,
            port = arm_resolution_port,
            topic = 'button'
        )

        self._arm_teleop_state_subscriber = ZMQKeypointSubscriber(
            host = host,
            port = teleoperation_reset_port,
            topic = 'pause'
        )
        # Robot Initial Frame
        self.robot_init_H = self.robot_interface.last_eef_pose
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

        self.logs = []


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
        for i in range(10):
            data = self.remote_message_subscriber.recv_keypoints()
            if data is not None: break
        if data is None: return None
        # pose is x, y, z, qx, qy, qz, qw
        # need to transform this to a (4,3) pose matrix
        remote_pose = np.array(data[0])
        offset_R = np.array(data[1])

        t = np.array(remote_pose[:3])
        R = self._get_dir_frame(remote_pose[:3], offset_R)

        # R = Rotation.from_quat(remote_pose[3:]).as_matrix()
        # transform_R = self.headset_init_R @ R

        remote_frame = np.vstack([t, R])

        return remote_frame

    # Create a coordinate frame for the arm
    def _get_dir_frame(self, base, offset):
        X = normalize_vector(offset[0] - base)
        Y = normalize_vector(offset[1] - base)
        Z = normalize_vector(base - offset[2])

        return [X, Y, Z]

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
        # a = -45*np.pi/180
        # Y_45 = np.array(
        #     [
        #     [np.cos(a), 0, np.sin(a)],
        #     [0,1,0],
        #     [-np.sin(a), 0, np.cos(a)],
        #      ]
        # )
        # # t = (Y_45 @ t.reshape(3, 1)).reshape(3)

        # a = -90 * np.pi/180
        # X_90 = np.array([
        #     [1,0,0],
        #     [0, np.cos(a), -np.sin(a)],
        #     [0, np.sin(a), np.cos(a)],
        # ])
        # # t = (X_90 @ t.reshape(3, 1)).reshape(3)
        # a = 45 * np.pi/180
        # Z_45 = np.array([
        #     [np.cos(a), -np.sin(a), 0],
        #     [np.sin(a), np.cos(a), 0],
        #     [0,0,1],
        # ])
        # a = 180 * np.pi/180
        # X_180 = np.array([
        #     [1,0,0],
        #     [0, np.cos(a), -np.sin(a)],
        #     [0, np.sin(a), np.cos(a)],
        # ])
        # meta2robo = X_180 @ Z_45 @ X_90 @ Y_45
        # t = (meta2robo @ t.reshape(3, 1)).reshape(3)

        R = frame[1:]

        homo_mat = np.zeros((4, 4))
        homo_mat[:3, :3] = np.transpose(R)  # TODO: Why?? This seeems to be critical to how this works.
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
        first_hand_frame = self._get_remote_message()
        while first_hand_frame is None:
            first_hand_frame = self._get_remote_message()
        self.hand_init_H = self._turn_frame_to_homo_mat(first_hand_frame)
        self.hand_init_t = copy(self.hand_init_H[:3, 3])
        self.is_first_frame = False
        return first_hand_frame

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
        # See if there is a reset in the teleop
        new_arm_teleop_state = self._get_arm_teleop_state()
        if self.is_first_frame or (self.arm_teleop_state == ARM_TELEOP_STOP and new_arm_teleop_state == ARM_TELEOP_CONT):
            # initialize
            moving_hand_frame = self._reset_controller_teleop()
        else:
            moving_hand_frame = self._get_remote_message()
        self.arm_teleop_state = new_arm_teleop_state

        if moving_hand_frame is None:
            return # It means we are not on the arm mode yet instead of blocking it is directly returning

        # Get the moving hand frame
        self.hand_moving_H = self._turn_frame_to_homo_mat(moving_hand_frame)

        # Transformation code
        H_HI_HH = copy(self.hand_init_H) # Homo matrix that takes P_HI  to P_HH - Point in Inital Hand Frame to Point in current hand Frame
        H_HT_HH = copy(self.hand_moving_H) # Homo matrix that takes P_HT to P_HH
        H_RI_RH = copy(self.robot_init_H) # Homo matrix that takes P_RI to P_RH


        H_HT_HI = np.linalg.pinv(H_HI_HH) @ H_HT_HH # Homo matrix that takes P_HT to P_HI
        H_RT_RH = H_RI_RH @ H_HT_HI # Homo matrix that takes P_RT to P_RH

        # Use the resolution scale to get the final cart pose
        final_pose = copy(self._get_scaled_cart_pose(H_RT_RH))  # this scales actions by 0.5
        # final_pose = self._homo2cart(copy(H_RT_RH))  # use this for unscaled actions

        # Add Gripper control. Gripper cmd should be in [-1, 1]
        gripper_cmd = self._get_gripper_message()
        self.arm_control(final_pose, gripper_cmd)


    def arm_control(self, cartesian_pose, gripper_cmd):
        cartesian_pose = np.array(cartesian_pose, dtype=np.float32)
        target_pos, target_quat = cartesian_pose[:3], cartesian_pose[3:]
        target_mat = transform_utils.pose2mat(pose=(target_pos, target_quat))

        current_quat, current_pos = self.robot_interface.last_eef_quat_and_pos
        current_mat = transform_utils.pose2mat(pose=(current_pos.flatten(), current_quat.flatten()))

        pose_error = transform_utils.get_pose_error(target_pose=target_mat, current_pose=current_mat)

        if np.dot(target_quat, current_quat) < 0.0:
            current_quat = -current_quat
        quat_diff = transform_utils.quat_distance(target_quat, current_quat)
        axis_angle_diff = transform_utils.quat2axisangle(quat_diff)

        action_pos = pose_error[:3]
        action_axis_angle = axis_angle_diff.flatten()

        action_pos, _ = transform_utils.clip_translation(action_pos, TRANSLATION_VELOCITY_LIMIT)
        action_axis_angle = np.clip(action_axis_angle, -ROTATION_VELOCITY_LIMIT, ROTATION_VELOCITY_LIMIT)
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
                'controller_type': CONTROLLER_TYPE,
                'controller_cfg': self.velocity_controller_cfg,
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
            controller_type=CONTROLLER_TYPE,
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
                # # save logs
                # with open('logs.pkl', 'wb') as f:
                #     pkl.dump(self.logs, f)

                if self.record is not None and self.storage_location is not None:
                    path = os.path.join(os.getcwd(), self.storage_location, f'deoxys_obs_cmd_history_{self.record}.pkl')
                    print('Saving the deoxys_obs_cmd_history to {}'.format(path))
                    with open(path, 'wb') as f:
                        pkl.dump(self.deoxys_obs_cmd_history, f)
                        # pkl.dump(self.robot._controller.franka.deoxys_obs_cmd_history, f)
                break
            except Exception as e:
                print(e)


        self.transformed_arm_keypoint_subscriber.stop()
        print('Stopping the teleoperator!')


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
