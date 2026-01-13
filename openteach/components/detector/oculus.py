from openteach.components import Component
from openteach.constants import ARM_TELEOP_CONT, ARM_TELEOP_STOP, VR_FREQ
from openteach.utils.network import ZMQKeypointPublisher, create_pull_socket
from openteach.utils.timer import FrequencyTimer


class OculusVRHandDetector(Component):
    def __init__(self, host, oculus_port, keypoint_pub_port, teleop_reset_port, teleop_reset_publish_port,
                 remote_port, remote_publish_port, gripper_message_port):
        self.notify_component_start('vr detector')
        # Initializing the network socket for getting the raw right hand keypoints
        # self.raw_keypoint_socket = create_pull_socket(host, oculus_port)
        # self.button_keypoint_socket = create_pull_socket(host, button_port)
        self.teleop_reset_socket = create_pull_socket(host, teleop_reset_port)

        # initializing the network socket for getting controller inputs
        self.remote_socket = create_pull_socket(host, remote_port)

        # ZMQ Socket for publishing controller inputs
        self.remote_pose_publisher = ZMQKeypointPublisher(
            host = host,
            port = remote_publish_port
        )

        # ZMQ Socket for Gripper
        self.gripper_publisher = ZMQKeypointPublisher(
            host = host,
            port = gripper_message_port
        )

        # Socket For Teleop Reset
        self.pause_info_publisher = ZMQKeypointPublisher(
            host =host,
            port =teleop_reset_publish_port
        )
        self.timer = FrequencyTimer(VR_FREQ)

    # Function to process the data token received from the VR
    def _process_data_token(self, data_token):
        return data_token.decode().strip()

    # Function to Extract the Keypoints from the String Token sent by the VR
    def _extract_data_from_token(self, token):
        data = self._process_data_token(token)
        information = dict()
        keypoint_vals = [0] if data.startswith('absolute') else [1]
        # Data is in the format <hand>:x,y,z|x,y,z|x,y,z
        vector_strings = data.split(':')[1].strip().split('|')
        for vector_str in vector_strings:
            vector_vals = vector_str.split(',')
            for float_str in vector_vals[:3]:
                keypoint_vals.append(float(float_str))

        information['keypoints'] = keypoint_vals
        return information

    def _extract_remote_data(self, message):
        data = self._process_data_token(message)
        typemarker, pos, quat, gripper, offset_forward, offset_right, offset_up = data.split('|')
        remote_pose = []
        for val in pos.split(','):
            remote_pose.append(float(val))
        for val in quat.split(','):
            remote_pose.append(float(val))

        offset_forward = [float(val) for val in offset_forward.split(',')]
        offset_right = [float(val) for val in offset_right.split(',')]
        offset_up = [float(val) for val in offset_up.split(',')]

        offset_R = [offset_forward, offset_right, offset_up]

        return remote_pose, gripper, offset_R

    # Function to Publish the transformed Keypoints
    def _publish_data(self, keypoint_dict):
        self.hand_keypoint_publisher.pub_keypoints(
            keypoint_array = keypoint_dict['keypoints'],
            topic_name = 'right'
        )

    # Function to Publish the Resolution Button Feedback
    def _publish_button_data(self,button_feedback):
        self.button_socket_publisher.pub_keypoints(
            keypoint_array = button_feedback,
            topic_name = 'button'
        )

    # Function to Publish the Teleop Reset Status
    def _publish_pause_data(self,pause_status):
        self.pause_info_publisher.pub_keypoints(
            keypoint_array = pause_status,
            topic_name = 'pause'
        )

    # Function to Publish the Remote Pose
    def _publish_remote_message(self, remote_pose, offset_R):
        self.remote_pose_publisher.pub_keypoints(
            keypoint_array = [remote_pose, offset_R],
            topic_name = 'remote_msg'
        )

    # Function to Publish the Remote Pose
    def _publish_gripper_message(self, gripper):
        msg = gripper == 'True'
        self.gripper_publisher.pub_keypoints(
            keypoint_array = msg,
            topic_name = 'gripper_msg'
        )

    # Function to Stream the Keypoints
    def stream(self):
        while True:
            try:
                self.timer.start_loop()

                # Getting remote message
                # TypeMarker|x,y,z|q1,q2,q3,q4|gripper|offset_forward,offset_right,offset_up
                remote_message = self.remote_socket.recv()
                remote_pose, gripper, offset_R = self._extract_remote_data(remote_message)

                # Getting the Teleop Reset Status
                pause_status = self.teleop_reset_socket.recv()
                if pause_status==b'Low':
                    pause_status = ARM_TELEOP_STOP
                else:
                    pause_status = ARM_TELEOP_CONT

                # Publish Remote Pose with offsets and Gripper
                self._publish_remote_message(remote_pose, offset_R)
                self._publish_gripper_message(gripper)

                # Publish Pause Data
                self._publish_pause_data(pause_status)
                self.timer.end_loop()
            except Exception as e:
                print(e)
                break

        self.remote_socket.close()
        self.teleop_reset_socket.close()
        self.pause_info_publisher.stop()
        self.remote_pose_publisher.stop()
        self.gripper_publisher.stop()

        print('Stopping the oculus keypoint extraction process.')