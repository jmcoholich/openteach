import struct

import cv2
import numpy as np
import zmq

RGB_HEADER_FORMAT = '!dIIIB'
RGB_DTYPE_UINT8 = 1


class VideoStreamer(object):
    def __init__(self, host, cam_port):
        self._init_socket(host, cam_port)

    def _init_socket(self, host, port):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        # ZMQ_CONFLATE is not safe with multipart messages.
        self.socket.connect('tcp://{}:{}'.format(host, port))
        self.socket.setsockopt(zmq.SUBSCRIBE, b"rgb_image")

    def _get_image(self):
        parts = self.socket.recv_multipart()

        if not (len(parts) == 3 and parts[0] == b"rgb_image"):
            raise ValueError('Unexpected rgb_image message format.')

        _, height, width, channels, dtype_code = struct.unpack(RGB_HEADER_FORMAT, parts[1])
        if dtype_code != RGB_DTYPE_UINT8:
            raise ValueError('Unsupported rgb_image dtype code: {}'.format(dtype_code))

        frame = np.frombuffer(parts[2], dtype = np.uint8)
        expected_values = height * width * channels
        if frame.size != expected_values:
            raise ValueError(
                'Invalid rgb_image payload size: expected {}, got {}.'.format(expected_values, frame.size)
            )
        if channels == 1:
            image = frame.reshape((height, width))
        else:
            image = frame.reshape((height, width, channels))
        _, encoded_data = cv2.imencode('.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        return encoded_data.tobytes()

    def yield_frames(self):
        while True:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + self._get_image() + b'\r\n')  # concat frame one by one and show result


class MonitoringApplication(object):
    def __init__(self, configs):
        # Loading the network configurations
        self.host_address = configs.host_address
        self.keypoint_port = configs.keypoint_port
        self.port_offset = configs.cam_port_offset
        self.num_cams = len(configs.robot_cam_serial_numbers)

        # Initializing the streamers
        self._init_cam_streamers()
        self._init_graph_streamer()

        # Initializing frequency checkers
        self._init_frequency_checkers()

    def _init_graph_streamer(self):
        # TODO
        pass

    def _init_frequency_checkers(self):
        # TODO - Raw keypoint frequency
        # TODO - Transformed keypoint frequency
        pass

    def _init_cam_streamers(self):
        self.cam_streamers = []
        for idx in range(self.num_cams):
            self.cam_streamers.append(
                VideoStreamer(
                    host = self.host_address,
                    cam_port = self.port_offset + idx
                )
            )

    def get_cam_streamer(self, id):
        return self.cam_streamers[id - 1]
