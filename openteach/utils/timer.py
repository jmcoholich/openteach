import pickle
import struct
import time

import cv2
import numpy as np
import zmq

RGB_HEADER_FORMAT = '!dIIIB'
RGB_DTYPE_UINT8 = 1


class FrequencyTimer(object):
    def __init__(self, frequency_rate):
        self.time_available = 1e9 / frequency_rate

    def start_loop(self):
        self.start_time = time.time_ns()

    def end_loop(self):
        wait_time = self.time_available + self.start_time

        while time.time_ns() < wait_time:
            continue

class SocketChecker(object):
    def __init__(self, host, port, topic_name, print_data, data_type = None):
        self.data = None
        self.previous_data = None
        self.print_data = print_data
        self.data_type = data_type
        self.topic_name = topic_name
        self._init_connection(host, port)

    def _init_connection(self, host, port):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        if self.data_type == 'FloatArray':
            self.socket.setsockopt(zmq.CONFLATE, 1)
        self.socket.setsockopt(zmq.SUBSCRIBE, bytes(self.topic_name, 'utf-8'))
        self.socket.connect('tcp://{}:{}'.format(host, port))

    def _reinit_counter(self):
        self.counter = 0
        self.start_time = time.time()

    def _calculate_frequency(self):
        return self.counter / (time.time() - self.start_time)

    def _decode_array(self):
        processed_data = self.data.lstrip(
            bytes("{} ".format(self.topic_name), 'utf-8')
        )
        print(pickle.loads(processed_data))

    def _decode_rgb_image(self):
        if not (isinstance(self.data, list) and len(self.data) == 3 and self.data[0] == b"rgb_image"):
            raise ValueError('Unexpected rgb_image message format.')

        _, height, width, channels, dtype_code = struct.unpack(RGB_HEADER_FORMAT, self.data[1])
        if dtype_code != RGB_DTYPE_UINT8:
            raise ValueError('Unsupported rgb_image dtype code: {}'.format(dtype_code))

        frame = np.frombuffer(self.data[2], dtype = np.uint8)
        expected_values = height * width * channels
        if frame.size != expected_values:
            raise ValueError(
                'Invalid rgb_image payload size: expected {}, got {}.'.format(expected_values, frame.size)
            )
        if channels == 1:
            image = frame.reshape((height, width))
        else:
            image = frame.reshape((height, width, channels))
        cv2.imshow('rgb_image', image)
        cv2.waitKey(1)

    def check_connection(self):
        self._reinit_counter()
        while True:
            if self.data_type == 'FloatArray':
                self.data = self.socket.recv()
            else:
                self.data = self.socket.recv_multipart()
            if self.data is not None and self.data is not self.previous_data:
                # To see the data - usually reduces the actual frequency. Use it to just see the stream
                if self.print_data:
                    if self.data_type == 'FloatArray':
                        self._decode_array()
                    else:
                        self._decode_rgb_image()

                self.counter += 1
                print('Frequency: {}'.format(self._calculate_frequency()))

                if self.counter > 10:
                    self._reinit_counter()
            else:
                self.start_time = time.time()
