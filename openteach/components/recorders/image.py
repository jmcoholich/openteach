import os
import time

import cv2
import numpy as np

from openteach.constants import (
    CAM_FPS,
    CAM_FPS_SIM,
    IMAGE_RECORD_RESOLUTION,
    IMAGE_RECORD_RESOLUTION_SIM,
)
from openteach.utils.files import store_pickle_data
from openteach.utils.network import ZMQCameraSubscriber
from openteach.utils.timer import FrequencyTimer

from .recorder import Recorder


def _video_fourcc(codec):
    codec = str(codec)
    if len(codec) != 4:
        raise ValueError('OpenCV video codec must be a four-character code, got {}.'.format(codec))

    return cv2.VideoWriter_fourcc(*codec)


# To record realsense streams
class RGBImageRecorder(Recorder):
    def __init__(
        self,
        host,
        image_stream_port,
        storage_path,
        filename,
        video_codec='FFV1',
        sim=False
    ):
        self.notify_component_start('RGB stream: {}'.format(image_stream_port))

        # Subscribing to the image stream port
        self._host, self._image_stream_port = host, image_stream_port
        self.image_subscriber = ZMQCameraSubscriber(
            host = host,
            port = image_stream_port,
            topic_type = 'RGB'
        )
        self.sim = sim
        # Timer
        if self.sim:
            self.timer = FrequencyTimer(CAM_FPS_SIM)
        else:
            self.timer = FrequencyTimer(CAM_FPS)

        # Storage path for file
        self._filename = filename
        self._recorder_file_name = os.path.join(storage_path, filename + '.avi')
        self._metadata_filename = os.path.join(storage_path, filename + '.metadata')
        self._video_codec = video_codec

        # Initializing the recorder
        if self.sim:
            self.recorder = cv2.VideoWriter(
                self._recorder_file_name,
                _video_fourcc(video_codec),
                CAM_FPS_SIM,
                IMAGE_RECORD_RESOLUTION_SIM
            )
        else:
            self.recorder = cv2.VideoWriter(
                self._recorder_file_name,
                _video_fourcc(video_codec),
                CAM_FPS,
                IMAGE_RECORD_RESOLUTION
            )
        self.timestamps = []
        self.timestamp_domains = []
        self.capture_timestamps = []
        self.capture_timestamp_domains = []


    def stream(self):
        print('Starting to record RGB frames from port: {}'.format(self._image_stream_port))

        self.num_image_frames = 0
        self.record_start_time = time.time()

        while True:
            try:
                self.timer.start_loop()
                image, timestamp = self.image_subscriber.recv_rgb_image()
                frame_timing = getattr(self.image_subscriber, 'last_rgb_timing', {})
                # Availability at this subscriber, before video encoding/disk I/O (milliseconds).
                available_ms = frame_timing.get('host_decoded_unix_s', time.time()) * 1000
                self.recorder.write(image)
                self.timestamps.append(available_ms)
                self.timestamp_domains.append('unix')
                self.capture_timestamps.append(frame_timing.get('timestamp_ms', timestamp))
                self.capture_timestamp_domains.append(frame_timing.get('timestamp_domain'))
                self.num_image_frames += 1
                self.timer.end_loop()
            except KeyboardInterrupt:
                    self.record_end_time = time.time()
                    break

        # Closing the socket
        self.image_subscriber.stop()

        # Displaying statistics
        self._display_statistics(self.num_image_frames)

        # Saving the metadata
        self._add_metadata(self.num_image_frames)
        self.metadata['timestamps'] = self.timestamps
        self.metadata['timestamp_domains'] = self.timestamp_domains
        self.metadata['timestamp_source'] = 'recorder_receive_decode'
        self.metadata['timestamp_units'] = 'ms'
        self.metadata['capture_timestamps'] = self.capture_timestamps
        self.metadata['capture_timestamp_domains'] = self.capture_timestamp_domains
        self.metadata['recorder_ip_address'] = self._host
        self.metadata['recorder_image_stream_port'] = self._image_stream_port
        self.metadata['video_codec'] = self._video_codec

        # Storing the data
        print('Storing the final version of the video...')
        self.recorder.release()
        store_pickle_data(self._metadata_filename, self.metadata)
        print('Stored the video in {}.'.format(self._recorder_file_name))
        print('Stored the metadata in {}.'.format(self._metadata_filename))


class FishEyeImageRecorder(Recorder):
    def __init__(
        self,
        host,
        image_stream_port,
        storage_path,
        filename,
        video_codec='FFV1'
    ):
        self.notify_component_start('RGB stream: {}'.format(image_stream_port))

        # Subscribing to the image stream port
        print("Image Stream Port", image_stream_port)
        self._host, self._image_stream_port = host, image_stream_port
        self.image_subscriber = ZMQCameraSubscriber(
            host = host,
            port = image_stream_port,
            topic_type = 'RGB'
        )

        # Timer
        self.timer = FrequencyTimer(CAM_FPS)

        # Storage path for file
        self._filename = filename
        self._recorder_file_name = os.path.join(storage_path, filename + '.avi')
        self._metadata_filename = os.path.join(storage_path, filename + '.metadata')
        self._pickle_filename = os.path.join(storage_path, filename + '.pkl')
        self._video_codec = video_codec

        # Initializing the recorder
        self.recorder = cv2.VideoWriter(
            self._recorder_file_name,
            _video_fourcc(video_codec),
            CAM_FPS,
            IMAGE_RECORD_RESOLUTION
        )
        self.timestamps = []
        self.frames = []




    def stream(self):
        print('Starting to record RGB frames from port: {}'.format(self._image_stream_port))

        self.num_image_frames = 0
        self.record_start_time = time.time()

        while True:
            try:
                self.timer.start_loop()
                image, timestamp = self.image_subscriber.recv_rgb_image()
                self.recorder.write(image)
                self.timestamps.append(timestamp)

                self.frames.append(np.array(image))

                self.num_image_frames += 1
                self.timer.end_loop()
            except KeyboardInterrupt:
                self.record_end_time = time.time()
                break
        self.image_subscriber.stop()

        # Displaying statistics
        self._display_statistics(self.num_image_frames)

        # Saving the metadata
        self._add_metadata(self.num_image_frames)
        self.metadata['timestamps'] = self.timestamps
        self.metadata['recorder_ip_address'] = self._host
        self.metadata['recorder_image_stream_port'] = self._image_stream_port
        self.metadata['video_codec'] = self._video_codec

        # Storing the data
        print('Storing the final version of the video...')
        self.recorder.release()
        store_pickle_data(self._metadata_filename, self.metadata)
        print('Stored the video in {}.'.format(self._recorder_file_name))
        print('Stored the metadata in {}.'.format(self._metadata_filename))
