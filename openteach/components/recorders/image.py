import os
import time

import cv2
import h5py
import numpy as np

from openteach.constants import (
    CAM_FPS,
    CAM_FPS_SIM,
    DEPTH_RECORD_FPS,
    IMAGE_RECORD_RESOLUTION,
    IMAGE_RECORD_RESOLUTION_SIM,
)
from openteach.utils.files import store_pickle_data
from openteach.utils.network import ZMQCameraSubscriber
from openteach.utils.timer import FrequencyTimer

from .recorder import Recorder


def _normalize_hdf5_compression(compression):
    if compression is None:
        return None

    compression = str(compression).lower()
    if compression in ('', 'none', 'false', '0'):
        return None

    return compression


def _hdf5_dataset_kwargs(compression, compression_opts=None, shuffle=False):
    compression = _normalize_hdf5_compression(compression)
    if compression is None:
        return {}

    kwargs = dict(compression=compression)
    if compression == 'gzip' and compression_opts is not None:
        kwargs['compression_opts'] = int(compression_opts)
    if shuffle:
        kwargs['shuffle'] = True

    return kwargs


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
        self.metadata['recorder_ip_address'] = self._host
        self.metadata['recorder_image_stream_port'] = self._image_stream_port
        self.metadata['video_codec'] = self._video_codec

        # Storing the data
        print('Storing the final version of the video...')
        self.recorder.release()
        store_pickle_data(self._metadata_filename, self.metadata)
        print('Stored the video in {}.'.format(self._recorder_file_name))
        print('Stored the metadata in {}.'.format(self._metadata_filename))


class DepthImageRecorder(Recorder):
    def __init__(
        self,
        host,
        image_stream_port,
        storage_path,
        filename,
        compression='gzip',
        compression_opts=6,
        shuffle=False
    ):
        self.notify_component_start('Depth stream: {}'.format(image_stream_port))

        # Subscribing to the image stream port
        self._host, self._image_stream_port = host, image_stream_port
        self.image_subscriber = ZMQCameraSubscriber(
            host = host,
            port = image_stream_port,
            topic_type = 'Depth'
        )

        # Timer
        self.timer = FrequencyTimer(DEPTH_RECORD_FPS)

        # Storage path for file
        self._filename = filename
        self._recorder_file_name = os.path.join(storage_path, filename + '.h5')
        self._compression = _normalize_hdf5_compression(compression)
        self._compression_opts = compression_opts
        self._shuffle = shuffle

        # Intializing the depth data containers
        self.depth_frames = []
        self.timestamps = []

    def stream(self):
        if self.image_subscriber.recv_depth_image() is None:
            raise ValueError('Depth image stream is not active.')

        print('Starting to record depth frames from port: {}'.format(self._image_stream_port))

        self.num_image_frames = 0
        self.record_start_time = time.time()

        while True:
            try:
                self.timer.start_loop()
                depth_data, timestamp = self.image_subscriber.recv_depth_image()
                self.depth_frames.append(depth_data)
                self.timestamps.append(timestamp)

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
        self.metadata['recorder_ip_address'] = self._host
        self.metadata['recorder_image_stream_port'] = self._image_stream_port
        self.metadata['depth_compression'] = self._compression or 'none'
        self.metadata['depth_compression_opts'] = -1 if self._compression_opts is None else self._compression_opts
        self.metadata['depth_shuffle'] = self._shuffle

        # Writing to dataset - hdf5 is faster and compresses more than blosc zstd with clevel 9
        if self._compression is None:
            print('Saving depth data without HDF5 compression...')
        else:
            print('Compressing depth data with {}...'.format(self._compression))

        dataset_kwargs = _hdf5_dataset_kwargs(
            self._compression,
            compression_opts=self._compression_opts,
            shuffle=self._shuffle
        )
        with h5py.File(self._recorder_file_name, "w") as file:
            stacked_frames = np.array(self.depth_frames, dtype = np.uint16)
            file.create_dataset("depth_images", data = stacked_frames, **dataset_kwargs)

            timestamps = np.array(self.timestamps, np.float64)
            file.create_dataset("timestamps", data = timestamps, **dataset_kwargs)

            file.update(self.metadata)

        print('Saved depth data in {}.'.format(self._recorder_file_name))

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
