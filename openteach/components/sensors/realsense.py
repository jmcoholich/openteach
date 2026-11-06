import time

import numpy as np
import pyrealsense2 as rs

from openteach.components import Component
from openteach.constants import *
from openteach.utils.images import rescale_image, rotate_image
from openteach.utils.network import ZMQCameraPublisher, ZMQCompressedImageTransmitter
from openteach.utils.timer import FrequencyTimer


class RealsenseCamera(Component):
    def __init__(self, stream_configs, cam_serial_num, cam_id, cam_configs, stream_oculus = False):
        # Disabling scientific notations
        np.set_printoptions(suppress=True)
        self.cam_id = cam_id
        self.cam_configs = cam_configs
        self._cam_serial_num = cam_serial_num
        self._stream_configs = stream_configs
        self._stream_oculus = stream_oculus

        # Different publishers to avoid overload
        self.rgb_publisher = ZMQCameraPublisher(
            host = stream_configs['host'],
            port = stream_configs['port'],
            rgb_encoding = cam_configs.get('rgb_encoding', 'png'),
        )

        if self._stream_oculus:
            self.rgb_viz_publisher = ZMQCompressedImageTransmitter(
                host = stream_configs['host'],
                port = stream_configs['port'] + VIZ_PORT_OFFSET
            )
        self.timer = FrequencyTimer(CAM_FPS)

        # Starting the realsense pipeline
        self._start_realsense(self._cam_serial_num)

    def _start_realsense(self, cam_serial_num):
        config = rs.config()
        self.pipeline = rs.pipeline()
        config.enable_device(cam_serial_num)

        # Enabling camera streams
        config.enable_stream(
            rs.stream.color,
            self.cam_configs.width,
            self.cam_configs.height,
            rs.format.bgr8,
            self.cam_configs.fps
        )
        # Starting the pipeline
        self.pipeline.start(config)
        self.realsense = self.pipeline

    def get_rgb_image(self):
        frames = None

        while frames is None:
            # The pipeline contains only the color stream.
            frames = self.realsense.wait_for_frames()
            received_at = time.time()
            rgb_frame = frames.get_color_frame()
            self._rgb_frame_timing = {
                "timestamp_ms": rgb_frame.get_timestamp(),
                "timestamp_domain": str(rgb_frame.get_frame_timestamp_domain()),
                "frame_number": rgb_frame.get_frame_number(),
                "host_frames_received_unix_s": received_at,
            }
            # Native device-clock metadata is in microseconds, when supported.
            for name in ("sensor_timestamp", "frame_timestamp"):
                key = getattr(rs.frame_metadata_value, name)
                if rgb_frame.supports_frame_metadata(key):
                    self._rgb_frame_timing[name + "_us"] = rgb_frame.get_frame_metadata(key)
            color_image = np.asanyarray(rgb_frame.get_data())

        self._rgb_frame_timing["host_rgb_ready_unix_s"] = time.time()
        return color_image, self._rgb_frame_timing["timestamp_ms"]

    def stream(self):
        # Starting the realsense stream
        self.notify_component_start('realsense')
        print(f"Started the Realsense pipeline for camera: {self._cam_serial_num}!")
        print("Starting stream on {}:{}...\n".format(self._stream_configs['host'], self._stream_configs['port']))

        if self._stream_oculus:
            print('Starting oculus stream on port: {}\n'.format(self._stream_configs['port'] + VIZ_PORT_OFFSET))

        while True:
            #try:
                self.timer.start_loop()
                color_image, timestamp = self.get_rgb_image()

                color_image = rotate_image(color_image, self.cam_configs.rotation_angle)

                # Publishing the rgb images
                self.rgb_publisher.pub_rgb_image(color_image, timestamp, frame_timing=self._rgb_frame_timing)
                # TODO - move the oculus publisher to a separate process - this cycle works at 40 FPS
                if self._stream_oculus:
                    self.rgb_viz_publisher.send_image(rescale_image(color_image, 2)) # 640 * 360

                self.timer.end_loop()
            # except KeyboardInterrupt:
            #     break

        print('Shutting down realsense pipeline for camera {}.'.format(self.cam_id))
        self.rgb_publisher.stop()
        if self._stream_oculus:
            self.rgb_viz_publisher.stop()
        self.pipeline.stop()
