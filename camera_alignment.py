import argparse

import cv2

from openteach.utils.network import ZMQCameraSubscriber

# Port mapping: 0 - top, 1 - side, 2 - front
port_map = {
    0: 10005,
    1: 10006,
    2: 10007,
}

# Parse positional argument for camera index
parser = argparse.ArgumentParser(description='Camera stream viewer')
parser.add_argument('cam', type=int, choices=port_map.keys(),
                    help='Camera index: 0 - top, 1 - side, 2 - front')
args = parser.parse_args()

selected_port = port_map[args.cam]

# Set up the camera subscriber
image_subscriber = ZMQCameraSubscriber(
    host="172.16.0.1",
    port=selected_port,
    topic_type='RGB'
)
# change this so the
# # Intrinsics
# w, h = 640, 360
# sf = 1.0
# cx = 314.9449462890625 * sf
# cy = 181.75074768066406 * sf
# fx = 456.4324951171875 * sf
# fy = 456.5205078125 * sf

# Display loop
while True:
    frames = image_subscriber.recv_rgb_image()
    color_frame = frames[0]
    h, w = color_frame.shape[:2]

    # Draw the centered square crop boundary.
    crop_size = min(h, w)
    x0 = (w - crop_size) // 2
    y0 = (h - crop_size) // 2
    x1 = x0 + crop_size
    y1 = y0 + crop_size
    cv2.rectangle(color_frame, (x0, y0), (x1, y1), (255, 0, 0), 2)

    # draw cross hairs on center
    cx, cy = w // 2, h // 2
    cv2.line(color_frame, (0, int(cy)), (w, int(cy)), (0, 255, 0), 2)
    cv2.line(color_frame, (int(cx), 0), (int(cx), h), (0, 255, 0), 2)

    # color_frame = color_frame[:, 280:1000]
    # cv2.line(color_frame, (0, int(cy)), (w, int(cy)), (0, 255, 0), 2)
    # cv2.line(color_frame, (int(cx), 0), (int(cx), h), (0, 255, 0), 2)
    cv2.imshow('frame', color_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
