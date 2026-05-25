import argparse

import cv2

from openteach.utils.network import ZMQCameraSubscriber


def opencv_highgui_unavailable(error):
    return (
        "The function is not implemented" in str(error)
        and "cvShowImage" in str(error)
    )


class OpenCVFrameViewer:
    def __init__(self, title):
        self.title = title

    def show(self, frame):
        cv2.imshow(self.title, frame)
        return cv2.waitKey(1) & 0xFF == ord('q')

    def close(self):
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass


class MatplotlibFrameViewer:
    def __init__(self, title):
        import matplotlib.pyplot as plt

        self.plt = plt
        self.plt.ion()
        self.figure, self.axis = self.plt.subplots(num=title)
        self.axis.axis('off')
        self.image = None
        self.closed = False
        self.quit_requested = False
        self.figure.canvas.mpl_connect('close_event', self._on_close)
        self.figure.canvas.mpl_connect('key_press_event', self._on_key_press)

    def _on_close(self, _event):
        self.closed = True

    def _on_key_press(self, event):
        if event.key == 'q':
            self.quit_requested = True
            self.plt.close(self.figure)

    def show(self, frame):
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        if self.image is None:
            self.image = self.axis.imshow(rgb_frame)
            self.figure.tight_layout(pad=0)
        else:
            self.image.set_data(rgb_frame)

        self.figure.canvas.draw_idle()
        self.plt.pause(0.001)
        return self.closed or self.quit_requested

    def close(self):
        self.plt.close(self.figure)

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
parser.add_argument('--viewer', choices=('auto', 'opencv', 'matplotlib'),
                    default='auto',
                    help='Frame viewer backend. Defaults to OpenCV with matplotlib fallback.')
args = parser.parse_args()

selected_port = port_map[args.cam]

# Set up the camera subscriber
image_subscriber = ZMQCameraSubscriber(
    host="172.16.0.1",
    port=selected_port,
    topic_type='RGB'
)

viewer = (
    MatplotlibFrameViewer('frame')
    if args.viewer == 'matplotlib'
    else OpenCVFrameViewer('frame')
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
try:
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
        try:
            should_quit = viewer.show(color_frame)
        except cv2.error as error:
            if args.viewer == 'opencv' or not opencv_highgui_unavailable(error):
                raise

            print(
                'OpenCV was installed without GUI window support; '
                'falling back to matplotlib. Press q in the window to quit.'
            )
            viewer.close()
            viewer = MatplotlibFrameViewer('frame')
            should_quit = viewer.show(color_frame)

        if should_quit:
            break
except KeyboardInterrupt:
    pass
finally:
    viewer.close()
