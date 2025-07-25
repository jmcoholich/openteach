import hydra
from openteach.components import RealsenseCameras
import time
# https://github.com/IntelRealSense/librealsense/issues/6628#issuecomment-646558144
import pyrealsense2 as rs

# initiating hardware reset
ctx = rs.context()
devices = ctx.query_devices()
print(devices)
for dev in devices:
    print("resetting device: ", dev.get_info(rs.camera_info.serial_number))
    dev.hardware_reset()
    print(" done")

import time; time.sleep(2)


@hydra.main(version_base = '1.2', config_path = 'configs', config_name = 'camera')
def main(configs):
    cameras = RealsenseCameras(configs)
    processes = cameras.get_processes()

    for process in processes:
        process.start()
        time.sleep(5)  # this prevents errors resulting from processes blocking each other's access to the realsense cameras

    for process in processes:
        process.join()
        time.sleep(5)

if __name__ == '__main__':
    main()