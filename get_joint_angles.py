import argparse
import os
import time

import numpy as np
from deoxys.franka_interface import FrankaInterface

DEFAULT_CONFIG = os.path.join(
    os.path.expanduser("~"),
    "deoxys_control/deoxys/config/charmander.yml",
)


def main():
    parser = argparse.ArgumentParser(description="Print the current Franka joint angles.")
    parser.add_argument(
        "--config",
        default=DEFAULT_CONFIG,
        help="Path to the Deoxys robot config.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=5.0,
        help="Seconds to wait for the first robot state.",
    )
    args = parser.parse_args()

    robot_interface = FrankaInterface(args.config, use_visualizer=False)

    start_time = time.time()
    while robot_interface.last_q is None:
        if time.time() - start_time > args.timeout:
            raise TimeoutError("Timed out waiting for robot joint state.")
        time.sleep(0.05)

    joint_angles = np.asarray(robot_interface.last_q)
    eef_pose = np.asarray(robot_interface.last_eef_pose)
    print(np.array2string(joint_angles, precision=6, separator=", "))
    # print(np.array2string(eef_pose, precision=6, separator=", "))

if __name__ == "__main__":
    main()
