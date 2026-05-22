import os
import time

from deoxys.franka_interface import FrankaInterface


def main():
    robot_interface = FrankaInterface(
        os.path.join(os.path.expanduser("~"), "deoxys_control/deoxys/config/charmander.yml"),
        use_visualizer=False,
    )

    robot_interface.gripper_stop()
    time.sleep(0.2)

    timeout = 5.0
    start_time = time.time()
    last_print_time = 0.0
    command_count = 0

    while robot_interface.last_gripper_q is None or robot_interface.last_gripper_q > 0.005:
        now = time.time()
        if now - start_time > timeout:
            print("close_gripper timed out waiting for gripper to close.", flush=True)
            break

        if now - last_print_time >= 1.0:
            gripper_state = robot_interface.last_gripper_state
            gripper_q = None if gripper_state is None else gripper_state.width
            max_width = None if gripper_state is None else gripper_state.max_width
            is_grasped = None if gripper_state is None else gripper_state.is_grasped
            state_count = robot_interface.gripper_state_buffer_size
            print(
                f"close_gripper diagnostic: last_gripper_q={gripper_q}, "
                f"max_width={max_width}, "
                f"is_grasped={is_grasped}, "
                f"gripper_state_buffer_size={state_count}, "
                f"close_commands_sent={command_count}",
                flush=True,
            )
            last_print_time = now

        robot_interface.gripper_control(1)
        command_count += 1
        time.sleep(0.05)


if __name__ == "__main__":
    main()
