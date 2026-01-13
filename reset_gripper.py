# deoxys_control
from deoxys.franka_interface import FrankaInterface

def main():
    robot_interface = FrankaInterface("/home/ripl/deoxys_control/deoxys/config/charmander.yml", use_visualizer=False)  # hardcoded path to config

    while robot_interface.last_gripper_q is None or robot_interface.last_gripper_q < 0.07:
        robot_interface.gripper_control(-1)


if __name__ == '__main__':
    main()