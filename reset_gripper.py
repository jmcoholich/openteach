from time import sleep
# deoxys_control
from deoxys.franka_interface import FrankaInterface

def main():
    robot_interface = FrankaInterface("/home/ripl/deoxys_control/deoxys/config/charmander.yml", use_visualizer=False)  # hardcoded path to config

    sleep(1)
    robot_interface.gripper_control(-1)
    sleep(2)

if __name__ == '__main__':
    main()