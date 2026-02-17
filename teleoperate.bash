#!/bin/bash
NUC_IP=${1:-172.16.0.3}
# Start Terminator
terminator &

# Give Terminator some time to start
sleep 1

# Send commands to Terminator to split and run your desired commands
xdotool key ctrl+shift+e
sleep 0.25
xdotool key ctrl+Tab
sleep 0.25
xdotool type "ssh ripl@$NUC_IP"
sleep 0.25
xdotool key Return
sleep 2.0
xdotool type "cd deoxys_control/deoxys && ./auto_scripts/auto_arm.sh config/charmander.yml"
sleep 0.25
xdotool key Return
sleep 0.25

xdotool key ctrl+shift+o
sleep 0.25
xdotool type "ssh ripl@$NUC_IP"
sleep 0.25
xdotool key Return
sleep 2.0
xdotool type "cd deoxys_control/deoxys && ./auto_scripts/auto_gripper.sh config/charmander.yml"
sleep 0.25
xdotool key Return
sleep 0.25

xdotool key ctrl+Tab
sleep 0.25
xdotool type 'conda activate openteach'
sleep 0.25
xdotool key Return
sleep 0.25

# xdotool type 'conda activate openteach && roslaunch franka_arm franka_arm.launch'
# sleep 0.25
# xdotool key Return
# sleep 0.25

xdotool key ctrl+shift+o
sleep 0.25
xdotool type 'conda activate openteach'
sleep 0.25
xdotool key Return
sleep 0.25
xdotool type 'python teleop.py robot=franka record='
sleep 0.25
xdotool key ctrl+shift+e
sleep 0.25
xdotool type 'conda activate openteach && cd ~/deoxys_control/deoxys/examples '
sleep 0.25
xdotool key Return
sleep 0.25
xdotool type 'python reset_robot_joints.py'

xdotool key ctrl+Tab
sleep 0.25
xdotool key ctrl+Tab
sleep 0.25
xdotool key ctrl+Tab
sleep 0.25
xdotool key ctrl+shift+e
sleep 0.25
xdotool type 'conda activate openteach && cd ~/openteach'
sleep 0.25
xdotool key Return
sleep 0.25
xdotool type 'python robot_camera.py'
sleep 0.25
xdotool key Return

xdotool key alt+Down
xdotool key alt+Left
sleep 0.25
xdotool key ctrl+shift+o
sleep 0.25
xdotool type 'conda activate openteach && cd ~/openteach'
sleep 0.25
xdotool key Return
sleep 0.25
xdotool type 'python data_collect.py robot=franka demo_num='

# control shift tab thrice
xdotool key alt+Right
sleep 0.25
xdotool key ctrl+shift+o
sleep 0.25
xdotool type 'conda activate openteach && cd ~/openteach'
sleep 0.25
xdotool key Return
sleep 0.25
xdotool type 'python reset_gripper.py'
sleep 0.25

# Open a pane at the top-left and run local Franka error monitoring.
xdotool key alt+Up
sleep 0.25
xdotool key alt+Up
sleep 0.25
xdotool key alt+Up
sleep 0.25
xdotool key alt+Left
sleep 0.25
xdotool key alt+Left
sleep 0.25
xdotool key alt+Left
sleep 0.25
xdotool key ctrl+shift+o
sleep 0.25
xdotool type 'conda activate openteach'
sleep 0.25
xdotool key Return
sleep 0.25
xdotool type 'python ~/deoxys_control/deoxys/auto_scripts/read_franka_errors.py'
sleep 0.25
xdotool key Return





exit 0
