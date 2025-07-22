# Modified OPEN TEACH for use with Quest 3 Controllers

#### Authors: Jeremiah Coholich and Justin Wit
##### Original OPEN TEACH Authors: Aadhithya Iyer ,Zhuoran Peng, Yinlong Dai, Irmak Guzey, Siddhant Haldar, Soumith Chintala, Lerrel Pinto
[Paper](https://arxiv.org/abs/2403.07870) [Website](https://open-teach.github.io/)

We have modified OPEN TEACH simplifying the teleoperation stack and enabling teleoperation with the Quest 3 controller for Franka Arm with a gripper.

Previously we found that using the vision-based hand detection for teleoperation added a lot of noise to movements, had difficult grasp actuation with  clicking fingers, and was not necessary for parallel-jaw grippers.
Features
- Improved tracking performance
- Stop and reposition hands with button
- Trigger-actuated grasping
- Remove the camera streaming window from the app (we often found that it was in the way and block our pass-through view of the robot)

This repo has only been tested with a single Franka Emika Panda arm with the Franka gripper and a Meta Quest 3. We use Ubuntu 20.04 and record from three RealSense D435 cameras.

Here is a video of us using the new openteach:

This repo consists of two parts:
- Updated teleoperation control stack and data recording
- .apk file for the app


## Code Installation

Install [Deoxys Control](https://github.com/UT-Austin-RPL/deoxys_control).

<!-- This is the official implementation of the Open Teach including unity scripts for the VR application, teleoperation pipeline and demonstration collection pipeline.

Open Teach consists of two parts.

- [x] Teleoperation using Meta Quest 3 and data collection over a range of robot morphologies and simulation environments.

- [x] Policy training for various dexterous manipulation tasks across different robots and simulations. -->

### VR Installation and User Interface

#### Installation
This can be done in two ways.

1. You can install the application directly into your Quest using the APK file we provide using SideQuest.
2. Inorder to have more flexibility and modify the VR APK we have also released the source code corresponding to it. The APK is made in Unity 2021.3.5f1. To ensure clean builds with no errors using this version is recommended.
3. For building the apk from source , add the zipped code files to Unity and within the settings in File menu run Build and Run. The targeted device in Build settings should be the Quest you are connecting to. Also users might need Meta account with Meta Quest Developer Hub and will need to enable Developer mode for the Meta Quest Application.
4. To setup the VR Application in Oculus Headset, enter the IP Address of the robot server. The robot server and the Oculus Headset should be in the same network.

#### User Interface for Franka Arm + Franka Gripper:


(JUSTIN)
| Pinch ( Left Hand) | Mode                 | Stream Border Color |
| ------------------ | -------------------- | ------------------- |
| Index Pinch        | Only Hand Mode       | Green               |
| Middle Pinch       | Arm + Hand Mode      | Blue                |
| Ring Pinch         | Pause                | Red                 |
| Pinky Pinch        | Resolution Selection | Black               |

<!-- **Note: Here the teleoperation is just like mimicking the human hand and arm actions** -->

<!-- ## Multi Robot Arm (Bimanual):

Since both the hands are being used here for teleoperation and gripper mode selection we use pinches in both the hands. Due to the noise in hand pose detection while moving the hands , for better detection of pinches we use keypoint distance threshold based approach between two fingers. For our setup we use Xarms as bimanual robots.

| Pinch (On Both Hands) | Mode                                                         | Stream Border Color |
| ----------------------- | ------------------------------------------------------------ | ------------------- |
| Index Pinch             | Start the Teleop ( Only used at the start of the teleoperation ) | Green               |
| Middle Pinch            | Pause/Resume the Robot                                       | Red                 |
| Ring Pinch              | Pause/Resume the Robot                                       | Red                 |
| Pinky Pinch             | Gripper Open/Close                                           | Yellow              |


**Note: Here the teleoperation is not mimicking the arm actions. Like other bimanual teleoperation methods we imagine we are holding the end effector of the arm and rotating and translating accordingly** -->


The VR APK files are available [here](/VR/APK/).

After you install the APK file. You will be prompted with a blank screen with red border with a Menu button on it. Click the Menu button (Ensure you have Hand tracking enabled in the Oculus.), you will see IP: Not Defined. Just Click on Change IP and enter the IP using the dropdown (The VR and the Robot should be under the same network provider). Once the IP is enter go back to the screen where you clicked Change IP and Click Stream. The screen border will become green and your App is ready to stream the controller position data.

#### Note: Remember to enter your same IP on the server host address variable [config](/configs/network.yaml)

Once finished setting up the APK proceed to [teleop](/docs/teleop_data_collect.md).

If Teleoperation server is not started, the APK will work for sometime and stop as there are ports to send the information to.

<!-- ### Server Code Installation

Install the conda environment from the yaml file in the codebase

**Allegro Sim**

`conda env create -f env_isaac.yml`

**Others**

`conda env create -f environment.yml`

This will install all the dependencies required for the server code.

After installing all the prerequisites, you can install this pipeline as a package with pip:

`pip install -e . `

You can test if it had installed correctly by running ` import openteach` from the python shell.

### Robot Controller Installation Specific Information

1. For Simulation specific information, follow the instructions [here](/docs/simulation.md). -->

<!-- 2. For Robot controller installation, follow the instructions [here](https://github.com/NYU-robot-learning/OpenTeach-Controllers) -->


## Running Teleoperation

The script `teleoperate.bash` uses [xdotool](https://github.com/jordansissel/xdotool) to start everything automatically in a GNOME [Terminator](https://gnome-terminator.org/) window. If any other keyboard or mouse commands are entered before the script is finished, it may fail. To run the automated script:

```bash
bash teleoperate.bash <NUC_IP_ADDR>
```

Run on the NUC

```bash
cd deoxys_control/deoxys && ./auto_scripts/auto_arm.sh config/charmander.yml
```

In another NUC terminal:
```bash
cd deoxys_control/deoxys && ./auto_scripts/auto_gripper.sh config/charmander.yml
```

On the other workstation, run each set of commands in a new terminal:

Reset the robot joints to a default position:
```bash
conda activate openteach && cd ~/deoxys_control/deoxys/examples
python reset_robot_joints.py
```

Stream the cameras
```bash
conda activate openteach && cd ~/openteach
python robot_camera.py
```

Start recording of camera steams
```bash
conda activate openteach && cd ~/openteach
python data_collect.py robot=franka demo_num=<name for demo>
```

Start teloperation of the robot
```bash
conda activate openteach && cd ~/openteach
python teleop.py robot=franka record=<SAME name for the demo as previous cmd>
```

To stop recording, simply kill the above two scripts. To reset the arm, just run

```bash
cd ~/deoxys_control/deoxys/examples
python reset_robot_joints.py
cd ~/openteach
python reset_gripper.py
```

<!-- ### Policy Learning

For open-source code of the policies we trained on the robots refer [here](/docs/policy_learning.md)

### Policy Learning API

For using the API we use for policy learning, use [this](https://github.com/NYU-robot-learning/Open-Teach-API)

### Call for contributions

For adding your own robot and simulation refer [here](/docs/add_your_own_robot.md) -->

### Citation
Citation for the original OPEN TEACH paper:
```
@misc{iyer2024open,
      title={OPEN TEACH: A Versatile Teleoperation System for Robotic Manipulation},
      author={Aadhithya Iyer and Zhuoran Peng and Yinlong Dai and Irmak Guzey and Siddhant Haldar and Soumith Chintala and Lerrel Pinto},
      year={2024},
      eprint={2403.07870},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}



