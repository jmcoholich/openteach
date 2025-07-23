# OPEN TEACH for Quest 3 Controller Teleoperation

#### Fork authors: [Jeremiah Coholich](https://www.jeremiahcoholich.com/) and [Justin Wit](https://www.linkedin.com/in/justin-wit/)
Original OPEN TEACH Authors: Aadhithya Iyer, Zhuoran Peng, Yinlong Dai, Irmak Guzey, Siddhant Haldar, Soumith Chintala, Lerrel Pinto
[OPEN TEACH Paper](https://arxiv.org/abs/2403.07870)
[OPEN TEACH Website](https://open-teach.github.io/)

This OPEN TEACH fork simplifies the software stack and enables teleoperation with the Quest 3 controller instead of hand tracking. We have only used this code for teleoperating a Franka Emika Panda arm with the Franka gripper in the real world with a Meta Quest 3.


## Summary of Changes

<!-- Add image before.png -->
![Original OPEN TEACH control flow](before.png)
![Control flow with our modifications](after.png)

Previously we found that using the vision-based hand detection for teleoperation added a lot of noise to movements, had difficult grasp actuation with  clicking fingers, and was not necessary for parallel-jaw grippers.

Features of our changes:
- Improved tracking performance
- Stop and reposition hand with button
- Trigger-actuated grasping
- Remove the camera streaming window from the app (we often found that it was in the way and block our pass-through view of the robot)

This repo has only been tested with a single Franka Emika Panda arm with the Franka gripper and a Meta Quest 3. We use Ubuntu 20.04 and record from three RealSense D435 cameras.

Here is a video of us using the new openteach:

This repo consists of two parts:
- Updated teleoperation control stack and data recording
- .apk file for the app


**We removed the requirement for ROS and the OpenTeach-Controllers repository entirely.** Many of our changes and additions can be found in the file [`openteach/components/operators/franka.py`](openteach/components/operators/franka.py)




## Code Installation

Install [Deoxys Control](https://github.com/UT-Austin-RPL/deoxys_control).

Clone this repository and create the conda environment:
```bash
conda env create -f environment.yml
```
(We highly recommend using [mamba](https://github.com/mamba-org/mamba) instead of conda for faster environment creation.)

<!-- This is the official implementation of the Open Teach including unity scripts for the VR application, teleoperation pipeline and demonstration collection pipeline.

Open Teach consists of two parts.

- [x] Teleoperation using Meta Quest 3 and data collection over a range of robot morphologies and simulation environments.

- [x] Policy training for various dexterous manipulation tasks across different robots and simulations. -->

## VR Installation and User Interface

#### Installation
This can be done in two ways.

##### 1. Install Our APK

You can install the application directly into your Quest using the [APK file](VR/APK/FrankaRemoteTrackingV2.apk) we provide using [SideQuest]('https://sidequestvr.com/').

##### 2. Build from Source

* In order to have more flexibility and modify the VR APK we have also released the source code corresponding to it. The APK is made in Unity 2021.3.5f1. To ensure clean builds with no errors using this version is recommended.
* For building the apk from source , add the [VR code files](/VR/Franka-Bot-Unity/) to Unity and within the settings in File menu run Build and Run. The targeted device in Build settings should be the Quest you are connecting to. Also users might need Meta account with Meta Quest Developer Hub and will need to enable Developer mode for the Meta Quest Application.


#### User Interface for Franka Arm + Franka Gripper:

Only requires use of the right hand controller. Swich modes with the **B** button and operate the gripper with the **Trigger** button. Note switching to the 'Pause' mode while operating will stop sending commands to the robot and will reinitialize the origin after unpausing, allowing the user to continue teleoperation from a new position.

 Mode                  | Remote Indicator |
| -------------------- | ---------------------- |
| Network Config       | Pink                   |
| Teleoperate          | Green                  |
| Pause                | Red                    |

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


<!-- The VR APK files are available [here](/VR/APK/). -->

After you install the APK file, launch the app on the headset. You will be prompted with a pink controller and a menu button. If you do not see a laser pointer from the remote click **B**. Click the Menu button then click on 'Change IP Address' and enter the IP using the dropdown (use the trigger button to scroll). The VR and the Robot should be under the same network provider. Once the IP is entered click 'Change' and then click 'Stream'. The remote will become red and your app is ready to stream the controller position data.

#### Note: Remember to enter your same IP on the server host address variable [config](/configs/network.yaml)

Once finished setting up the APK proceed to **Running Teleoperation**.


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
### Breakdown of automated script:
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
## Data files
After stopping the `teleop.py` script, a file named `deoxys_obs_cmd_history_<demo name>.pkl` will be saved in `openteach/extracted_data`. This pkl file will contain all robot state/observations (except camera inputs) and actions saved by `openteach/openteach/components/operators/franka.py`.

After stopping the `data_collect.py` script, a folder named `demonstration_<demo name>` will be saved in the same folder. This folder contains the RGBD camera recordings.

To post-process this data, run

```bash
python visualize_demo.py --demo_number <demo name>
```

This script will combine the camera observations and robot actions/proprioception into a single pkl file `openteach/extracted_data/demonstration_<demo name>/demo_<demo name>.pkl`. Additionally, the script will save a video of the demo at `openteach/extracted_data/demonstration_<demo name>/demo_<demo name>.mp4`. Below is a link to an example of such a video. (Clicking the image will navigate to Youtube and play the video.)

[![Watch the video](https://img.youtube.com/vi/8bc508QxUwo/0.jpg)](https://youtu.be/8bc508QxUwo)


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
```
