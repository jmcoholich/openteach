"""
Combines and cleans data from franka arm and realsense cameras.

Outputs
- A video of the demonstration including joint angle plots and rgb and depth cams
- A .pkl file containing all processed data
"""

import h5py
import numpy as np
import matplotlib.pyplot as plt
import subprocess
from tqdm import tqdm
import os
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor, as_completed
import cv2
import pickle as pkl
from copy import copy

from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
# Set global matplotlib settings for better quality
plt.rcParams['text.antialiased'] = True
plt.rcParams['lines.antialiased'] = True
plt.rcParams['patch.antialiased'] = True

import argparse
import warnings
warnings.filterwarnings( "ignore")
DEBUG = False

# demo numer is the first argument
def main():
    parser = argparse.ArgumentParser()
    # add mutually exclusive args "demo_number" and "demo_folder"
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--demo_number", type=str, help="The number of the demonstration to process and visualize")
    group.add_argument("--demo_folder", type=str, help="Process and visualize all demos in folder.")
    args = parser.parse_args()

    if args.demo_number:
        make_combined_video(None, args.demo_number)

    elif args.demo_folder:
        data_root = f"{os.path.expanduser('~')}/openteach/extracted_data/{args.demo_folder}"
        if not os.path.exists(data_root):
            raise FileNotFoundError(f"Folder {data_root} does not exist. Please check the folder name and try again.")
        for file in os.listdir(data_root):
            if file.endswith(".pkl"):
                continue
            demo_number = file.split("_")[-1].split(".")[0]
            if os.path.exists(os.path.join(data_root, file, f"demo_{demo_number}.pkl")):
                print(f"Demo {demo_number} already processed. Skipping...")
                continue
            make_combined_video(args.demo_folder, demo_number)

    else:
        raise ValueError("Either --demo_number or --demo_folder must be provided")


def make_combined_video(folder, demo_number):
    root_folder = f"{os.path.expanduser('~')}/openteach/extracted_data"
    if folder is None:
        demo_path = os.path.join(root_folder, f"demonstration_{demo_number}")
        cmds_path = os.path.join(root_folder, f"deoxys_obs_cmd_history_{demo_number}.pkl")
    else:
        demo_path = os.path.join(root_folder, f"{folder}/demonstration_{demo_number}")
        cmds_path = os.path.join(root_folder, folder, f"deoxys_obs_cmd_history_{demo_number}.pkl")
    print(demo_path)
    depth_timestamps = []
    rgb_timestamps = []

    # freq = 15.0

    print('loading observations and commands ...')
    with open(cmds_path, "rb") as f:
        cmd_data = pkl.load(f)


    # print("Loading logged tcp commands...")
    # with h5py.File(f"{demo_path}/franka_arm_tcp_commands.h5", "r") as f:
    #     tcp_cmds = np.array(f["arm_tcp_commands"])
    #     for key in f.keys():
    #         if key in ["commands", "timestamps"]:
    #             continue
    #         if DEBUG: print(key.ljust(25), f[key][()])
    #     if DEBUG: print()
    #     tcp_cmd_timestamps = np.array(f["timestamps"])
    #     assert round(f['record_frequency'][()]) == freq

    # print("Loading gripper states...")
    # with h5py.File(f"{demo_path}/franka_gripper_state.h5", "r") as f:
    #     gripper_pos = np.array(f["positions"])
    #     gripper_cmd = np.array(f["commands"])
    #     for key in f.keys():
    #         if key in ["positions", "timestamps"]:
    #             continue
    #         if DEBUG: print(key.ljust(25), f[key][()])
    #     if DEBUG: print()
    #     gripper_timestamps = np.array(f["timestamps"])
    #     assert round(f['record_frequency'][()]) == freq

    # print("Loading joint states...")
    # cmd_metadata = {}
    # more_data = {}
    # with h5py.File(f"{demo_path}/franka_joint_states.h5", "r") as f:
    #     angles = np.array(f["positions"])
    #     # cmds = np.copy(angles)
    #     # cmds = np.array(f["commands"])
    #     more_data_keys = [
    #         "dq",
    #         "q_d",
    #         "dq_d",
    #         "ddq_d",
    #         "tau_J",
    #         "dtau_J",
    #         "tau_J_d",
    #         "tau_ext_hat_filtered",
    #         "eef_pose",
    #         "eef_pose_d",
    #         "F_T_EE",
    #         "F_T_NE",
    #     ]
    #     for key in f.keys():
    #         if key in ["positions", "timestamps"]:
    #             continue
    #         elif key[:-1] in more_data_keys:  # so annoying how the script adds an "s" to the end of the key
    #             more_data[key[:-1]] = np.array(f[key])
    #         if DEBUG: print(key.ljust(25), f[key][()])
    #         cmd_metadata[key] = f[key][()]
    #     if DEBUG: print()
    #     joint_state_timestamps = np.array(f["timestamps"])
    #     assert round(f['record_frequency'][()]) == freq

    # depth frames
    depth_frames = []
    for j in [0, 1, 2]:
        print(f"Loading depth images from cam_{j}...")
        with h5py.File(f"{demo_path}/cam_{j}_depth.h5", "r") as f:
            x = np.array(f['depth_images'])
            for key in f.keys():
                if key in ["orientations", "positions", "timestamps", "depth_images"]:
                    continue
                if DEBUG: print(key.ljust(25), f[key][()])
            if DEBUG: print()
            depth_timestamps.append(np.array(f["timestamps"]) / 1000)
            # assert round(f['record_frequency'][()]) == freq
        depth_frames.append(x)

    # rgb frames
    rgb_frames = []
    for j in [0, 1, 2]:
        print(f"Loading rgb images from cam_{j}...")
        fname = f"cam_{j}_rgb_video.avi"
        rgb_frames.append(load_video_to_numpy_array(f"{demo_path}/{fname}"))
        # load the metadata file (pkl file)
        # f"cam_{j}_rgb_video.metadata"
        with open(f"{demo_path}/cam_{j}_rgb_video.metadata", "rb") as f:
            metadata = pkl.load(f)
            for key in metadata.keys():
                if key in ["timestamps"]:
                    continue
                if DEBUG: print(key.ljust(25), metadata[key])
            if DEBUG: print()
            # assert round(metadata['record_frequency']) == freq
        rgb_timestamps.append(np.array(metadata["timestamps"]) / 1000)

    # max_depth_value = max([np.max(x) for x in depth_frames]) * 0.5
    max_depth_value = np.percentile(np.concatenate([x.flatten() for x in depth_frames]), 98)  # get rid of outliers

    output_data = {
        "cartesian_pose_cmd": [],
        "arm_action": [],
        "gripper_action": [],
        "gripper_state": [],
        "eef_quat": [],
        "eef_pos": [],
        "eef_pose": [],
        "joint_pos": [],
        "rgb_frames": [],
        "depth_frames": [],
        "timestamp": [],
        # "controller_type"
        # "controller_cfg"
        # "index"
    }
    all_cams_started_time = np.max([x[0] for x in rgb_timestamps] + [x[0] for x in depth_timestamps])
    cam_stopped = np.min([x[-1] for x in rgb_timestamps] + [x[-1] for x in depth_timestamps])
    for i in tqdm(range(len(cmd_data['index'])), desc="Processing data..."):
        # once the robot is stopped (by releasing deadman switch), the robot state stops updating but the commands continue
        # detect this and skip these frames
        if i != 0 and (cmd_data['joint_pos'][i] == cmd_data['joint_pos'][i - 1]).all():
            continue

        if cmd_data['timestamp'][i] < all_cams_started_time or cmd_data['timestamp'][i] > cam_stopped:  # throws away the last frame but thats fine
            continue
        output_data["cartesian_pose_cmd"].append(cmd_data['cartesian_pose_cmd'][i])
        output_data["arm_action"].append(cmd_data['arm_action'][i])
        output_data["gripper_action"].append(cmd_data['gripper_action'][i])
        output_data["gripper_state"].append(cmd_data['gripper_state'][i])
        output_data["eef_quat"].append(cmd_data['eef_quat'][i])
        output_data["eef_pos"].append(cmd_data['eef_pos'][i])
        output_data["eef_pose"].append(cmd_data['eef_pose'][i])
        output_data["joint_pos"].append(cmd_data['joint_pos'][i])
        output_data["timestamp"].append(cmd_data['timestamp'][i])

        # pick paired rgb and depth frames. Just pick the frame that comes immediately before the timestamp
        curr_rgb_frames = []
        curr_depth_frames = []
        for j in range(3):
            # pick the smallest value that is not negative
            temp = cmd_data['timestamp'][i] - rgb_timestamps[j]
            temp[temp < 0] = np.inf
            idx = np.argmin(temp)
            curr_rgb_frames.append(rgb_frames[j][idx])
            temp = cmd_data['timestamp'][i] - depth_timestamps[j]
            temp[temp < 0] = np.inf
            idx = np.argmin(temp)
            curr_depth_frames.append(depth_frames[j][idx])
        output_data[f"rgb_frames"].append(curr_rgb_frames)
        output_data[f"depth_frames"].append(curr_depth_frames)

    for k, v in output_data.items():
        output_data[k] = np.array(v)
    output_data["controller_type"] = cmd_data["controller_type"]
    output_data["controller_cfg"] = cmd_data["controller_cfg"]
    path = f"{demo_path}/demo_{demo_number}.pkl"
    print(f"Saving processed data to {path}...")
    with open(path, "wb") as f:
        pkl.dump(output_data, f)

    # make video
    frames_dir = f"{demo_path}/combined_frames"
    if not os.path.exists(frames_dir):
        os.makedirs(frames_dir)
    else:
        subprocess.run(f"rm -r {frames_dir}", shell=True)
        os.makedirs(frames_dir)


    # print("Loading cartesion position data...")
    # fname = "franka_cartesian_states.h5"
    # with h5py.File(f"{demo_path}/{fname}", "r") as f:
    #     cartesian_quats = np.array(f["orientations"])
    #     cartesian_pos = np.array(f["positions"])
    #     for key in f.keys():
    #         if key in ["orientations", "positions", "timestamps"]:
    #             continue
    #         if DEBUG: print(key.ljust(25), f[key][()])
    #     if DEBUG: print()
    #     cartesian_timestamps = np.array(f["timestamps"])
    #     assert round(f['record_frequency'][()]) == freq
    # num_cartesian_frames = cartesian_quats.shape[0]

    # if DEBUG:
    #     # Print number of frames for each component.
    #     print("\nNumber of frames for each component:")
    #     just_val = 17
    #     for i in range(3):
    #         print(f"rgb cam_{i}: ".ljust(just_val) , f"{rgb_frames[i].shape[0]}")
    #         print(f"depth cam_{i}:".ljust(just_val) , f"{depth_frames[i].shape[0]}")
    #     # print("cartesian:".ljust(just_val), num_cartesian_frames)
    #     # print( "joint positions:".ljust(just_val), angles.shape[0], '\n')
    #     # print("gripper:".ljust(just_val), gripper_pos.shape[0])
    #     # print("tcp cmds:".ljust(just_val), tcp_cmds.shape[0])

    # all_timestamps = (
    #     rgb_timestamps
    #     + depth_timestamps
    #     + [cartesian_timestamps, joint_state_timestamps, gripper_timestamps, tcp_cmd_timestamps]
    #     )
    # # to avoid referencing these old unchanged variables after changes are made to all_timestamps
    # del rgb_timestamps, depth_timestamps, cartesian_timestamps, joint_state_timestamps, gripper_timestamps
    # series_list = [
    #     [rgb_frames[0]], # "rgb_timestamp_0":
    #     [rgb_frames[1]], # "rgb_timestamp_1"
    #     [rgb_frames[2]], # "rgb_timestamp_2"
    #     [depth_frames[0]], # "depth_timestamp_0"
    #     [depth_frames[1]], # "depth_timestamp_1"
    #     [depth_frames[2]], # "depth_timestamp_2"
    #     [cartesian_quats, cartesian_pos], # "cartesian_timestamps"
    #     [angles] + [more_data[key] for key in more_data_keys], # "joint_state_timestamps"
    #     [gripper_pos, gripper_cmd], # "gripper_state_timestamps"
    #     [tcp_cmds], # "tcp_cmd_timestamps"
    # ]
    # assert len(series_list) == len(all_timestamps), "Number of series and timestamps must be the same"

    # # repair missing and duplicate timestamps.
    # num_missing = 0
    # num_duplicates = 0
    # for j, x in enumerate(all_timestamps):
    #     missing_idcs = []
    #     missing_tstamps = []
    #     for i in range(1, len(x)):
    #         if x[i] - x[i - 1] > 1 / freq * 1.5:  # if the gap is more than 1.5 times expected gap between frames, consider it missed
    #             num_missing += 1
    #             missing_idcs.append(i)
    #             missing_tstamps.append((x[i] + x[i - 1]) / 2)  # missing tstamp will be avg of neighbors
    #         elif x[i] == x[i - 1]:
    #             num_duplicates += 1
    #             x[i] += 1 / freq
    #     all_timestamps[j] = np.insert(x, missing_idcs, missing_tstamps)
    #     if missing_idcs:
    #         for k, individual_series in enumerate(series_list[j]):
    #             # fill missing datapoints with the previous value
    #             series_list[j][k] = np.insert(
    #                 individual_series,
    #                 missing_idcs,
    #                 [individual_series[l - 1] for l in missing_idcs],
    #                 axis=0
    #                 )

    # print(f"\nNumber of repaired missing timestamps: {num_missing}")
    # print(f"Number of repaired duplicate timestamps: {num_duplicates}")

    # # skip bad files
    # if num_missing > 50 or num_duplicates > 50:
    #     print("Skipping bad files...")
    #     return
    # assert num_missing < 50, "Many missing timestamps, consider rerecording demo"
    # assert num_duplicates < 50, "Many duplicate timestamps, consider rerecording demo"

    # # reassign cleaned data back to original variables
    # rgb_frames = [x[0] for x in series_list[:3]]
    # depth_frames = [x[0] for x in series_list[3:6]]
    # cartesian_quats = series_list[6][0]
    # cartesian_pos = series_list[6][1]
    # angles = series_list[7][0]
    # # cmds = series_list[7][1]
    # gripper_pos = series_list[8][0]
    # gripper_cmd = series_list[8][1]

    # more_data = {}
    # for i, key in enumerate(more_data_keys):  # reassign the more_data dict with cleaned data
    #     more_data[key] = series_list[7][i + 1]
    # tcp_cmds = series_list[9][0]


    # start_idcs, end_idcs = tstamp_syncing(all_timestamps, demo_path, freq)

    joint_futures = []
    cartesian_futures = []
    workers = 8
    num_frames = len(output_data["timestamp"])
    print(f"Total number of final frames in demo: {num_frames}")

    # print("\nGenerating joint state plots and cartesian plots...\n")
    print("\nGenerating joint state plots...\n")
    # split the range up into equal parts equal to the number of workers
    with ProcessPoolExecutor(max_workers=workers) as executor:
        chunk_size = num_frames // (workers - 1)
        remainder = num_frames % (workers - 1)
        for i in range(workers - 1):
            start = i * chunk_size
            end = (i + 1) * chunk_size
            joint_futures.append(executor.submit(
                make_joint_state_plots,
                output_data["joint_pos"],
                # more_data["q_d"][start_idcs[7]: end_idcs[7]],
                output_data["gripper_state"],
                output_data["gripper_action"],
                np.arange(start, end)
                ))
            # cartesian_futures.append(executor.submit(
            #     make_cartesian_frame,
            #     output_data["eef_pos"][start: end],
            #     output_data["eef_quat"][start: end],
            #     ))
        if remainder > 0:
            # add the remainder
            joint_futures.append(executor.submit(
                make_joint_state_plots,
                output_data["joint_pos"],
                # more_data["q_d"][start_idcs[7]: end_idcs[7]],
                output_data["gripper_state"],
                output_data["gripper_action"],
                np.arange(end, num_frames)
                ))
            # cartesian_futures.append(executor.submit(
            #     make_cartesian_frame,
            #     output_data["eef_pos"][end:],
            #     output_data["eef_quat"][end:],
            #     ))

        joint_state_plots = []
        cartesian_frames = []
        for future in joint_futures: # needs to be in order
            joint_state_plots.extend(future.result())
        # for future in cartesian_futures:
        #     cartesian_frames.extend(future.result())


    # idcs = []
    # for i in range(8):
    #     idcs.append(range(start_idcs[i], end_idcs[i]))

    # clear and recreate frames dir
    frames_dir = f"{demo_path}/combined_frames"
    if not os.path.exists(frames_dir):
        os.makedirs(frames_dir)
    else:
        run_cmd(f"rm -r {frames_dir}")
        os.makedirs(frames_dir)

    with ThreadPoolExecutor() as executor:
        futures = []
        progress_bar = tqdm(total=num_frames, desc="Saving combined frames...")
        for i in range(num_frames):

            futures.append(executor.submit(
                make_combined_frame,
                [output_data["depth_frames"][i, 0], output_data["depth_frames"][i, 1], output_data["depth_frames"][i, 2]],
                [output_data["rgb_frames"][i, 0], output_data["rgb_frames"][i, 1], output_data["rgb_frames"][i, 2]],
                None,  # cartesian_frames[i],
                joint_state_plots[i],
                i,
                max_depth_value,
                frames_dir,
                ))

        for future in as_completed(futures):
            future.result()
            progress_bar.update(1)

    # compile video
    compile_video(f"demo_{demo_number}", frames_dir, demo_path)

    # # save all processed data to a .pkl file
    # data = {
    #     "franka_tcp_cmds": tcp_cmds[start_idcs[9]: end_idcs[9]],
    #     "depth_imgs": [x[start_idcs[i]: end_idcs[i]] for i, x in zip([3, 4, 6], depth_frames)],
    #     "rgb_imgs": [x[start_idcs[i]: end_idcs[i]] for i, x in zip([0, 1, 2], rgb_frames)],
    #     "eef_pos": cartesian_pos[start_idcs[6]: end_idcs[6]],
    #     "eef_quat": cartesian_quats[start_idcs[6]: end_idcs[6]],
    #     "joint_angles": angles[start_idcs[7]: end_idcs[7]],
    #     "gripper_state": gripper_pos[start_idcs[8]: end_idcs[8]],
    #     "gripper_cmd": gripper_cmd[start_idcs[8]: end_idcs[8]],
    #     # "arm_cmd": cmds[start_idcs[7]: end_idcs[7]],
    #     "cmd_metadata": cmd_metadata,
    #     "timestamps": all_timestamps[0][start_idcs[0]: end_idcs[0]],  # arbitrary choice of zero-idx timestamps
    #     }
    # for key, val in more_data.items():
    #     data[key] = val[start_idcs[7]: end_idcs[7]]
    # for key, val in data.items():
    #     if isinstance(val, list):
    #         for x in val:
    #             assert len(x) == num_frames, f"Length of {key} is {len(x)} but should be {num_frames}"
    #     elif isinstance(val, dict):
    #         continue
    #     else:
    #         assert len(val) == num_frames, f"Length of {key} is {len(val)} but should be {num_frames}"
    # print()
    # print("Saving processed data to .pkl file...")
    # with open(f"{demo_path}/demo_{demo_number}.pkl", "wb") as f:
    #     pkl.dump(data, f)


# def tstamp_syncing(all_tstamps, demo_path, freq):
#     """This is some spaghetti but it works. Likely there is a clever or
#     established way to do this.
#     """

#     # ref idx should be shortest series
#     lens = [len(x) for x in all_tstamps]
#     ref = lens.index(min(lens))
#     ref_len = len(all_tstamps[ref])
#     min_shifts = []
#     start_idcs = []
#     end_idcs = []
#     min_errors = []
#     labels = ["rgb0", "rgb1", "rgb2", "depth0", "depth1", "depth2", "cartesian", "joint_state", "gripper_state", "arm_tcp_cmd"]
#     assert len(all_tstamps) == len(labels), "Number of timestamps and labels must be the same"
#     # if DEBUG: plot_timestamps(all_tstamps, labels, demo_path)  # This command throws errors about multithreading and X server, can also just cause machine to freeze. But it also sometimes works ¯\_(ツ)_/¯
#     # plot_timestamps(all_tstamps, labels, demo_path); sys.exit()
#     for counter, x in enumerate(all_tstamps):
#         min_error = float("inf")
#         # compute average error
#         min_length = min(ref_len, len(x))
#         test_shifts = [0, 1, -1, 2, -2, 3, -3, 4, -4, 5, -5, 6, -6, 7, -7, 8, -8]
#         for shift in test_shifts:

#             if shift > 0:
#                 # positive shift means the series ends before the reference series
#                 err = get_err(x[len(x) - min_length + shift: ],
#                                all_tstamps[ref][ref_len - min_length: -shift])
#                 start_idx = len(x) - min_length + shift
#                 end_idx = len(x)
#             elif shift < 0:
#                 err = get_err(x[len(x) - min_length: shift],
#                               all_tstamps[ref][ref_len - min_length - shift:])
#                 start_idx = len(x) - min_length
#                 end_idx = len(x) + shift
#             else:
#                 err = get_err(x[len(x) - min_length: ],
#                                all_tstamps[ref][ref_len - min_length:])
#                 start_idx = len(x) - min_length
#                 end_idx = len(x)

#             if err < min_error:
#                 min_error = err
#                 min_shift = shift
#         if DEBUG:
#             # print avg max and min time diff
#             print(labels[counter])
#             print("avg_time_diff:", np.mean(np.diff(x)))
#             print("max_time_diff:", np.max(np.diff(x)))
#             print("min_time_diff:", np.min(np.diff(x)))
#             print()
#         min_shifts.append(min_shift)
#         start_idcs.append(start_idx)
#         end_idcs.append(end_idx)
#         min_errors.append(min_error)
#         try:
#             assert min_error < 1 / freq * 0.75, f"Error is too large: {min_error} for {labels[counter]}"
#         except AssertionError as e:
#             print(e)
#             breakpoint()
#     for i in range(len(all_tstamps)):
#         assert start_idcs[i] >= 0
#         assert end_idcs[i] > 0
#         assert end_idcs[i] - start_idcs[i] > 0

#     # the difference between the shift and the largest positive shift (max shift) needs to be subtracted from the end idx
#     # then, simply truncate the series from the start so that they are all the same length
#     max_shift = max(min_shifts)
#     for i in range(len(all_tstamps)):
#         shift_diff = max_shift - min_shifts[i]
#         assert shift_diff >= 0
#         end_idcs[i] -= shift_diff

#     # find the shortest series
#     min_len = float('inf')
#     for i in range(len(all_tstamps)):
#         length = end_idcs[i] - start_idcs[i]
#         if length < min_len:
#             min_len = length
#             # min_len_idx = i

#     for i in range(len(all_tstamps)):
#         start_idcs[i] = end_idcs[i] - min_len

#     if DEBUG:
#         for i in range(len(all_tstamps)):
#             print(labels[i])
#             print("min shift:", min_shifts[i])
#             print("min error:", min_errors[i])
#             print("start idx:", start_idcs[i])
#             print("end idx:", end_idcs[i])
#             print("final_length:", end_idcs[i] - start_idcs[i])
#             print()

#         # do final checks on error
#         for i in range(len(all_tstamps)):
#             err = get_err(all_tstamps[i][start_idcs[i]: end_idcs[i]],
#                         all_tstamps[ref][start_idcs[ref]: end_idcs[ref]])
#             print((labels[i] + "error: ").ljust(25), err)
#     return start_idcs, end_idcs


# def get_err(a, b):
#     return np.abs(a - b).max()


# def plot_timestamps(series, labels, demo_path):
#     spacing = 0.2  # Vertical spacing between series

#     # Flatten all timestamps to find the overall x-axis range
#     all_timestamps = np.concatenate(series)
#     x_min = np.min(all_timestamps)
#     x_max = np.max(all_timestamps)
#     x_range = x_max - x_min

#     # Define the maximum x-axis length per plot
#     max_x_length = 30

#     # Calculate the number of plots needed
#     num_plots = int((x_range) / max_x_length) + 1

#     # Ensure the demo_path exists
#     os.makedirs(demo_path, exist_ok=True)

#     for plot_idx in range(num_plots):
#         # Define the x-axis limits for this plot
#         start_x = x_min + plot_idx * max_x_length
#         end_x = start_x + max_x_length

#         # Create a new figure
#         plt.figure(figsize=(250/732 * max_x_length * 15, 5))  # Adjust figsize as needed

#         # Plot each series
#         for i, timestamps in enumerate(series):
#             timestamps = np.array(timestamps)
#             # Get indices of timestamps within the current x-axis range
#             indices_in_range = np.where((timestamps >= start_x) & (timestamps <= end_x))[0]
#             timestamps_in_range = timestamps[indices_in_range]
#             y_value = i * spacing
#             if len(timestamps_in_range) > 0:
#                 plt.scatter(timestamps_in_range, [y_value]*len(timestamps_in_range), label=labels[i], s=30)
#                 # Optionally, draw lines representing events
#                 plt.vlines(timestamps_in_range, y_value - 0.1, y_value + 0.1, colors='k', linewidth=1)

#                 # Label each point with its index in the series
#                 for idx, x in zip(indices_in_range, timestamps_in_range):
#                     plt.annotate(str(idx), (x, y_value), textcoords="offset points", xytext=(0,10), ha='center')

#         plt.xlabel('Time')
#         plt.ylabel('Series')
#         plt.yticks([i*spacing for i in range(len(series))], labels)
#         plt.title(f'Timestamp Series Plot (Part {plot_idx + 1})')
#         # plt.legend()
#         plt.xlim(start_x, end_x)
#         plt.tight_layout()
#         # Save plot
#         print(f"Saving timestamps plot part {plot_idx + 1}/{num_plots}...")
#         plot_filename = os.path.join(demo_path, f"timestamps_part_{plot_idx + 1}.png")
#         plt.savefig(plot_filename)
#         plt.close()


def make_combined_frame(depth_frames, rgb_frames, cartesian_frames, joint_state_plot, i, max_depth_value, frames_dir):
    # create a new frame
    frame = np.zeros((360 * 2 + 480, 640 * 3, 3), dtype=np.uint8)

    # add depth frames. Depth frames are single channel, so need to use a colormap to convert them to rgb
    for j, x in enumerate(depth_frames):
        frame[360:720, j*640:(j+1)*640] = (plt.cm.viridis(x / max_depth_value)[:, :, :3] * 255).astype(np.uint8)
        if j == 2:
            # add a "2x" label to the bottom right corner with cv2
            cv2.putText(frame, "2x", (640*3 - 50, 720 - 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

    # add rgb frames
    for j, x in enumerate(rgb_frames):
        frame[0:360, j*640:(j+1)*640] = x[:, :, ::-1]

    # add cartesian frames
    # frame[360*2:, :640] = (cartesian_frames[:, :, :3]).astype(np.uint8)

    # add joint state_frame
    frame[360*2:, 640:] = (joint_state_plot[..., :3]).astype(np.uint8)

    # save_combined frames
    plt.imsave(f"{frames_dir}/frame_{i:03d}.png", frame)


# def make_joint_state_plots(angles, q_d, gripper_pos, gripper_cmd, idcs):
def make_joint_state_plots(angles, gripper_pos, gripper_cmd, idcs):
    # make 2 x 4 subplots for 7 joints. Figure size should have a height of 480 and width of 1280. Return fig as an np array.
    # make dir joint_state_plots
    joint_state_plots = []

    fig, axs = plt.subplots(2, 4, figsize=(1280/100, 480/100))
    vlines = []
    canvas = FigureCanvas(fig)
    for i in range(8):
        ax = axs[i // 4, i % 4]
        if i ==7:
            ax.plot(gripper_pos, antialiased=True)
            ax.plot(gripper_cmd, antialiased=True)
            ax.set_title(f"Gripper", antialiased=True)
        else:
            ax.plot(angles[:, i], antialiased=True)
            # ax.plot(q_d[:, i], antialiased=True)
            ax.set_title(f"Joint {i+1}", antialiased=True)
        ax.grid()
        # draw a vertical red line corresponding to the timestep
        vlines.append(ax.axvline(idcs[0], color='r'))
    plt.tight_layout()
    # Convert the plot to a NumPy array
    # make a super legend for the whole figure: ["actual", "commanded"]
    # fig.legend(["pos", "cmd pos"], loc='upper right')
    fig.legend(["pos"], loc='upper right')
    canvas.draw()
    image = np.frombuffer(canvas.tostring_rgb(), dtype='uint8')
    image = image.reshape(canvas.get_width_height()[::-1] + (3,))
    joint_state_plots.append(image)
    # plt.savefig(f"{demo_path}/joint_state_plots/frame_{0:03d}.png")
    # the eight plot is for gripper state
    for j in range(1, idcs.shape[0]):
        for i in range(8):
            # erase previous red line
            ax = axs[i // 4, i % 4]
            # ax.lines.pop(1)
            vlines[i].remove()
        vlines = []
        for i in range(8):
            # draw a vertical red line corresponding to the timestep
            ax = axs[i // 4, i % 4]
            vlines.append(ax.axvline(idcs[j], color='r'))
            # Draw the canvas to update the figure

        # Convert the plot to a NumPy array
        canvas.draw()
        # Convert the plot to a NumPy array using ARGB
        image = np.frombuffer(canvas.tostring_rgb(), dtype='uint8')
        image = image.reshape(canvas.get_width_height()[::-1] + (3,))
        joint_state_plots.append(image)

    plt.close(fig)

    return joint_state_plots


def load_video_to_numpy_array(video_path):
    # Open the video file
    cap = cv2.VideoCapture(video_path)

    # Initialize a list to hold the frames
    frames = []

    # Loop until the end of the video
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        # Append the frame to the list
        frames.append(frame)

    # Release the video capture object
    cap.release()

    # Convert the list of frames to a NumPy array
    frames_np = np.array(frames)

    return frames_np


def make_cartesian_frame(pos, quats):
    cartesian_frames = []
    fig = plt.figure()
    canvas = FigureCanvas(fig)
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(-0.75, 0.75)
    ax.set_ylim(-0.75, 0.75)
    ax.set_zlim(-0.75, 0.75)
    ax.set_aspect('equal')
    # tight layout
    plt.tight_layout()
    # add title
    # label axes
    ax.set_xlabel('X', antialiased=True)
    ax.set_ylabel('Y', antialiased=True)
    ax.set_zlabel('Z', antialiased=True)
    plt.title(f"cartesian Pose", antialiased=True)
    # calculate end point of cartesian. Multiple x unit vector by quaternion
    # for base_vec in [np.array([0.0, 0.0, 0.1]),np.array([0.0, 0.1, 0.0]),np.array([0.1, 0.0, 0.0])]:
    # zdiff = end[2] - pos[2]  # I want the largest negative z -diff
    # print(zdiff)

    # permute axes
    # pos = pos[[2, 0, 1]]
    # end = end[[2, 0, 1]]
    # pos = pos[[0, 1, 2]]
    # end = end[[0, 1, 2]]
    # pos = pos[[1, 2, 0]]
    # end = end[[1, 2, 0]]
    # pos *= -1
    # end *= -1

    # arrow = end - pos
    items = None
    for i in range(pos.shape[0]):
        base_vec = np.array([0.0, 0.0, -0.23])
        end = qv_mult(quats[i], base_vec)
        # rotate 90 deg about z-axis
        end = np.array([-end[1], end[0], end[2]])

        if items is not None:
            for item in items:
                item.remove()
        items = []
        items.append(ax.quiver(pos[i, 0], pos[i, 1], pos[i, 2], end[0], end[1], end[2], color='r'))
        alpha = 0.3
        for vec, color in zip([pos[i], pos[i] + end], ['g', 'r']):
            # plot the z-plane transparently
            items.append(
                ax.plot_surface(
                np.array([[-0.75, -0.75], [0.75, 0.75]]),
                np.array([[-0.75, 0.75], [-0.75, 0.75]]),
                np.array([[vec[2], vec[2]], [vec[2], vec[2]]]),
                color=color,
                alpha=alpha,
                )
            )
            # plot the x-plane transparently
            items.append(
            ax.plot_surface(
                np.array([[vec[0], vec[0]], [vec[0], vec[0]]]),
                np.array([[-0.75, 0.75], [-0.75, 0.75]]),
                np.array([[-0.75, -0.75], [0.75, 0.75]]),
                color=color,
                alpha=alpha,
                )
            )
        canvas.draw()
        image = np.frombuffer(canvas.tostring_rgb(), dtype='uint8')
        image = image.reshape(canvas.get_width_height()[::-1] + (3,))
        cartesian_frames.append(image)
    plt.close(fig)

    return cartesian_frames


def q_mult(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    return w, x, y, z


def qv_mult(q1, v1):
    # q2 = (0.0,) + v1
    q2 = np.zeros(4)
    q2[1:] = v1
    return q_mult(q_mult(q1, q2), q_conjugate(q1))[1:]


def q_conjugate(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


def make_depth_videos(demo_number):
    demo_path = f"/home/ripl/openteach/extracted_data/demonstration_{demo_number}"
    frames_dir = f"{demo_path}/frames"
    for j in [0, 1, 2]:
        with h5py.File(f"{demo_path}/cam_{j}_depth.h5", "r") as f:
            x = np.array(f['depth_images'])

        # save frames
        print("Number of frames:", x.shape[0])
        print("Shape of each frame:", x.shape[1:])
        if not os.path.exists(frames_dir):
            os.makedirs(frames_dir)
        with ThreadPoolExecutor(max_workers=8) as executor:
            for i in tqdm(range(x.shape[0]), desc="Saving frames..."):
                # plt.imshow(x[i])
                # plt.imsave(f"{frames_dir}/frame_{i:03d}.png", x[i])
                executor.submit(save_img, f"{frames_dir}/frame_{i:03d}.png", x[i])



        # compile video
        compile_video(f"cam_{j}_depth" , frames_dir, demo_path)

        # delete frames dir
        run_cmd(f"rm -r {frames_dir}")


def save_img(path, img):
    plt.imsave(path, img)


def run_cmd(command, env=None):
    print('------------------------------------------')
    print("Running command:", command)
    if env is None:
        env = {}
    completed_process = subprocess.run(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=env)
    # Print the output and error messages
    print("Standard Output:")
    print(completed_process.stdout)

    if completed_process.returncode != 0:
        print("Standard Error:")
        print(completed_process.stderr)

    # Print the return code
    print("Return Code:", completed_process.returncode)
    print('------------------------------------------')
    print()


def compile_video(vid_name, frames_dir, results_dir):
    command = f"yes | ffmpeg -framerate 10 -i {frames_dir}/frame_%03d.png -c:v libx264 -pix_fmt yuv420p {results_dir}/{vid_name}.mp4"
    run_cmd(command, env={'LD_PRELOAD': '/usr/lib/x86_64-linux-gnu/libffi.so.7'})


if __name__ == "__main__":
    main()
