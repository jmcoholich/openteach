"""
Combines and cleans data from franka arm and realsense cameras.

Outputs
- A video of the demonstration including joint angle plots and rgb and depth cams
- An .h5 file containing processed data
"""

import os
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor, as_completed

import cv2
import h5py
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from tqdm import tqdm

from visualize_demo import run_cmd

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
    parser.add_argument("--demo_num", type=str, help="The number of the demonstration to process and visualize")
    parser.add_argument("--reversed", action="store_true", help="Whether to visualize the reversed demonstration or not.")
    parser.add_argument("--suffix",  type=str, help="Suffix to the replay files")
    args = parser.parse_args()

    make_replay_video(args)


def load_data(h5_path):
    return_data = {}
    with h5py.File(h5_path, "r") as f:
        for key in f.keys():
            return_data[key] = np.array(f[key])
        # copy attributes from the h5 file
        for attr in f.attrs.keys():
            return_data[attr] = f.attrs[attr]
    return return_data


def make_replay_video(args):
    root_folder = f"{os.path.expanduser('~')}/openteach/extracted_data"

    orig_path = os.path.join(root_folder, f"demonstration_{args.demo_num}/demo_{args.demo_num}.h5")
    suffix = f"_{args.suffix}" if args.suffix else ""
    if args.reversed:
        replay_path = os.path.join(root_folder, f"demonstration_{args.demo_num}_playback_reversed{suffix}/demo_{args.demo_num}_playback_reversed{suffix}.h5")
    else:
        replay_path = os.path.join(root_folder, f"demonstration_{args.demo_num}_playback{suffix}/demo_{args.demo_num}_playback{suffix}.h5")

    orig_data = load_data(orig_path)
    replay_data = load_data(replay_path)

    replay_num_frames = replay_data["arm_action"].shape[0]
    orig_num_frames = orig_data["arm_action"].shape[0]
    num_frames = min(replay_num_frames, orig_num_frames)
    if replay_num_frames != orig_num_frames:
        print(
            f"\nWarning: replay has {replay_num_frames} timesteps, original has {orig_num_frames}. "
            f"Using {num_frames} frames."
        )
    truncate_all(orig_data, num_frames, reverse=args.reversed)
    truncate_all(replay_data, num_frames)

    # clear and recreate frames dir
    if args.reversed:
        frames_dir = os.path.join(root_folder, f"demonstration_{args.demo_num}_playback_reversed_{suffix}/playback_comparision_frames")
    else:
        frames_dir = os.path.join(root_folder, f"demonstration_{args.demo_num}_playback{suffix}/playback_comparision_frames")
    if not os.path.exists(frames_dir):
        os.makedirs(frames_dir)
    else:
        run_cmd(f"rm -r {frames_dir}")
        os.makedirs(frames_dir)

    workers = 8
    # print("\nGenerating joint state plots and cartesian plots...\n")
    print("\nGenerating joint state plots...\n")
    # split the range up into equal parts equal to the number of workers
    with ProcessPoolExecutor(max_workers=workers) as executor:
        futures = []
        chunk_size = num_frames // (workers - 1)
        remainder = num_frames % (workers - 1)
        for i in range(workers - 1):
            start = i * chunk_size
            end = (i + 1) * chunk_size
            futures.append(executor.submit(
                make_replay_joint_plots,
                orig_data["joint_pos"],
                orig_data["gripper_state"],
                replay_data["joint_pos"],
                replay_data["gripper_state"],
                np.arange(start, end)
                ))
        if remainder > 0:
            futures.append(executor.submit(
                make_replay_joint_plots,
                orig_data["joint_pos"],
                orig_data["gripper_state"],
                replay_data["joint_pos"],
                replay_data["gripper_state"],
                np.arange(end, num_frames)
                ))

        joint_state_plots = []
        for future in futures: # needs to be in order
            joint_state_plots.extend(future.result())


    with ThreadPoolExecutor() as executor:
        futures = []
        progress_bar = tqdm(total=num_frames, desc="Saving combined frames...")
        for i in range(num_frames):

            futures.append(executor.submit(
                make_replay_vis_frame,
                [orig_data["rgb_frames"][i, 0], orig_data["rgb_frames"][i, 1], orig_data["rgb_frames"][i, 2]],
                [replay_data["rgb_frames"][i, 0], replay_data["rgb_frames"][i, 1], replay_data["rgb_frames"][i, 2]],
                joint_state_plots[i],
                i,
                frames_dir,
                args.reversed,
                ))

        for future in as_completed(futures):
            future.result()
            progress_bar.update(1)

    # compile video
    if args.reversed:
        vid_name = f"demo_{args.demo_num}_replay_comparison_reversed{suffix}"
        dirname = f"demonstration_{args.demo_num}_playback_reversed{suffix}"
    else:
        vid_name = f"demo_{args.demo_num}_replay_comparison{suffix}"
        dirname = f"demonstration_{args.demo_num}_playback{suffix}"
    compile_video(vid_name, frames_dir, os.path.join(root_folder, dirname))

def truncate_all(data_dict, num_frames, reverse=False):
    data_keys = [
        'arm_action', 'eef_pos', 'eef_pose', 'eef_quat', 'gripper_action', 'gripper_state', 'joint_pos', 'rgb_frames', 'cartesian_pose_cmd', 'index', 'timestamp'
    ]
    for key in data_dict.keys():
        if key not in data_keys:
            continue
        data_dict[key] = data_dict[key][:num_frames]
        if reverse:
            data_dict[key] = data_dict[key][::-1]


def make_replay_vis_frame(rgb_frames, replay_rgb_frames, joint_state_plot, i, frames_dir, is_reversed=False):
    # get shape of rgb frames
    h, w, _ = rgb_frames[0].shape
    # create a new frame
    frame = np.zeros((h * 2 + 480, w * 3, 3), dtype=np.uint8)

    # add replay rgb frames
    for j, x in enumerate(replay_rgb_frames):
        frame[h:h*2, j*w:(j+1)*w] = x[:, :, ::-1]
        if j == 2:
            # add a "2x" label to the bottom right corner with cv2
            cv2.putText(frame, "2x", (w*3 - 50, 720 - 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

    # add rgb frames
    for j, x in enumerate(rgb_frames):
        frame[0:h, j*w:(j+1)*w] = x[:, :, ::-1]

    # add row labels (once per row)
    cv2.putText(frame, "original", (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(frame, "replay", (12, h + 32), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

    if is_reversed and ((i // 10) % 2 == 0):
        cv2.putText(frame, "Video Reversed", (190, 32), cv2.FONT_HERSHEY_SIMPLEX, 1, (60, 60, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, "Actions Reversed", (170, h + 32), cv2.FONT_HERSHEY_SIMPLEX, 1, (60, 60, 255), 2, cv2.LINE_AA)

    joint_state_plot = np.pad(
    joint_state_plot,
    ((0, max(0, frame[h*2:, w:].shape[0] - joint_state_plot.shape[0])),
     (0, max(0, frame[h*2:, w:].shape[1] - joint_state_plot.shape[1])),
     (0, 0)),
    mode='constant')
    # add joint state_frame
    frame[h*2:, w:] = (joint_state_plot[..., :3]).astype(np.uint8)

    overlay_h, overlay_w = frame[h*2:, :w].shape[:2]
    orig_cam2 = rgb_frames[2][:, :, ::-1]
    replay_cam2 = replay_rgb_frames[2][:, :, ::-1]
    if orig_cam2.shape[0] != overlay_h or orig_cam2.shape[1] != overlay_w:
        orig_cam2 = cv2.resize(orig_cam2, (overlay_w, overlay_h), interpolation=cv2.INTER_LINEAR)
        replay_cam2 = cv2.resize(replay_cam2, (overlay_w, overlay_h), interpolation=cv2.INTER_LINEAR)
    frame[h*2:, :w] = cv2.addWeighted(orig_cam2, 0.5, replay_cam2, 0.5, 0)
    cv2.putText(frame, "overlay", (12, h * 2 + 32), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

    # save_combined frames
    plt.imsave(f"{frames_dir}/frame_{i:03d}.png", frame)


# def make_joint_state_plots(angles, q_d, gripper_pos, gripper_cmd, idcs):
def make_replay_joint_plots(angles, gripper_pos, replay_angles, replay_gripper_pos, idcs):
    # make 2 x 4 subplots for 7 joints. Figure size should have a height of 480 and width of 1280. Return fig as an np array.
    # make dir joint_state_plots
    joint_state_plots = []

    fig, axs = plt.subplots(2, 4, figsize=(1280/100, 480/100))
    vlines = []
    canvas = FigureCanvas(fig)
    angle_error = replay_angles - angles
    gripper_error = replay_gripper_pos - gripper_pos
    for i in range(8):
        ax = axs[i // 4, i % 4]
        if i ==7:
            ax.plot(gripper_pos, antialiased=True)
            ax.plot(replay_gripper_pos, antialiased=True)
            ax.plot(gripper_error, antialiased=True)
            ax.set_title("Gripper")
        else:
            ax.plot(angles[:, i], antialiased=True)
            ax.plot(replay_angles[:, i], antialiased=True)
            ax.plot(angle_error[:, i], antialiased=True)
            ax.set_title(f"Joint {i+1}")
        ax.grid()
        # draw a vertical red line corresponding to the timestep
        vlines.append(ax.axvline(idcs[0], color='r'))
    plt.tight_layout()
    # Convert the plot to a NumPy array
    # make a super legend for the whole figure: ["actual", "commanded"]
    # fig.legend(["pos", "cmd pos"], loc='upper right')
    fig.legend(["angle", "replay", "error"], loc='upper right')
    canvas.draw()
    image = np.asarray(canvas.buffer_rgba())[:, :, :3].copy()
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
        image = np.asarray(canvas.buffer_rgba())[:, :, :3].copy()
        joint_state_plots.append(image)

    plt.close(fig)

    return joint_state_plots


def compile_video(vid_name, frames_dir, results_dir):
    command = f"yes | ffmpeg -framerate 40 -i {frames_dir}/frame_%03d.png -c:v libx264 -pix_fmt yuv420p {results_dir}/{vid_name}.mp4"
    run_cmd(command, env={'LD_PRELOAD': '/usr/lib/x86_64-linux-gnu/libffi.so.7'})


if __name__ == "__main__":
    main()
