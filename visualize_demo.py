"""
Combines and cleans data from franka arm and realsense cameras.

Outputs
- A video of the demonstration including joint angle plots and rgb and depth cams
- An .h5 file containing processed data
"""

import os
import pickle as pkl
import subprocess
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor, as_completed

import cv2
import h5py
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from tqdm import tqdm

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
            if file.endswith((".pkl", ".h5")):
                continue
            # demo_number = file.split("_")[-1].split(".")[0]
            assert file.startswith("demonstration_")
            demo_number = file[14:]
            if os.path.exists(os.path.join(data_root, file, f"demo_{demo_number}.h5")):
                print(f"Demo {demo_number} already processed. Skipping...")
                continue
            make_combined_video(args.demo_folder, demo_number)

    else:
        raise ValueError("Either --demo_number or --demo_folder must be provided")


def make_combined_video(folder, demo_number):
    root_folder = f"{os.path.expanduser('~')}/openteach/extracted_data"
    if folder is None:
        demo_path = os.path.join(root_folder, f"demonstration_{demo_number}")
        cmds_path = os.path.join(root_folder, f"deoxys_obs_cmd_history_{demo_number}.h5")
    else:
        demo_path = os.path.join(root_folder, f"{folder}/demonstration_{demo_number}")
        cmds_path = os.path.join(root_folder, folder, f"deoxys_obs_cmd_history_{demo_number}.h5")
    print(demo_path)
    depth_timestamps = []
    rgb_timestamps = []

    # freq = 15.0

    print('loading observations and commands ...')
    try:
        current_commit = subprocess.check_output(
            ["git", "rev-parse", "HEAD"],
            cwd=os.path.dirname(os.path.abspath(__file__)),
            text=True,
        ).strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        current_commit = "unknown"
    with h5py.File(cmds_path, "r") as f:
        cmd_data = {}
        for key in f.keys():
            cmd_data[key] = np.array(f[key])
        # copy attributes from the h5 file
        for attr in f.attrs.keys():
            cmd_data[attr] = f.attrs[attr]
    cmd_data["openteach current commit"] = current_commit

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
        if not os.path.exists(f"{demo_path}/{fname}"):
            raise FileNotFoundError(f"File {fname} does not exist in {demo_path}.")
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
    }
    all_cams_started_time = np.max([x[0] for x in rgb_timestamps] + [x[0] for x in depth_timestamps])
    cam_stopped = np.min([x[-1] for x in rgb_timestamps] + [x[-1] for x in depth_timestamps])
    x = rgb_timestamps[0]
    dt = np.diff(x)
    print(
        f"cam_0 rgb dt stats (s): "
        f"count={dt.size}, min={dt.min():.6f}, median={np.median(dt):.6f}, "
        f"mean={dt.mean():.6f}, max={dt.max():.6f}"
    )
#    breakpoint()
    for i in tqdm(range(len(cmd_data['index'])), desc="Processing data..."):
        # once the robot is stopped (by releasing deadman switch), the robot state stops updating but the commands continue
        # detect this and skip these frames
        # print()
        # print(f"Processing frame {i}/{len(cmd_data['index'])} with timestamp {cmd_data['timestamp'][i]:.3f}...")
        if i != 0 and (cmd_data['joint_pos'][i] == cmd_data['joint_pos'][i - 1]).all():
            # print("Robot state has stopped updating. Skipping frame...")
            continue

        if cmd_data['timestamp'][i] < all_cams_started_time or cmd_data['timestamp'][i] > cam_stopped:  # throws away the last frame but thats fine
            # print(f"Frame timestamp {cmd_data['timestamp'][i]:.3f} is outside of camera recording range of {all_cams_started_time:.3f} to {cam_stopped:.3f}. Skipping frame...")
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
        output_data["rgb_frames"].append(curr_rgb_frames)
        output_data["depth_frames"].append(curr_depth_frames)

    # # Debug: save timestamp alignment plot and exit early.
    # timestamp_plot_path = os.path.join(demo_path, f"timestamp_debug_{demo_number}.png")
    # fig, ax = plt.subplots(figsize=(14, 7))
    # cmd_ts = np.asarray(cmd_data["timestamp"])
    # ax.plot(cmd_ts[:10], label="cmd_data.timestamp", linewidth=2.0, color="black")
    # for j in range(3):
    #     rgb_ts = rgb_timestamps[j][:10]
    #     depth_ts = depth_timestamps[j][:10]
    #     ax.plot(rgb_ts, label=f"cam_{j}_rgb_timestamps", linewidth=1.2, alpha=0.9, linestyle="--")
    #     ax.plot(depth_ts, label=f"cam_{j}_depth_timestamps", linewidth=1.2, alpha=0.9, linestyle=":")
    #     ax.scatter(len(rgb_ts) - 1, rgb_ts[-1], s=24, marker="o")
    #     ax.scatter(len(depth_ts) - 1, depth_ts[-1], s=28, marker="x")
    # ax.set_title(f"Timestamp Debug Plot - demo_{demo_number}")
    # ax.set_xlabel("Frame Index")
    # ax.set_ylabel("Timestamp")
    # ax.grid(True, alpha=0.3)
    # ax.legend(loc="best", fontsize=8)
    # fig.tight_layout()
    # fig.savefig(timestamp_plot_path, dpi=200)
    # plt.close(fig)
    # print(f"Saved timestamp debug plot to {timestamp_plot_path}")
    # print("Exiting early after timestamp debug plot.")
    # return

    for k, v in output_data.items():
        output_data[k] = np.array(v)
    path = f"{demo_path}/demo_{demo_number}.h5"
    print(f"Saving processed data to {path}...")
    h5_keys = [
        "rgb_frames",
        "eef_pos",
        "eef_quat",
        "arm_action",
        "gripper_action",
        "gripper_state",
        "eef_pose",
        "joint_pos",
        "cartesian_pose_cmd",
    ]
    with h5py.File(path, "w") as h5f:
        for key in h5_keys:
            h5f.create_dataset(key, data=output_data[key])
        # copy attrs from the cmd_data h5
        for attr in cmd_data.keys():
            if attr in h5_keys:
                continue
            h5f.attrs[attr] = cmd_data[attr]

    # make video
    frames_dir = f"{demo_path}/combined_frames"
    if not os.path.exists(frames_dir):
        os.makedirs(frames_dir)
    else:
        subprocess.run(f"rm -r {frames_dir}", shell=True)
        os.makedirs(frames_dir)



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


def make_combined_frame(depth_frames, rgb_frames, cartesian_frames, joint_state_plot, i, max_depth_value, frames_dir):
    # get shape of rgb frames
    h, w, _ = rgb_frames[0].shape
    # create a new frame
    frame = np.zeros((h * 2 + 480, w * 3, 3), dtype=np.uint8)

    # add depth frames. Depth frames are single channel, so need to use a colormap to convert them to rgb
    for j, x in enumerate(depth_frames):
        frame[h:h*2, j*w:(j+1)*w] = (plt.cm.viridis(x / max_depth_value)[:, :, :3] * 255).astype(np.uint8)
        if j == 2:
            # add a "2x" label to the bottom right corner with cv2
            cv2.putText(frame, "2x", (w*3 - 50, 720 - 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

    # add rgb frames
    for j, x in enumerate(rgb_frames):
        frame[0:h, j*w:(j+1)*w] = x[:, :, ::-1]

    # add cartesian frames
    # frame[h*2:, :w] = (cartesian_frames[:, :, :3]).astype(np.uint8)


    joint_state_plot = np.pad(
    joint_state_plot,
    ((0, max(0, frame[h*2:, w:].shape[0] - joint_state_plot.shape[0])),
     (0, max(0, frame[h*2:, w:].shape[1] - joint_state_plot.shape[1])),
     (0, 0)),
    mode='constant')
    # add joint state_frame
    frame[h*2:, w:] = (joint_state_plot[..., :3]).astype(np.uint8)

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
            ax.set_title("Gripper")
        else:
            ax.plot(angles[:, i], antialiased=True)
            # ax.plot(q_d[:, i], antialiased=True)
            ax.set_title(f"Joint {i+1}")
        ax.grid()
        # draw a vertical red line corresponding to the timestep
        vlines.append(ax.axvline(idcs[0], color='r'))
    plt.tight_layout()
    # Convert the plot to a NumPy array
    # make a super legend for the whole figure: ["actual", "commanded"]
    # fig.legend(["pos", "cmd pos"], loc='upper right')
    fig.legend(["pos"], loc='upper right')
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
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.title("cartesian Pose")
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
        for vec, color in zip([pos[i], pos[i] + end], ['g', 'r'], strict=True):
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
        image = np.asarray(canvas.buffer_rgba())[:, :, :3].copy()
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
    demo_path = os.path.join(os.path.expanduser("~"), f"openteach/extracted_data/demonstration_{demo_number}")
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
    command = f"yes | ffmpeg -framerate 40 -i {frames_dir}/frame_%03d.png -c:v libx264 -pix_fmt yuv420p {results_dir}/{vid_name}.mp4"
    run_cmd(command, env={'LD_PRELOAD': '/usr/lib/x86_64-linux-gnu/libffi.so.7'})


if __name__ == "__main__":
    main()
