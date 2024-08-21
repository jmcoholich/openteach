"""Make video from depth images"""

import h5py
import numpy as np
import matplotlib.pyplot as plt
import subprocess
from tqdm import tqdm
import os
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor, as_completed
import cv2
import pickle as pkl

from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
# Set global matplotlib settings for better quality
plt.rcParams['text.antialiased'] = True
plt.rcParams['lines.antialiased'] = True
plt.rcParams['patch.antialiased'] = True

import argparse
import warnings
warnings.filterwarnings( "ignore")


# demo numer is the first argument
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("demo_number", type=int, help="The number of the demonstration to visualize")
    args = parser.parse_args()
    # make_depth_videos(demo_number)
    gripper_timestamps = plot_gripper_pose(args.demo_number)
    make_combined_video(args.demo_number, gripper_timestamps)


def make_combined_video(demo_number, gripper_timestamps):
    demo_path = f"/home/ripl/openteach/extracted_data/demonstration_{demo_number}"
    depth_timestamps = []
    rgb_timestamps = []

    print("Loading joint states...")
    with h5py.File(f"{demo_path}/franka_joint_states.h5", "r") as f:
        angles = np.array(f["positions"])
        for key in f.keys():
            if key in ["orientations", "positions", "timestamps"]:
                continue
            print(key.ljust(25), f[key][()])
        print()
        joint_state_timestamps = np.array(f["timestamps"])

    # depth frames
    depth_frames = []
    for j in [0, 1, 2]:
        print(f"Loading depth images from cam_{j}...")
        with h5py.File(f"{demo_path}/cam_{j}_depth.h5", "r") as f:
            x = np.array(f['depth_images'])
            for key in f.keys():
                if key in ["orientations", "positions", "timestamps", "depth_images"]:
                    continue
                print(key.ljust(25), f[key][()])
            print()
            depth_timestamps.append(np.array(f["timestamps"]) / 1000)
        depth_frames.append(x)

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
                print(key.ljust(25), metadata[key])
            print()
        rgb_timestamps.append(np.array(metadata["timestamps"]) / 1000)

    # max_depth_value = max([np.max(x) for x in depth_frames]) * 0.5
    max_depth_value = np.percentile(np.concatenate([x.flatten() for x in depth_frames]), 98)  # get rid of outliers

    # gripper frames (just load from gripper_frames dir)
    gripper_frames = []
    print("Loading gripper frames...")
    # see how many gripper frames there are:
    num_gripper_frames = len(os.listdir(f"{demo_path}/gripper_frames"))
    for i in range(num_gripper_frames):
        gripper_frames.append(plt.imread(f"{demo_path}/gripper_frames/frame_{i:03d}.png"))


    # Print number of frames for each component. I want all the numbers to start at the same place when printed though
    print("\nNumber of frames for each component:")
    just_val = 17
    for i in range(3):
        print(f"rgb cam_{i}: ".ljust(just_val) , f"{rgb_frames[i].shape[0]}")
        print(f"depth cam_{i}:".ljust(just_val) , f"{depth_frames[i].shape[0]}")
    print("gripper:".ljust(just_val), num_gripper_frames)
    print( "joint positions:".ljust(just_val), angles.shape[0], '\n')

    # breakpoint()
    all_timestamps = rgb_timestamps + depth_timestamps + [gripper_timestamps, joint_state_timestamps]
    start_idcs, end_idcs = tstamp_syncing(all_timestamps)
    # print(max([x[0] for x in all_timestamps]) - min([x[0] for x in all_timestamps]))
    # print(max([x[-1] for x in all_timestamps]) - min([x[-1] for x in all_timestamps]))

    futures = []
    workers = 8
    num_frames = end_idcs[0] - start_idcs[0]
    print("Generating joint state plots...")
    # split the range up into equal parts equal to the number of workers
    with ProcessPoolExecutor(max_workers=workers) as executor:
        chunk_size = num_frames // (workers - 1)
        for i in range(workers - 1):
            start = i * chunk_size
            end = (i + 1) * chunk_size
            futures.append(executor.submit(
                make_joint_state_plots,
                angles,
                np.arange(start, end)
                ))
        # add the remainder
        futures.append(executor.submit(
            make_joint_state_plots,
            angles,
            np.arange(end, num_frames)
            ))

        joint_state_plots = []
        for future in futures: # needs to be in order
            joint_state_plots.extend(future.result())


    idcs = []
    for i in range(8):
        idcs.append(range(start_idcs[i], end_idcs[i]))

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
                [depth_frames[0][idcs[3][i]], depth_frames[1][idcs[4][i]], depth_frames[2][idcs[5][i]]],
                [rgb_frames[0][idcs[0][i]], rgb_frames[1][idcs[1][i]], rgb_frames[2][idcs[2][i]]],
                gripper_frames[idcs[6][i]],
                joint_state_plots[i],
                i,
                max_depth_value,
                frames_dir,
                ))

        for future in as_completed(futures):
            future.result()
            progress_bar.update(1)


    # compile video
    compile_video("combined", frames_dir, demo_path)


def tstamp_syncing(all_tstamps):
    """This is some spaghetti but it works. Likely there is a clever or
    established way to do this.
    """

    # ref idx should be shortest series
    lens = [len(x) for x in all_tstamps]
    ref = lens.index(min(lens))
    ref_len = len(all_tstamps[ref])
    min_shifts = []
    start_idcs = []
    end_idcs = []
    min_errors = []
    labels = ["rgb0", "rgb1", "rgb2", "depth0", "depth1", "depth2", "gripper", "joint_state"]
    for x in all_tstamps:
        min_error = float("inf")
        # compute average error
        min_length = min(ref_len, len(x))
        test_shifts = [0, 1, -1, 2, -2, 3, -3, 4, -4, 5, -5, 6, -6, 7, -7, 8, -8]
        for shift in test_shifts:

            if shift > 0:
                # positive shift means the series ends before the reference series
                err = get_err(x[len(x) - min_length + shift: ],
                               all_tstamps[ref][ref_len - min_length: -shift])
                start_idx = len(x) - min_length + shift
                end_idx = len(x)
            elif shift < 0:
                err = get_err(x[len(x) - min_length: shift],
                              all_tstamps[ref][ref_len - min_length - shift:])
                start_idx = len(x) - min_length
                end_idx = len(x) + shift
            else:
                err = get_err(x[len(x) - min_length: ],
                               all_tstamps[ref][ref_len - min_length:])
                start_idx = len(x) - min_length
                end_idx = len(x)

            # print(shift, err)
            if err < min_error:
                min_error = err
                min_shift = shift

        min_shifts.append(min_shift)
        start_idcs.append(start_idx)
        end_idcs.append(end_idx)
        min_errors.append(min_error)

    for i in range(8):
        assert start_idcs[i] >= 0
        assert end_idcs[i] > 0
        assert end_idcs[i] - start_idcs[i] > 0

    # the difference between the shift and the largest positive shift (max shift) needs to be subtracted from the end idx
    # then, simply truncate the series from the start so that they are all the same length
    max_shift = max(min_shifts)
    for i in range(8):
        shift_diff = max_shift - min_shifts[i]
        assert shift_diff >= 0
        end_idcs[i] -= shift_diff

    # find the shortest series
    min_len = float('inf')
    for i in range(8):
        length = end_idcs[i] - start_idcs[i]
        if length < min_len:
            min_len = length
            # min_len_idx = i

    for i in range(8):
        start_idcs[i] = end_idcs[i] - min_len

    for i in range(8):
        print(labels[i])
        print("min shift:", min_shifts[i])
        print("min error:", min_errors[i])
        print("start idx:", start_idcs[i])
        print("end idx:", end_idcs[i])
        print("final_length:", end_idcs[i] - start_idcs[i])
        print()

    # do final checks on error
    for i in range(8):
        err = get_err(all_tstamps[i][start_idcs[i]: end_idcs[i]],
                      all_tstamps[ref][start_idcs[ref]: end_idcs[ref]])
        print((labels[i] + "error: ").ljust(25), err)
    return start_idcs, end_idcs

def get_err(a, b):
    return np.abs(a - b).mean()

def make_combined_frame(depth_frames, rgb_frames, gripper_frames, joint_state_plot, i, max_depth_value, frames_dir):
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

    # add gripper frames
    frame[360*2:, :640] = (gripper_frames[:, :, :3] * 255).astype(np.uint8)

    # add joint state_frame
    frame[360*2:, 640:] = (joint_state_plot[..., :3]).astype(np.uint8)

    # save_combined frames
    plt.imsave(f"{frames_dir}/frame_{i:03d}.png", frame)


def make_joint_state_plots(angles, idcs):
    # make 2 x 4 subplots for 7 joints. Figure size should have a height of 480 and width of 1280. Return fig as an np array.
    # make dir joint_state_plots
    joint_state_plots = []

    fig, axs = plt.subplots(2, 4, figsize=(1280/100, 480/100))
    vlines = []
    canvas = FigureCanvas(fig)
    for i in range(7):
        ax = axs[i // 4, i % 4]
        ax.plot(angles[:, i], antialiased=True)
        ax.grid()
        ax.set_title(f"Joint {i+1}", antialiased=True)
        # draw a vertical red line corresponding to the timestep
        vlines.append(ax.axvline(idcs[0], color='r'))
    plt.tight_layout()
    # Convert the plot to a NumPy array
    canvas.draw()
    image = np.frombuffer(canvas.tostring_rgb(), dtype='uint8')
    image = image.reshape(canvas.get_width_height()[::-1] + (3,))
    joint_state_plots.append(image)
    # plt.savefig(f"{demo_path}/joint_state_plots/frame_{0:03d}.png")
    for j in range(1, idcs.shape[0]):
        for i in range(7):
            # erase previous red line
            ax = axs[i // 4, i % 4]
            # ax.lines.pop(1)
            vlines[i].remove()
        vlines = []
        for i in range(7):
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

# def save_joint_state_plot(path):
#     plt.savefig(path)

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


def plot_gripper_pose(demo_number):
    demo_path = f"/home/ripl/openteach/extracted_data/demonstration_{demo_number}"
    fname = "franka_cartesian_states.h5"
    with h5py.File(f"{demo_path}/{fname}", "r") as f:
        quats = np.array(f["orientations"])
        pos = np.array(f["positions"])
        for key in f.keys():
            if key in ["orientations", "positions", "timestamps"]:
                continue
            print(key.ljust(25), f[key][()])
        print()
        timestamps = np.array(f["timestamps"])

    # save frames that show position and orientation of gripper
    frames_dir = f"{demo_path}/gripper_frames"
    if not os.path.exists(frames_dir):
        os.makedirs(frames_dir)
    futures = []
    progress_bar = tqdm(total=quats.shape[0], desc="Saving gripper frames...")
    with ProcessPoolExecutor(max_workers=8) as executor:
    # with ThreadPoolExecutor(max_workers=1) as executor:
        for i in range(quats.shape[0]):
            futures.append(executor.submit(save_gripper_frame, frames_dir, pos[i], quats[i], i))


        for future in as_completed(futures):
            future.result()
            progress_bar.update(1)


    # compile video
    compile_video("gripper_pose", frames_dir, demo_path)
    return timestamps


def save_gripper_frame(frames_dir, pos, quats, i):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    # calculate end point of gripper. Multiple x unit vector by quaternion
    # for base_vec in [np.array([0.0, 0.0, 0.1]),np.array([0.0, 0.1, 0.0]),np.array([0.1, 0.0, 0.0])]:
    base_vec = np.array([0.0, 0.0, -0.23])
    end = qv_mult(quats, base_vec)
    # rotate 90 deg about z-axis
    end = np.array([-end[1], end[0], end[2]])
    # zdiff = end[2] - pos[2]  # I want the largest negative z -diff
    # print(zdiff)
    # breakpoint()

    # breakpoint()

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
    ax.quiver(pos[0], pos[1], pos[2], end[0], end[1], end[2], color='r')
    alpha = 0.3
    for vec, color in zip([pos, pos + end], ['g', 'r']):
        # plot the z-plane transparently
        ax.plot_surface(
            np.array([[-0.75, -0.75], [0.75, 0.75]]),
            np.array([[-0.75, 0.75], [-0.75, 0.75]]),
            np.array([[vec[2], vec[2]], [vec[2], vec[2]]]),
            color=color,
            alpha=alpha,
            )
        # plot the x-plane transparently
        ax.plot_surface(
            np.array([[vec[0], vec[0]], [vec[0], vec[0]]]),
            np.array([[-0.75, 0.75], [-0.75, 0.75]]),
            np.array([[-0.75, -0.75], [0.75, 0.75]]),
            color=color,
            alpha=alpha,
            )
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
    plt.title(f"Gripper Pose")
    plt.savefig(f"{frames_dir}/frame_{i:03d}.png")
    plt.close(fig)


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
    command = f"yes | ffmpeg -framerate 30 -i {frames_dir}/frame_%03d.png -c:v libx264 -pix_fmt yuv420p {results_dir}/{vid_name}.mp4"
    run_cmd(command, env={'LD_PRELOAD': '/usr/lib/x86_64-linux-gnu/libffi.so.7'})


if __name__ == "__main__":
    main()
