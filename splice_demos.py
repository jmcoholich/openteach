"""
Splice two processed OpenTeach demos into one processed demo folder.

Inputs should already have been post-processed with visualize_demo.py, so each
demo has:
  extracted_data/demonstration_<name>/demo_<name>.h5
  extracted_data/demonstration_<name>/demo_<name>.mp4

Example:
  python splice_demos.py policy_prefix human_finish plug_dagger_01

This writes:
  ~/openteach/extracted_data/demonstration_plug_dagger_01/demo_plug_dagger_01.h5
  ~/openteach/extracted_data/demonstration_plug_dagger_01/demo_plug_dagger_01.mp4
"""

import argparse
import os
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path

import h5py
import numpy as np


@dataclass(frozen=True)
class ProcessedDemo:
    name: str
    folder: Path
    h5_path: Path
    video_path: Path


def main():
    parser = argparse.ArgumentParser(
        description="Concatenate two processed OpenTeach demos into one h5 and one mp4."
    )
    parser.add_argument("first_demo", help="Demo name, demonstration folder, or demo_*.h5 path.")
    parser.add_argument("second_demo", help="Demo name, demonstration folder, or demo_*.h5 path.")
    parser.add_argument("output_name", help="Name for the spliced output demo.")
    parser.add_argument(
        "--data_root",
        default=os.path.join(os.path.expanduser("~"), "openteach", "extracted_data"),
        help="Folder containing demonstration_<name> folders.",
    )
    parser.add_argument(
        "--trim_second_start_frames",
        type=int,
        default=0,
        help="Drop this many frames from the start of the second processed h5.",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite output h5/mp4 if they already exist.",
    )
    args = parser.parse_args()

    data_root = Path(args.data_root).expanduser()
    first = resolve_demo(args.first_demo, data_root)
    second = resolve_demo(args.second_demo, data_root)

    output_folder = data_root / f"demonstration_{args.output_name}"
    output_h5 = output_folder / f"demo_{args.output_name}.h5"
    output_video = output_folder / f"demo_{args.output_name}.mp4"
    output_folder.mkdir(parents=True, exist_ok=True)

    if not args.overwrite:
        for path in [output_h5, output_video]:
            if path.exists():
                raise FileExistsError(f"{path} already exists. Pass --overwrite to replace it.")

    splice_h5(first, second, output_h5, args.trim_second_start_frames)
    splice_videos(first.video_path, second.video_path, output_video)

    print(f"Saved spliced h5: {output_h5}")
    print(f"Saved spliced video: {output_video}")


def resolve_demo(demo_arg, data_root):
    path = Path(demo_arg).expanduser()

    if path.suffix == ".h5":
        h5_path = path
        folder = h5_path.parent
        name = name_from_processed_h5(h5_path)
        video_path = folder / f"demo_{name}.mp4"
    elif path.exists() and path.is_dir():
        folder = path
        name = name_from_demo_folder(folder)
        h5_path = folder / f"demo_{name}.h5"
        video_path = folder / f"demo_{name}.mp4"
    else:
        name = demo_arg
        if name.startswith("demonstration_"):
            name = name[len("demonstration_") :]
        folder = data_root / f"demonstration_{name}"
        h5_path = folder / f"demo_{name}.h5"
        video_path = folder / f"demo_{name}.mp4"

    if not h5_path.exists():
        raise FileNotFoundError(f"Missing processed h5 for {demo_arg}: {h5_path}")
    if not video_path.exists():
        raise FileNotFoundError(f"Missing processed video for {demo_arg}: {video_path}")

    return ProcessedDemo(name=name, folder=folder, h5_path=h5_path, video_path=video_path)


def name_from_processed_h5(path):
    stem = path.stem
    return stem[len("demo_") :] if stem.startswith("demo_") else stem


def name_from_demo_folder(path):
    name = path.name
    return name[len("demonstration_") :] if name.startswith("demonstration_") else name


def splice_h5(first, second, output_h5, trim_second_start_frames):
    if trim_second_start_frames < 0:
        raise ValueError("--trim_second_start_frames must be >= 0")

    with h5py.File(first.h5_path, "r") as f0, h5py.File(second.h5_path, "r") as f1:
        first_keys = set(f0.keys())
        second_keys = set(f1.keys())
        if first_keys != second_keys:
            raise ValueError(
                "Input h5 files do not have matching datasets:\n"
                f"  only in first: {sorted(first_keys - second_keys)}\n"
                f"  only in second: {sorted(second_keys - first_keys)}"
            )

        first_len = first_dataset_length(f0)
        second_len = first_dataset_length(f1)
        if trim_second_start_frames >= second_len:
            raise ValueError(
                f"--trim_second_start_frames={trim_second_start_frames} removes all "
                f"{second_len} frames from {second.h5_path}"
            )

        with h5py.File(output_h5, "w") as out:
            copy_attrs(f0.attrs, out.attrs)
            out.attrs["spliced_from"] = f"{first.h5_path},{second.h5_path}"
            out.attrs["splice_source_names"] = f"{first.name},{second.name}"
            out.attrs["splice_source_lengths"] = np.array([first_len, second_len], dtype=np.int64)
            out.attrs["trim_second_start_frames"] = trim_second_start_frames

            for key in f0.keys():
                d0 = f0[key]
                d1 = f1[key]
                validate_dataset_pair(key, d0, d1)

                total_len = d0.shape[0] + d1.shape[0] - trim_second_start_frames
                out_dset = out.create_dataset(
                    key,
                    shape=(total_len, *d0.shape[1:]),
                    dtype=d0.dtype,
                )
                copy_attrs(d0.attrs, out_dset.attrs)
                copy_frames(d0, out_dset, dst_start=0)
                copy_frames(d1, out_dset, dst_start=d0.shape[0], src_start=trim_second_start_frames)


def first_dataset_length(h5_file):
    for key in h5_file.keys():
        dataset = h5_file[key]
        if not isinstance(dataset, h5py.Dataset) or dataset.ndim == 0:
            continue
        return dataset.shape[0]
    raise ValueError(f"No frame datasets found in {h5_file.filename}")


def validate_dataset_pair(key, d0, d1):
    if not isinstance(d0, h5py.Dataset) or not isinstance(d1, h5py.Dataset):
        raise ValueError(f"{key} is not a plain dataset in both h5 files.")
    if d0.ndim == 0 or d1.ndim == 0:
        raise ValueError(f"{key} is scalar; expected a frame-major dataset.")
    if d0.shape[1:] != d1.shape[1:]:
        raise ValueError(f"{key} shapes differ after frame axis: {d0.shape} vs {d1.shape}")
    if d0.dtype != d1.dtype:
        raise ValueError(f"{key} dtypes differ: {d0.dtype} vs {d1.dtype}")


def copy_attrs(src_attrs, dst_attrs):
    for key, value in src_attrs.items():
        dst_attrs[key] = value


def copy_frames(src, dst, dst_start, src_start=0, chunk_size=128):
    frame_count = src.shape[0] - src_start
    for start in range(0, frame_count, chunk_size):
        end = min(start + chunk_size, frame_count)
        dst[dst_start + start : dst_start + end] = src[src_start + start : src_start + end]


def splice_videos(first_video, second_video, output_video):
    with tempfile.NamedTemporaryFile("w", suffix=".txt", delete=False) as f:
        list_path = Path(f.name)
        f.write(f"file '{escape_ffmpeg_concat_path(first_video)}'\n")
        f.write(f"file '{escape_ffmpeg_concat_path(second_video)}'\n")

    try:
        copy_cmd = [
            "ffmpeg",
            "-y",
            "-f",
            "concat",
            "-safe",
            "0",
            "-i",
            str(list_path),
            "-c",
            "copy",
            str(output_video),
        ]
        result = subprocess.run(copy_cmd, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if result.returncode == 0:
            return

        reencode_cmd = [
            "ffmpeg",
            "-y",
            "-f",
            "concat",
            "-safe",
            "0",
            "-i",
            str(list_path),
            "-c:v",
            "libx264",
            "-crf",
            "18",
            "-preset",
            "slow",
            "-pix_fmt",
            "yuv420p",
            str(output_video),
        ]
        subprocess.run(reencode_cmd, check=True)
    finally:
        list_path.unlink(missing_ok=True)


def escape_ffmpeg_concat_path(path):
    return str(Path(path).resolve()).replace("'", "'\\''")


if __name__ == "__main__":
    main()
