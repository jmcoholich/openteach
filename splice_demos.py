"""Splice two processed OpenTeach demo H5 files into one output demo."""

import argparse
from dataclasses import dataclass
from pathlib import Path

import h5py
import numpy as np

MISSING_VALUES = {"", "none", "null", "nan"}


@dataclass(frozen=True)
class ProcessedDemo:
    name: str
    h5_path: Path


def main():
    parser = argparse.ArgumentParser(description="Concatenate two processed OpenTeach H5 demos.")
    parser.add_argument("first_demo", help="Demo name, demonstration folder, or demo_*.h5 path.")
    parser.add_argument("second_demo", help="Demo name, demonstration folder, or demo_*.h5 path.")
    parser.add_argument("output_name", help="Name for the spliced output demo.")
    parser.add_argument("--data_root", type=Path, default=Path.home() / "openteach" / "extracted_data")
    parser.add_argument("--trim_second_start_frames", type=int, default=0)
    parser.add_argument("--trim_first_end_frames", type=int, default=0)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()

    data_root = Path(args.data_root).expanduser()
    first = resolve_demo(args.first_demo, data_root)
    second = resolve_demo(args.second_demo, data_root)
    output_h5 = data_root / f"demonstration_{args.output_name}" / f"demo_{args.output_name}.h5"
    output_h5.parent.mkdir(parents=True, exist_ok=True)

    if output_h5.exists() and not args.overwrite:
        raise FileExistsError(f"{output_h5} already exists. Pass --overwrite to replace it.")

    splice_h5(first, second, output_h5, args.trim_second_start_frames, args.trim_first_end_frames)

    print(f"Saved spliced h5: {output_h5}")


def resolve_demo(demo_arg, data_root):
    path = Path(demo_arg).expanduser()

    if path.suffix == ".h5":
        h5_path = path
        name = path.stem.removeprefix("demo_")
    elif path.exists() and path.is_dir():
        name = path.name.removeprefix("demonstration_")
        h5_path = path / f"demo_{name}.h5"
    else:
        name = demo_arg.removeprefix("demonstration_")
        h5_path = data_root / f"demonstration_{name}" / f"demo_{name}.h5"

    if not h5_path.exists():
        raise FileNotFoundError(f"Missing processed h5 for {demo_arg}: {h5_path}")

    return ProcessedDemo(name=name, h5_path=h5_path)


def splice_h5(first, second, output_h5, trim_second_start_frames, trim_first_end_frames):
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
        validate_trim("--trim_first_end_frames", trim_first_end_frames, first_len, first.h5_path)
        validate_trim("--trim_second_start_frames", trim_second_start_frames, second_len, second.h5_path)

        with h5py.File(output_h5, "w") as out:
            copy_attrs(f0.attrs, out.attrs)
            copy_attrs(
                {
                    "spliced_from": f"{first.h5_path},{second.h5_path}",
                    "splice_source_names": f"{first.name},{second.name}",
                    "splice_source_lengths": np.array([first_len, second_len], dtype=np.int64),
                    "trim_first_end_frames": trim_first_end_frames,
                    "trim_second_start_frames": trim_second_start_frames,
                },
                out.attrs,
            )

            for key in f0.keys():
                d0 = f0[key]
                d1 = f1[key]
                output_tail_shape, output_dtype = dataset_output_spec(key, d0, d1)
                first_frame_count = d0.shape[0] - trim_first_end_frames

                out_dset = out.create_dataset(
                    key,
                    shape=(first_frame_count + d1.shape[0] - trim_second_start_frames, *output_tail_shape),
                    dtype=output_dtype,
                )
                copy_attrs(d0.attrs, out_dset.attrs)
                copy_frames(key, d0, out_dset, dst_start=0, src_end=first_frame_count)
                copy_frames(
                    key,
                    d1,
                    out_dset,
                    dst_start=first_frame_count,
                    src_start=trim_second_start_frames,
                )


def validate_trim(option, value, length, path):
    if value < 0:
        raise ValueError(f"{option} must be >= 0")
    if value >= length:
        raise ValueError(f"{option}={value} removes all {length} frames from {path}")


def first_dataset_length(h5_file):
    for dataset in h5_file.values():
        if not isinstance(dataset, h5py.Dataset) or dataset.ndim == 0:
            continue
        return dataset.shape[0]
    raise ValueError(f"No frame datasets found in {h5_file.filename}")


def dataset_output_spec(key, d0, d1):
    if not isinstance(d0, h5py.Dataset) or not isinstance(d1, h5py.Dataset):
        raise ValueError(f"{key} is not a plain dataset in both h5 files.")
    if d0.ndim == 0 or d1.ndim == 0:
        raise ValueError(f"{key} is scalar; expected a frame-major dataset.")

    if d0.shape[1:] == d1.shape[1:]:
        return d0.shape[1:], shared_dtype(key, d0.dtype, d1.dtype)

    if key == "cartesian_pose_cmd":
        if is_missing_cartesian_pose_cmd(d0) and d1.shape[1:] == (7,):
            return (7,), np.result_type(d1.dtype, np.float64)
        if is_missing_cartesian_pose_cmd(d1) and d0.shape[1:] == (7,):
            return (7,), np.result_type(d0.dtype, np.float64)

    raise ValueError(f"{key} shapes differ after frame axis: {d0.shape} vs {d1.shape}")


def shared_dtype(key, dtype0, dtype1):
    if dtype0 == dtype1:
        return dtype0
    if np.issubdtype(dtype0, np.number) and np.issubdtype(dtype1, np.number):
        return np.result_type(dtype0, dtype1)
    raise ValueError(f"{key} dtypes differ: {dtype0} vs {dtype1}")


def is_missing_cartesian_pose_cmd(dataset):
    if dataset.shape[1:] != ():
        return False
    if np.issubdtype(dataset.dtype, np.number):
        values = dataset[()]
        return bool(values.size == 0 or np.isnan(values).all())
    return all(normalize_value(value) in MISSING_VALUES for value in dataset[()])


def normalize_value(value):
    return (value.decode("utf-8") if isinstance(value, bytes) else str(value)).strip().lower()


def copy_attrs(src_attrs, dst_attrs):
    for key, value in src_attrs.items():
        dst_attrs[key] = value


def copy_frames(key, src, dst, dst_start, src_start=0, src_end=None, chunk_size=128):
    src_end = src.shape[0] if src_end is None else src_end
    frame_count = src_end - src_start
    if key == "cartesian_pose_cmd" and src.shape[1:] == () and dst.shape[1:] == (7,):
        dst[dst_start : dst_start + frame_count] = np.nan
        return

    for start in range(0, frame_count, chunk_size):
        end = min(start + chunk_size, frame_count)
        dst[dst_start + start : dst_start + end] = src[src_start + start : src_start + end]


if __name__ == "__main__":
    main()
