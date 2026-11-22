"""Storage and destructive-cleanup checks, using only temporary synthetic demos."""

import contextlib
import io
import pickle
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import cv2
import h5py
import numpy as np

import visualize_demo as demo


class DemoStorageTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="visualize_demo_test_")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / "demonstration_test"
        self.root.mkdir()
        self.output = self.root / "demo_test.h5"
        self.parent_output = self.root.parent / "demo_test.h5"
        self.data = {key: np.arange(21, dtype=np.float64).reshape(3, 7) for key in demo.H5_KEYS}
        self.data["rgb_frames"] = np.arange(3 * 3 * 16 * 16 * 3, dtype=np.uint8).reshape(3, 3, 16, 16, 3)
        self.data["cartesian_pose_cmd"] = np.array([])
        self.data["gripper_state"] = np.array([0.1, 0.2, 0.3])
        self.data["gripper_action"] = np.array([0, 1, 1])
        self.attrs = {"task": "synthetic", "values": np.array([1, 2]), "missing": np.nan}

    def make_sources(self, extra_video_frames=0):
        sources = demo.source_file_paths(str(self.root), "test")
        with h5py.File(sources[0], "w") as f:
            for key, values in self.data.items():
                if key not in ("rgb_frames", "cartesian_pose_cmd"):
                    f[key] = values
            f["index"] = np.arange(3)
            f["timestamp"] = [1.0, 1.05, 1.1]
            f.attrs["task"] = "synthetic"
        for camera in range(3):
            path = self.root / f"cam_{camera}_rgb_video.avi"
            writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"FFV1"), 20, (16, 16))
            self.assertTrue(writer.isOpened())
            try:
                for frame in self.data["rgb_frames"][:, camera]:
                    writer.write(frame)
                for _ in range(extra_video_frames):
                    writer.write(np.zeros((16, 16, 3), dtype=np.uint8))
            finally:
                writer.release()
            with open(self.root / f"cam_{camera}_rgb_video.metadata", "wb") as f:
                pickle.dump({"timestamps": [1000, 1050, 1100]}, f)
        return sources

    def convert(self, make_video=False, **kwargs):
        with contextlib.redirect_stdout(io.StringIO()):
            demo.make_combined_video(None, str(self.root), make_video=make_video, **kwargs)

    def assert_sources_exist(self, sources):
        self.assertTrue(all(Path(path).is_file() for path in sources))

    def test_all_compression_modes_round_trip(self):
        for compression in ("none", "lzf", "gzip"):
            with self.subTest(compression=compression):
                demo.write_processed_h5(self.output, self.data, self.attrs, compression)
                demo.validate_processed_h5(self.output, self.data, self.attrs)
                with h5py.File(self.output) as f:
                    self.assertEqual(f["rgb_frames"].compression, None if compression == "none" else compression)

    def test_default_conversion_retains_sources(self):
        sources = self.make_sources()
        self.convert()
        self.assert_sources_exist(sources)
        with h5py.File(self.output) as f:
            np.testing.assert_array_equal(f["rgb_frames"][:], self.data["rgb_frames"])

    def test_parallel_loading_matches_sequential_camera_order_and_timestamps(self):
        self.make_sources(extra_video_frames=1)
        sequential_frames, sequential_times = demo.load_rgb_cameras(str(self.root), camera_workers=1)
        parallel_frames, parallel_times = demo.load_rgb_cameras(str(self.root), camera_workers=3)
        for camera in range(3):
            np.testing.assert_array_equal(parallel_frames[camera], self.data["rgb_frames"][:, camera])
            np.testing.assert_array_equal(parallel_frames[camera], sequential_frames[camera])
            np.testing.assert_array_equal(parallel_times[camera], sequential_times[camera])

    def test_parallel_camera_failure_keeps_sources_and_creates_no_output(self):
        sources = self.make_sources()
        original = demo.load_rgb_camera

        def fail_one_camera(demo_path, camera):
            if camera == 1:
                raise OSError("camera 1 decode failed")
            return original(demo_path, camera)

        with mock.patch.object(demo, "load_rgb_camera", side_effect=fail_one_camera):
            with self.assertRaisesRegex(OSError, "camera 1 decode failed"):
                self.convert(delete_source_files=True)
        self.assert_sources_exist(sources)
        self.assertFalse(self.output.exists())
        self.assertFalse(self.parent_output.exists())

    def test_cleanup_moves_outputs_removes_folder_and_can_be_repeated(self):
        sources = self.make_sources()
        (self.root / "demo_test.mp4").write_bytes(b"existing preview")
        plots = self.root / "joint_state_plots"
        plots.mkdir()
        (plots / "joint_state_plot.png").write_bytes(b"plot")
        frames = self.root / "combined_frames"
        frames.mkdir()
        (frames / "frame_000.png").write_bytes(b"frame")
        self.output.write_bytes(b"old invalid output must not be trusted")
        self.convert(delete_source_files=True, h5_compression="gzip")
        self.assertFalse(any(Path(path).exists() for path in sources))
        self.assertFalse(self.root.exists())
        self.assertEqual((self.root.parent / "demo_test.mp4").read_bytes(), b"existing preview")
        with h5py.File(self.parent_output) as f:
            np.testing.assert_array_equal(f["rgb_frames"][:], self.data["rgb_frames"])
        self.convert(delete_source_files=True)
        self.assertTrue(self.parent_output.is_file())

    def test_cleanup_refuses_unexpected_files_before_conversion(self):
        sources = self.make_sources()
        keep = self.root / "unrelated.txt"
        keep.write_text("keep me")
        with self.assertRaisesRegex(ValueError, "Unexpected file"):
            self.convert(delete_source_files=True)
        self.assertEqual(keep.read_text(), "keep me")
        self.assert_sources_exist(sources)
        self.assertFalse(self.parent_output.exists())

    def test_cleanup_refuses_parent_output_collisions(self):
        sources = self.make_sources()
        for extension in ("h5", "mp4"):
            with self.subTest(extension=extension):
                destination = self.root.parent / f"demo_test.{extension}"
                destination.write_bytes(b"keep existing output")
                with self.assertRaises(FileExistsError):
                    self.convert(delete_source_files=True)
                self.assertEqual(destination.read_bytes(), b"keep existing output")
                self.assert_sources_exist(sources)
                destination.unlink()

    def test_cleanup_migrates_previously_cleaned_folder(self):
        demo.write_processed_h5(self.output, self.data, self.attrs)
        self.convert(delete_source_files=True)
        self.assertFalse(self.root.exists())
        demo.validate_processed_h5(self.parent_output, self.data, self.attrs)

    def test_cleanup_keeps_invalid_previously_cleaned_folder(self):
        with h5py.File(self.output, "w") as f:
            f["rgb_frames"] = self.data["rgb_frames"]
        with self.assertRaisesRegex(ValueError, "missing datasets"):
            self.convert(delete_source_files=True)
        self.assertTrue(self.output.exists())
        self.assertFalse(self.parent_output.exists())

    def test_preview_failure_keeps_sources_and_local_h5(self):
        sources = self.make_sources()
        with mock.patch.object(demo, "make_video_from_data", side_effect=RuntimeError("preview failed")):
            with self.assertRaisesRegex(RuntimeError, "preview failed"):
                self.convert(make_video=True, delete_source_files=True)
        self.assert_sources_exist(sources)
        self.assertTrue(self.output.exists())
        self.assertFalse(self.parent_output.exists())

    def test_new_preview_is_published_before_folder_removal(self):
        self.make_sources()

        def create_preview(demo_path, demo_number, output_data):
            (Path(demo_path) / f"demo_{demo_number}.mp4").write_bytes(b"new preview")

        with mock.patch.object(demo, "make_video_from_data", side_effect=create_preview):
            self.convert(make_video=True, delete_source_files=True)
        self.assertEqual((self.root.parent / "demo_test.mp4").read_bytes(), b"new preview")
        self.assertTrue(self.parent_output.exists())
        self.assertFalse(self.root.exists())

    def test_partial_publication_failure_rolls_back_links_and_keeps_sources(self):
        sources = self.make_sources()
        (self.root / "demo_test.mp4").write_bytes(b"preview")
        original_link = demo.os.link

        def fail_video_link(source, destination):
            if Path(source).suffix == ".mp4":
                raise OSError("link failed")
            original_link(source, destination)

        with mock.patch.object(demo.os, "link", side_effect=fail_video_link):
            with self.assertRaisesRegex(OSError, "link failed"):
                self.convert(delete_source_files=True)
        self.assert_sources_exist(sources)
        self.assertTrue(self.output.exists())
        self.assertFalse(self.parent_output.exists())

    def test_encoder_nonzero_exit_raises(self):
        with mock.patch.object(demo, "make_joint_state_plot", return_value=(None, [])), \
                mock.patch.object(demo, "make_combined_frame"), \
                mock.patch.object(demo, "compile_video", return_value=mock.Mock(returncode=1)):
            with self.assertRaisesRegex(RuntimeError, "encoding failed"):
                demo.make_video_from_data(str(self.root), "test", self.data)

    def test_corrupt_readback_preserves_sources_and_previous_output(self):
        sources = self.make_sources()
        self.output.write_bytes(b"previous output")
        original = demo.validate_processed_h5

        def corrupt_then_validate(path, data, attrs):
            with h5py.File(path, "r+") as f:
                f["rgb_frames"][0, 0, 0, 0, 0] = 255
            original(path, data, attrs)

        with mock.patch.object(demo, "validate_processed_h5", side_effect=corrupt_then_validate):
            with self.assertRaisesRegex(ValueError, "read-back mismatch"):
                self.convert(delete_source_files=True)
        self.assert_sources_exist(sources)
        self.assertEqual(self.output.read_bytes(), b"previous output")
        self.assertEqual(list(self.root.glob("*.tmp")), [])

    def test_publish_failure_keeps_sources(self):
        sources = self.make_sources()
        with mock.patch.object(demo.os, "replace", side_effect=OSError("disk error")):
            with self.assertRaisesRegex(OSError, "disk error"):
                self.convert(delete_source_files=True)
        self.assert_sources_exist(sources)
        self.assertFalse(self.output.exists())

    def test_source_change_during_conversion_prevents_all_deletions(self):
        sources = self.make_sources()
        original = demo.validate_processed_h5

        def validate_then_change_source(path, data, attrs):
            original(path, data, attrs)
            with open(sources[-1], "ab") as f:
                f.write(b"changed")

        with mock.patch.object(demo, "validate_processed_h5", side_effect=validate_then_change_source):
            with self.assertRaisesRegex(RuntimeError, "changed during conversion"):
                self.convert(delete_source_files=True)
        self.assert_sources_exist(sources)

    def test_truncated_video_keeps_sources(self):
        sources = self.make_sources()
        with mock.patch.object(demo, "load_video_to_numpy_array", return_value=self.data["rgb_frames"][:2, 0]):
            with self.assertRaisesRegex(ValueError, "frame count"):
                self.convert(delete_source_files=True)
        self.assert_sources_exist(sources)
        self.assertFalse(self.output.exists())

    def test_single_untimestamped_trailing_frame_is_ignored(self):
        sources = self.make_sources(extra_video_frames=1)
        with mock.patch.object(demo.warnings, "warn") as warn:
            self.convert(delete_source_files=True)
        warn.assert_not_called()
        with h5py.File(self.parent_output) as f:
            np.testing.assert_array_equal(f["rgb_frames"][:], self.data["rgb_frames"])
        self.assertFalse(any(Path(path).exists() for path in sources))

    def test_multiple_extra_video_frames_keep_sources(self):
        sources = self.make_sources(extra_video_frames=2)
        with self.assertRaisesRegex(ValueError, "frame count"):
            self.convert(delete_source_files=True)
        self.assert_sources_exist(sources)
        self.assertFalse(self.output.exists())

    def test_empty_processed_episode_is_rejected(self):
        self.data = {key: value[:0] for key, value in self.data.items()}
        with self.assertRaisesRegex(ValueError, "nonempty"):
            demo.write_processed_h5(self.output, self.data, {})
        self.assertFalse(self.output.exists())

    def test_symlink_source_is_rejected(self):
        source = self.root / "source"
        source.write_text("source")
        link = self.root / "link"
        link.symlink_to(source)
        with self.assertRaisesRegex(ValueError, "regular file"):
            demo.snapshot_sources([str(link)])
        self.assertTrue(source.is_file())

    def test_video_only_cannot_delete_sources(self):
        with mock.patch("sys.argv", ["visualize_demo.py", "--demo_number", "test", "--video-only", "--delete-source-files"]):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
                demo.main()
        self.assertEqual(error.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
