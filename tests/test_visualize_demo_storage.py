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
        self.data = {key: np.arange(21, dtype=np.float64).reshape(3, 7) for key in demo.H5_KEYS}
        self.data["rgb_frames"] = np.arange(3 * 3 * 16 * 16 * 3, dtype=np.uint8).reshape(3, 3, 16, 16, 3)
        self.data["cartesian_pose_cmd"] = np.array([])
        self.data["gripper_state"] = np.array([0.1, 0.2, 0.3])
        self.data["gripper_action"] = np.array([0, 1, 1])
        self.attrs = {"task": "synthetic", "values": np.array([1, 2]), "missing": np.nan}

    def make_sources(self):
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
            finally:
                writer.release()
            with open(self.root / f"cam_{camera}_rgb_video.metadata", "wb") as f:
                pickle.dump({"timestamps": [1000, 1050, 1100]}, f)
        return sources

    def convert(self, **kwargs):
        with contextlib.redirect_stdout(io.StringIO()):
            demo.make_combined_video(None, str(self.root), make_video=False, **kwargs)

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

    def test_cleanup_deletes_only_consumed_sources_and_can_be_repeated(self):
        sources = self.make_sources()
        keep = self.root / "unrelated.txt"
        keep.write_text("keep me")
        self.output.write_bytes(b"old invalid output must not be trusted")
        self.convert(delete_source_files=True, h5_compression="gzip")
        self.assertFalse(any(Path(path).exists() for path in sources))
        self.assertEqual(keep.read_text(), "keep me")
        with h5py.File(self.output) as f:
            np.testing.assert_array_equal(f["rgb_frames"][:], self.data["rgb_frames"])
        self.convert(delete_source_files=True)
        self.assertTrue(self.output.is_file())

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
