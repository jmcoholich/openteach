#!/usr/bin/env python3
"""Capture stationary Franka/camera calibration samples without sending commands.

Run with the robot state publisher and OpenTeach cameras already streaming:
    python capture_calibration.py --output extracted_data/table_calibration
    python capture_calibration.py --output extracted_data/table_calibration --depth

Press Enter to capture, or enter a label (e.g. table_corner_left) and press Enter.
Enter q to quit. Hold the arm still until the sample has been saved.
Images/state are approximately synchronized using local reception times, not
hardware synchronized. Camera timestamps are native RealSense milliseconds;
robot time is a separate clock. EEF means libfranka's configured EE frame, not
necessarily the fingertip or table contact point.
"""

import argparse
import json
import pickle
import shutil
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np
import yaml
import zmq
from deoxys.proto.franka_interface.franka_robot_state_pb2 import FrankaRobotStateMessage
from google.protobuf.json_format import MessageToDict


def subscribe(context, endpoint, topic):
    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.LINGER, 0)
    socket.setsockopt(zmq.CONFLATE, 1)
    socket.setsockopt(zmq.SUBSCRIBE, topic)
    socket.connect(endpoint)
    return socket


def capture(streams, timeout, max_skew):
    """Discard queued messages, then collect fresh messages from every stream."""
    poller = zmq.Poller()
    for socket in streams.values():
        # CONFLATE bounds each queue to one message.
        try:
            socket.recv(zmq.NOBLOCK)
        except zmq.Again:
            pass
        poller.register(socket, zmq.POLLIN)
    started = time.time()
    deadline = time.monotonic() + timeout
    samples = {}
    while time.monotonic() < deadline:
        events = dict(poller.poll(max(1, min(100, int((deadline - time.monotonic()) * 1000)))))
        for name, socket in streams.items():
            if socket in events:
                try:
                    raw = socket.recv(zmq.NOBLOCK)
                except zmq.Again:
                    continue
                samples[name] = (raw, time.time(), time.monotonic())
        if len(samples) == len(streams):
            times = [sample[2] for sample in samples.values()]
            if time.monotonic() - min(times) <= max_skew:
                return samples, started
    stale = [name for name in streams if name not in samples or time.monotonic() - samples[name][2] > max_skew]
    raise TimeoutError('Missing/stale streams: ' + ', '.join(stale))


def unpack_camera(raw, topic):
    prefix = topic + b' '
    if not raw.startswith(prefix):
        raise ValueError(f'Expected camera topic {topic!r}')
    # Same trusted local publisher protocol as openteach.utils.network.
    return pickle.loads(raw[len(prefix):])


def write_sample(output, samples, started, label, endpoints):
    """Save a complete sample directory; never expose partially written samples."""
    state = FrankaRobotStateMessage()
    state.ParseFromString(samples['robot'][0])
    q = np.asarray(state.q)
    pose = np.asarray(state.O_T_EE)
    if q.shape != (7,) or pose.shape != (16,) or not np.isfinite(np.r_[q, pose]).all():
        raise ValueError('Robot state lacks valid q[7] or O_T_EE[16]')
    transform = pose.reshape(4, 4, order='F')
    received = [sample[1] for sample in samples.values()]
    metadata = {
        'label': label,
        'trigger_unix_s': started,
        'saved_utc': datetime.now(timezone.utc).isoformat(),
        'host_receive_unix_s': {name: sample[1] for name, sample in samples.items()},
        'host_receive_spread_s': max(s[2] for s in samples.values()) - min(s[2] for s in samples.values()),
        'capture_duration_s': max(received) - started,
        'endpoints': endpoints,
        'joint_angles_rad': q.tolist(),
        'eef_position_base_m': transform[:3, 3].tolist(),
        'eef_rotation_base': transform[:3, :3].tolist(),
        'base_T_ee': transform.tolist(),
        'robot_state': MessageToDict(state, preserving_proto_field_name=True),
        'camera_source_timestamps_ms': {},
        'notes': 'Stationary approximate snapshot. Native camera/robot clocks are separate. EE frame is not necessarily the contact point. Depth arrays, when present, retain publisher units; no depth scale is assumed.',
    }
    sample_name = 'sample_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
    temp = Path(tempfile.mkdtemp(prefix='.pending_', dir=output))
    try:
        np.savez(temp / 'robot_state.npz', joint_angles_rad=q,
                 eef_position_base_m=transform[:3, 3], base_T_ee=transform,
                 joint_velocities_rad_s=np.asarray(state.dq),
                 joint_torques_nm=np.asarray(state.tau_J))
        (temp / 'robot_state.pb').write_bytes(samples['robot'][0])
        for name, (raw, _, _) in samples.items():
            if name == 'robot':
                continue
            camera, kind = name.split('/')
            if kind == 'rgb':
                data = unpack_camera(raw, b'rgb_image')
                image = cv2.imdecode(np.frombuffer(data['rgb_image'], np.uint8), cv2.IMREAD_COLOR)
                if image is None:
                    raise ValueError(f'Invalid RGB image from {camera}')
                if not cv2.imwrite(str(temp / f'{camera}.png'), image):
                    raise OSError(f'Failed to save {camera} PNG')
                metadata['camera_source_timestamps_ms'][name] = float(data['timestamp'])
            elif kind == 'depth':
                import blosc
                data = unpack_camera(raw, b'depth_image')
                np.save(temp / f'{camera}_depth.npy', blosc.unpack_array(data['depth_image']))
                metadata['camera_source_timestamps_ms'][name] = float(data['timestamp'])
            elif kind == 'intrinsics':
                matrix = np.asarray(unpack_camera(raw, b'intrinsics'))
                np.save(temp / f'{camera}_intrinsics.npy', matrix)
        (temp / 'metadata.json').write_text(json.dumps(metadata, indent=2, allow_nan=False) + '\n')
        destination = output / sample_name
        temp.rename(destination)
    except BaseException:
        shutil.rmtree(temp)
        raise
    return destination, metadata


def main():
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--output', type=Path, default=root / 'extracted_data' / 'calibration' / datetime.now().strftime('%Y%m%d_%H%M%S'))
    parser.add_argument('--config', type=Path, default=root / 'configs' / 'deoxys.yml', help='Read NUC.IP and NUC.PUB_PORT from this file')
    parser.add_argument('--camera-host', default='172.16.0.1')
    parser.add_argument('--camera', action='append', metavar='NAME=PORT', help='RGB stream, repeatable; defaults: front=10007, side=10005, wrist=10006')
    parser.add_argument('--depth', action='store_true', help='Also require depth and intrinsics on RGB port + 1000; camera depth publishing must be enabled')
    parser.add_argument('--timeout', type=float, default=5.0, help='Seconds to wait for fresh messages per capture (default: 5)')
    parser.add_argument('--max-skew', type=float, default=0.25, help='Maximum age/spread of host reception times in seconds (default: 0.25)')
    args = parser.parse_args()
    if not (np.isfinite(args.timeout) and np.isfinite(args.max_skew) and args.timeout > 0 and args.max_skew > 0):
        parser.error('--timeout and --max-skew must be finite and positive')
    config = yaml.safe_load(args.config.read_text())
    cameras = {}
    for entry in args.camera or ['front=10007', 'side=10005', 'wrist=10006']:
        try:
            name, port = entry.split('=')
            port = int(port)
            if not name or not all(c.isascii() and (c.isalnum() or c in '_-') for c in name) or name in cameras:
                raise ValueError
            if not 1 <= port <= (64535 if args.depth else 65535):
                raise ValueError
        except ValueError:
            parser.error(f'Invalid/duplicate camera: {entry!r}; use NAME=PORT')
        cameras[name] = port
    endpoints = {'robot': f"tcp://{config['NUC']['IP']}:{config['NUC']['PUB_PORT']}"}
    topics = {'robot': b''}
    for name, port in cameras.items():
        endpoints[f'{name}/rgb'] = f'tcp://{args.camera_host}:{port}'
        topics[f'{name}/rgb'] = b'rgb_image '
        if args.depth:
            for kind, topic in [('depth', b'depth_image '), ('intrinsics', b'intrinsics ')]:
                endpoints[f'{name}/{kind}'] = f'tcp://{args.camera_host}:{port + 1000}'
                topics[f'{name}/{kind}'] = topic
    args.output.mkdir(parents=True, exist_ok=True)
    context = zmq.Context()
    streams = {}
    try:
        for name, endpoint in endpoints.items():
            streams[name] = subscribe(context, endpoint, topics[name])
        print(f'Subscribe-only calibration capture. Output: {args.output.resolve()}')
        for name, endpoint in endpoints.items():
            print(f'  {name}: {endpoint}')
        print('Hold the arm still. Enter = capture; label + Enter = labeled capture; q = quit.')
        while True:
            label = input('\nCapture> ').strip()
            if label.lower() in ('q', 'quit', 'exit'):
                break
            try:
                samples, started = capture(streams, args.timeout, args.max_skew)
                path, metadata = write_sample(args.output, samples, started, label, endpoints)
                print(f'Saved {path.name}; EEF base XYZ (m): {np.round(metadata["eef_position_base_m"], 5)}; receive spread: {metadata["host_receive_spread_s"] * 1000:.1f} ms')
            except (TimeoutError, ValueError, OSError) as error:
                print(f'Capture not saved: {error}')
    except (KeyboardInterrupt, EOFError):
        print('\nStopped.')
    finally:
        for socket in streams.values():
            socket.close()
        context.term()


if __name__ == '__main__':
    main()
