# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

"""Extended 2C POC: independent Color 0 alongside stereo Color 1/2."""
import threading
import time

import pytest
import pyrealsense2 as rs

pytestmark = [pytest.mark.device_each("Dual RGB")]


@pytest.fixture
def concurrent_device(test_device):
    dev, _ = test_device
    if not any(s.is_color_sensor() for s in dev.query_sensors()):
        pytest.skip("Firmware does not advertise the third RGB function")
    return dev


def test_independent_profile_resolution(concurrent_device, test_context):
    cfg = rs.config()
    cfg.enable_device(concurrent_device.get_info(rs.camera_info.serial_number))
    cfg.enable_stream(rs.stream.color, 0, 1280, 960, rs.format.yuyv, 60)
    cfg.enable_stream(rs.stream.color, 1, 1280, 720, rs.format.yuyv, 30)
    cfg.enable_stream(rs.stream.color, 2, 1280, 720, rs.format.yuyv, 30)
    pipeline = rs.pipeline(test_context)
    resolved = cfg.resolve(rs.pipeline_wrapper(pipeline))
    colors = {p.stream_index(): p for p in resolved.get_streams() if p.stream_type() == rs.stream.color}
    assert set(colors) == {0, 1, 2}
    assert colors[0].fps() == 60
    assert colors[1].fps() == colors[2].fps() == 30


def test_three_native_rgb_streams(concurrent_device):
    sensors = [concurrent_device.first_depth_sensor(), concurrent_device.first_color_sensor()]
    requests = []
    for sensor, indices, fps in [(sensors[0], (1, 2), 30), (sensors[1], (0,), 60)]:
        profiles = sensor.get_stream_profiles()
        requests.append([next(p for p in profiles
                              if p.stream_type() == rs.stream.color and p.stream_index() == i
                              and p.format() == rs.format.yuyv and p.fps() == fps
                              and (p.as_video_stream_profile().width(), p.as_video_stream_profile().height()) == (1280, 960))
                         for i in indices])
    frames = {0: [], 1: [], 2: []}
    lock = threading.Lock()

    def receive(frame):
        with lock:
            frames[frame.get_profile().stream_index()].append((time.monotonic(), frame.get_frame_number()))

    opened, started = [], []
    try:
        # Windows requires each sensor to start before the next sensor opens.
        for sensor, profiles in zip(sensors, requests):
            sensor.open(profiles)
            opened.append(sensor)
            sensor.start(receive)
            started.append(sensor)
        time.sleep(3)
        begin = time.monotonic()
        time.sleep(10)
        end = time.monotonic()
    finally:
        for sensor in reversed(started):
            sensor.stop()
        for sensor in reversed(opened):
            sensor.close()
    for index, target in [(0, 60), (1, 30), (2, 30)]:
        data = [n for t, n in frames[index] if begin <= t <= end]
        assert abs(len(data) / (end - begin) - target) < 2, (index, len(data))
        assert all(b == a + 1 for a, b in zip(data, data[1:])), index
