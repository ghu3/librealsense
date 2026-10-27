# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

import json
import pytest
import numpy as np
import pyrealsense2 as rs
from pytest_check import check
import logging
log = logging.getLogger(__name__)

pytestmark = [
    pytest.mark.device("D400*"),
    pytest.mark.device_each("D500*"),
]


def xfail_on_dds(dev, request):
    # DDS FW reports neither a depth-units option nor depth-units metadata yet
    if dev.get_info(rs.camera_info.connection_type) == "DDS":
        request.applymarker(pytest.mark.xfail(reason="DDS FW does not report depth units"))


def start_depth(dev, ctx):
    pipeline = rs.pipeline(ctx)
    cfg = rs.config()
    # On hubless multi-device rigs (e.g. Jetson with D457 + D436) the context sees every
    # connected device; without enable_device(sn) the pipeline picks the first match.
    cfg.enable_device(dev.get_info(rs.camera_info.serial_number))
    # Depth-only: the test validates depth units, and a default config would also wait on
    # color/IMU streams, failing the test if any unrelated stream doesn't deliver.
    cfg.enable_stream(rs.stream.depth)
    pipeline.start(cfg)
    return pipeline


def test_depth_units_metadata(test_device, request):
    """Get metadata depth units value and make sure it's non zero and equal to the depth sensor matching option value"""
    dev, ctx = test_device
    xfail_on_dds(dev, request)

    pipeline = start_depth(dev, ctx)
    try:
        depth_units_from_metadata = pipeline.wait_for_frames().get_depth_frame().get_units()
        ds = pipeline.get_active_profile().get_device().first_depth_sensor()
    finally:
        pipeline.stop()
    check.is_true(depth_units_from_metadata > 0, "Depth units from metadata should be non-zero")

    assert ds.supports(rs.option.depth_units), "Depth sensor should support depth_units option"
    check.equal(ds.get_option(rs.option.depth_units), depth_units_from_metadata,
        f"Depth units option ({ds.get_option(rs.option.depth_units)}) should match metadata ({depth_units_from_metadata})")


@pytest.fixture
def restore_depth_table(test_device_wrapped):
    am = rs.rs400_advanced_mode(test_device_wrapped[0])
    original = am.get_depth_table()
    yield
    am.set_depth_table(original)


@pytest.mark.parametrize("write_path", ["depth-table", "json"])
def test_depth_units_from_advanced_mode(test_device_wrapped, restore_depth_table, write_path, request):
    """Depth units changed through advanced mode while streaming must reach the frames and the depth sensor"""
    dev, ctx = test_device_wrapped
    xfail_on_dds(dev, request)

    def median_distance():
        for _ in range(30):  # let the FW apply the change and older frames drain
            frame = pipeline.wait_for_frames().get_depth_frame()
        raw = np.asanyarray(frame.get_data())
        return frame.get_units(), float(np.median(raw[raw > 0])) * frame.get_units()

    pipeline = start_depth(dev, ctx)
    try:
        # Write through the streaming device: each device handle keeps its own cached depth scale
        streaming_dev = pipeline.get_active_profile().get_device()
        am = rs.rs400_advanced_mode(streaming_dev)
        ds = streaming_dev.first_depth_sensor()
        _, baseline = median_distance()
        for units in (2000, 500):  # micrometers; 500 still covers 32m, so no far pixel saturates
            if write_path == "json":
                am.load_json(json.dumps({"param-depthunits": units}))
            else:
                dt = am.get_depth_table()
                dt.depthUnits = units
                am.set_depth_table(dt)
            expected = units * 1e-6
            frame_units, distance = median_distance()
            check.almost_equal(frame_units, expected, rel=1e-3, msg=f"frame units after setting {units}um")
            check.almost_equal(ds.get_depth_scale(), expected, rel=1e-3, msg=f"depth scale after setting {units}um")
            if ds.supports(rs.option.depth_units):
                check.almost_equal(ds.get_option(rs.option.depth_units), expected, rel=1e-3)
            # The scene did not move, so the measured distance must not follow the units. A JSON load does not
            # restart the depth stream and the FW does not always apply the new units to it, so check the table only
            if write_path == "depth-table":
                check.almost_equal(distance, baseline, rel=0.15, msg=f"median distance after setting {units}um")
    finally:
        pipeline.stop()
