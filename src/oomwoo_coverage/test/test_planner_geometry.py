# Copyright 2026 Jayadev Rana
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Unit tests for planner geometry and reactive command transitions."""

from types import MethodType, SimpleNamespace

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np
from oomwoo_coverage.coverage_planner_node import (
    _compact_sweep,
    _contiguous_runs,
    _decompose_cells,
    _dilate,
    _flood_fill,
    _nearest_true,
    _pt_in_zones,
    _sweep_swath,
    CoveragePlanner,
)
import pytest
from rclpy.clock import ClockType
from rclpy.task import Future
from rclpy.time import Time
from tf2_ros import Buffer


def reactive_driver():
    """Exercise the actual row tick with a recording publisher and pose source."""
    sent, dispatched = [], []
    target = SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=0.5, y=0.0)))
    node = SimpleNamespace(
        awaiting=False, goal_deadline=None, goal_timeout=30.0,
        nav_linear_speed=0.0, robot_xy=None,
        _maybe_wedge_escape=lambda: False,
        _robot_pose=lambda: (0.0, 0.0, 0.0),
        _advance_to_target=lambda: True,
        cached_poses=[target], wp_index=0, wp_retries=0,
        consecutive_skips=0, min_transit_len=0.7, reach_tol=0.12,
        _drive_best_dist=None, _drive_progress_t=None, no_progress_sec=6.0,
        get_clock=lambda: SimpleNamespace(now=lambda: 1.0),
        _elapsed=lambda stamp: 0.0,
        get_logger=lambda: SimpleNamespace(warn=lambda message: None),
        rotate_speed=1.0, k_heading=1.6, align_tol=0.35, v_cruise=0.35,
        cmd_pub=SimpleNamespace(publish=lambda tw: sent.append(
            (tw.linear.x, tw.angular.z))),
        nav_client=SimpleNamespace(server_is_ready=lambda: True),
        _send_goal_to=lambda pose: dispatched.append(sent[-1]),
    )
    return node, sent, dispatched


def keepout_grid(width=64, height=64):
    """Build a virtual zone on empty map pixels with real ROS metadata."""
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.width, grid.info.height = width, height
    grid.info.resolution = .05
    grid.info.origin.position.x = grid.info.origin.position.y = -1.6
    grid.info.origin.orientation.w = 1.
    grid.data = [0] * (width * height)
    grid.data[32 * width + 32] = 100
    return grid


def keepout_driver():
    """Exercise actual mask callbacks/building with the existing STOP path recorded."""
    base = keepout_grid()
    base.data = [0] * len(base.data)
    faults = []
    node = SimpleNamespace(
        map_msg=base, keepout=None, _keepout_geometry_value=None,
        _keepout_fault=None, _keepout_required=True, _relay_enabled=True,
        robot_radius=.18, total_free_cells=0, free_mask=None,
        _relay_fail=faults.append,
        _keepout_geometry=CoveragePlanner._keepout_geometry)
    for name in ('_on_keepout', '_build_masks', '_invalid_keepout'):
        setattr(node, name, MethodType(getattr(CoveragePlanner, name), node))
    return node, faults


def test_required_keepout_waits_for_mask_before_any_plan_free_space():
    node, faults = keepout_driver()
    node._build_masks()
    assert node.free_mask is None and node.total_free_cells == 0
    assert faults == []


def test_keepout_margin_blocks_diagonal_footprint_overlap_and_retains_distant_floor():
    node, faults = keepout_driver()
    node._on_keepout(keepout_grid())
    # At (.15,.15) offset, the disk intersects the forbidden pixel's corner.
    # Cardinal-only dilation would leave this diagonally overlapping cell free.
    assert not node.free_mask[35, 35]
    assert node.free_mask[43, 43]
    assert faults == [] and node.total_free_cells > 0


@pytest.mark.parametrize('invalid', [
    'shift', 'rotation', 'resolution', 'size', 'unknown', 'gray', 'frame',
])
def test_invalid_keepout_geometry_or_encoding_uses_existing_fault_stop(invalid):
    node, faults = keepout_driver()
    mask = keepout_grid(63) if invalid == 'size' else keepout_grid()
    if invalid == 'shift':
        mask.info.origin.position.x += .05
    elif invalid == 'rotation':
        mask.info.origin.orientation.z = .6
        mask.info.origin.orientation.w = .8
    elif invalid == 'resolution':
        mask.info.resolution = .1
    elif invalid == 'unknown':
        mask.data[0] = -1
    elif invalid == 'gray':
        mask.data[0] = 50
    elif invalid == 'frame':
        mask.header.frame_id = 'odom'
    node._on_keepout(mask)
    assert faults and node._keepout_fault
    assert node.free_mask is None and node.total_free_cells == 0


def test_mask_arriving_before_map_is_validated_when_map_arrives():
    node, faults = keepout_driver()
    base = node.map_msg
    node.map_msg = None
    node._on_keepout(keepout_grid())
    base.info.origin.position.x += .05
    node.map_msg = base
    node._build_masks()
    assert faults and node.free_mask is None


def test_changed_mask_cannot_leave_old_plan_running_or_rearm_with_old_bytes():
    node, faults = keepout_driver()
    node._on_keepout(keepout_grid())
    changed = keepout_grid()
    changed.data[0] = 100
    node._on_keepout(changed)
    node._on_keepout(keepout_grid())
    assert faults and node._keepout_fault
    assert node.free_mask is None


@pytest.mark.parametrize('transition', [
    'pose_unavailable', 'no_target', 'reached', 'stalled',
    'transit_ready', 'transit_unready',
])
def test_reactive_exit_publishes_zero_after_prior_motion(transition):
    node, sent, dispatched = reactive_driver()
    CoveragePlanner._tick_reactive(node)
    assert sent[-1] == (0.35, 0.0)  # establish a real preceding row command
    if transition == 'pose_unavailable':
        node._robot_pose = lambda: None
    elif transition == 'no_target':
        node._advance_to_target = lambda: False
    elif transition == 'reached':
        node._robot_pose = lambda: (0.5, 0.0, 0.0)
    elif transition == 'stalled':
        node._elapsed = lambda stamp: 7.0
    else:
        node.cached_poses[0].pose.position.x = 2.0
        node.nav_client.server_is_ready = lambda: transition == 'transit_ready'
    CoveragePlanner._tick_reactive(node)
    assert sent[-1] == (0.0, 0.0)
    assert len(sent) == 2
    assert dispatched == ([(0.0, 0.0)] if transition == 'transit_ready' else [])


def test_reactive_active_transit_does_not_publish_competing_row_stop():
    node, sent, dispatched = reactive_driver()
    node.awaiting = True
    CoveragePlanner._tick_reactive(node)
    assert sent == []
    assert dispatched == []


def timed_reactive_driver():
    """Use real dispatch/results and a controllable ROS clock, without a node."""
    node, sent, _ = reactive_driver()
    now = [1.0]
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=now[0]))
    node._elapsed = lambda stamp: now[0] - stamp.nanoseconds * 1e-9
    node.finished = False
    node.max_retries = 2
    node.get_logger = lambda: SimpleNamespace(
        warn=lambda message: None, info=lambda message: None)
    node._on_goal_response = MethodType(CoveragePlanner._on_goal_response, node)
    node._send_goal_to = MethodType(CoveragePlanner._send_goal_to, node)
    node._dispatch_goal = MethodType(CoveragePlanner._dispatch_goal, node)
    node.nav_client.send_goal_async = lambda goal: SimpleNamespace(
        add_done_callback=lambda callback: None)
    return node, sent, now


def row_target(x):
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.orientation.w = 1.0
    return pose


def contact_pocket_driver():
    """Use the actual contact recorder and waypoint filter with a fresh pose peer."""
    node, sent, _ = timed_reactive_driver()
    node.wedge_zones = []
    node.wedge_avoid_radius = 0.45
    node.escapes_done = 0
    node.max_escapes = 25
    node.escape_sec = 2.5
    node.covered_grid = node.map_msg = None
    node._covered_at = MethodType(CoveragePlanner._covered_at, node)
    node._in_wedge_zone = MethodType(CoveragePlanner._in_wedge_zone, node)
    return node, sent


@pytest.mark.parametrize('robot_xy,skip_target', [
    ((3.0438363696510855, 0.07799449561218721), False),
    ((4.1, 0.025), True),
], ids=['far', 'near'])
def test_bumper_contact_pocket_keeps_unvisited_distant_target(robot_xy, skip_target):
    node, sent = contact_pocket_driver()
    target = row_target(4.225)
    target.pose.position.y = 0.025
    node.cached_poses = [target]
    node._robot_pose = lambda: (*robot_xy, 0.0)
    CoveragePlanner._begin_bumper_escape(node, 0, target, -0.6)
    assert CoveragePlanner._skip_waypoint(node, 0) is skip_target
    assert _pt_in_zones(*robot_xy, node.wedge_zones)
    assert node.escapes_done == 1 and node._escape_az == -0.6
    assert sent == [(0.0, 0.0)]


@pytest.mark.parametrize('pose', [None, (float('nan'), 0.0, 0.0),
                                  (0.0, float('inf'), 0.0)])
def test_contact_without_a_known_position_does_not_invent_a_blocked_target(pose):
    node, sent = contact_pocket_driver()
    target = row_target(2.0)
    node.cached_poses = [target]
    node._robot_pose = lambda: pose
    CoveragePlanner._begin_bumper_escape(node, 0, target, -0.6)
    assert node.wedge_zones == [] and not CoveragePlanner._skip_waypoint(node, 0)
    assert node.escapes_done == 1 and sent == [(0.0, 0.0)]


def test_repeated_contact_at_same_position_deduplicates_the_observed_pocket():
    node, _ = contact_pocket_driver()
    node._robot_pose = lambda: (0.0, 0.0, 0.0)
    for x in (2.0, 4.0):
        CoveragePlanner._begin_bumper_escape(node, 0, row_target(x), -0.6)
    assert node.wedge_zones == [(0.0, 0.0, 0.45)]
    assert node.escapes_done == 2


@pytest.mark.parametrize('cause', ['held_bumper', 'consecutive_skips', 'spent_budget'])
def test_escape_entry_zeroes_previous_row_in_its_scheduling_tick(cause):
    node, sent, _ = timed_reactive_driver()
    CoveragePlanner._tick_reactive(node)
    assert sent == [(0.35, 0.0)]
    node.map_msg = object()
    node.total_free_cells = 1
    node.ext_ratio = 0.0
    node._warn_if_no_feedback = lambda: None
    node.stop_at_target = False
    node.exec_mode = 'reactive'
    node.plan_started = True
    node.escape_until = None
    node.escapes_done = 25 if cause == 'spent_budget' else 0
    node.max_escapes = 25
    node._bumper_held_long = lambda: cause == 'held_bumper'
    node.goal_handle = None
    node.escape_after = 2
    node.escape_sec = 2.5
    node._escape_angular = lambda: -0.6
    node.wedge_zones = []
    node.wedge_avoid_radius = 0.45
    node.consecutive_skips = 0 if cause == 'held_bumper' else 2
    node._maybe_wedge_escape = MethodType(CoveragePlanner._maybe_wedge_escape, node)
    node._begin_bumper_escape = MethodType(CoveragePlanner._begin_bumper_escape, node)
    node._tick_reactive = MethodType(CoveragePlanner._tick_reactive, node)
    CoveragePlanner._tick(node)
    assert sent == [(0.35, 0.0), (0.0, 0.0)]


@pytest.mark.parametrize('result', [4, 6, 'rejected'])
def test_reactive_return_from_transit_gets_a_fresh_progress_interval(result):
    node, sent, now = timed_reactive_driver()
    node.cached_poses = [row_target(0.2)]
    CoveragePlanner._tick_reactive(node)  # preceding row got close to its target
    assert sent[-1] == (0.35, 0.0)

    # A later long connector delegates to the actual Nav2 dispatch method.
    node.cached_poses = [row_target(2.0), row_target(2.6)]
    now[0] = 2.0
    CoveragePlanner._tick_reactive(node)
    assert node.awaiting
    assert sent[-1] == (0.0, 0.0)

    # Time in Nav2 exceeds the row watchdog; it is not time stuck in this row.
    now[0] = 20.0
    if result == 'rejected':
        node._on_goal_response(SimpleNamespace(result=lambda: SimpleNamespace(
            accepted=False)))
    else:
        CoveragePlanner._on_result(node, SimpleNamespace(
            result=lambda: SimpleNamespace(status=result)))
    target_index = 1 if result == 4 else 0
    node._robot_pose = lambda: (
        node.cached_poses[target_index].pose.position.x - 0.4, 0.0, 0.0)
    CoveragePlanner._tick_reactive(node)
    assert node.wp_index == target_index, 'fresh row was falsely skipped'
    assert sent[-1] == (0.35, 0.0)

    # A genuine new stall must still stop and skip after its own time budget.
    now[0] = 27.0
    CoveragePlanner._tick_reactive(node)
    assert node.wp_index == target_index + 1
    assert sent[-1] == (0.0, 0.0)


@pytest.mark.parametrize('skipped_count', [1, 2])
def test_covered_waypoint_skip_starts_progress_for_the_new_target(skipped_count):
    node, sent, now = timed_reactive_driver()
    node.cached_poses = [row_target(0.2), row_target(0.4), row_target(0.6)]
    covered = set()
    node._skip_waypoint = lambda index: index in covered
    node._advance_to_target = MethodType(CoveragePlanner._advance_to_target, node)
    CoveragePlanner._tick_reactive(node)
    assert sent[-1] == (0.35, 0.0)
    now[0] = 10.0
    covered.update(range(skipped_count))
    CoveragePlanner._tick_reactive(node)
    assert node.wp_index == skipped_count, 'new target inherited an old stall'
    assert sent[-1] == (0.35, 0.0)
    now[0] = 17.0
    CoveragePlanner._tick_reactive(node)
    assert node.wp_index == skipped_count + 1
    assert sent[-1] == (0.0, 0.0)


def test_gapfill_plan_starts_a_fresh_reactive_progress_interval():
    node, sent, now = timed_reactive_driver()
    node.cached_poses = [row_target(0.2)]
    node._skip_waypoint = lambda index: False
    node._advance_to_target = MethodType(CoveragePlanner._advance_to_target, node)
    node.gapfill_passes, node.max_gapfill = 0, 1
    node.stop_at_target, node.ext_ratio = False, 0.8
    node._gapfill_waypoints = lambda: [row_target(0.4)]
    node._publish_plan = lambda: None
    CoveragePlanner._tick_reactive(node)
    assert sent[-1] == (0.35, 0.0)
    now[0] = 10.0
    node.wp_index = 1  # exhausted old plan; next tick installs a gap-fill plan
    CoveragePlanner._tick_reactive(node)
    assert sent[-1] == (0.0, 0.0)
    assert node.gapfill_passes == 1
    CoveragePlanner._tick_reactive(node)
    assert node.wp_index == 0, 'new gap-fill target inherited the prior row stall'
    assert sent[-1] == (0.35, 0.0)


def gapfill_with_small_unclean_hole(center_value=0):
    """Real gap-fill selection on a mostly clean raster with one dirty cell."""
    node, _, _ = timed_reactive_driver()
    node.map_msg = OccupancyGrid()
    node.map_msg.info.resolution = 0.05
    node.map_msg.info.width = node.map_msg.info.height = 21
    node.covered_grid = np.full((21, 21), 100, dtype=np.int8)
    node.covered_grid[6, 6] = center_value
    node.free_mask = np.ones((21, 21), dtype=bool)
    node.robot_xy = (0.1, 0.1)
    node.cleaning_radius = 0.2
    node.global_frame = 'map'
    node.wedge_zones = []
    node.gapfill_passes, node.max_gapfill = 1, 3
    node.stop_at_target, node.ext_ratio = False, 0.98
    node._gapfill_waypoints = MethodType(CoveragePlanner._gapfill_waypoints, node)
    node._covered_at = MethodType(CoveragePlanner._covered_at, node)
    node._in_wedge_zone = MethodType(CoveragePlanner._in_wedge_zone, node)
    node._skip_waypoint = MethodType(CoveragePlanner._skip_waypoint, node)
    node._publish_plan = lambda: None
    node.active_pub = SimpleNamespace(publish=lambda msg: None)
    node.cached_poses = node._gapfill_waypoints()
    assert len(node.cached_poses) == 1
    return node


@pytest.mark.parametrize('center_value', [0, 50, -1])
def test_gapfill_does_not_discard_a_hole_inside_a_mostly_clean_disk(center_value):
    node = gapfill_with_small_unclean_hole(center_value)
    # The existing selection found a genuinely uncredited target cell. Nearby
    # clean cells must not consume all three passes without visiting it.
    assert CoveragePlanner._advance_to_target(node)
    assert node.wp_index == 0
    assert node.gapfill_passes == 1
    assert not node.finished


def test_main_sweep_keeps_its_mostly_clean_disk_shortcut():
    node = gapfill_with_small_unclean_hole()
    node.gapfill_passes = 0
    assert node._skip_waypoint(0)


def test_gapfill_can_skip_a_target_cleaned_since_the_plan_was_built():
    node = gapfill_with_small_unclean_hole()
    node.covered_grid[6, 6] = 100
    assert node._skip_waypoint(0)
    assert not CoveragePlanner._advance_to_target(node)
    assert node.finished


def test_gapfill_still_skips_a_known_wedge_pocket():
    node = gapfill_with_small_unclean_hole()
    p = node.cached_poses[0].pose.position
    node.wedge_zones = [(p.x, p.y, 0.45)]
    assert node._skip_waypoint(0)
    assert not CoveragePlanner._advance_to_target(node)
    assert node.finished


def test_missing_coverage_does_not_credit_a_gapfill_target_as_clean():
    node = gapfill_with_small_unclean_hole()
    node.covered_grid = None
    assert not node._skip_waypoint(0)


def test_pt_in_zones_inside_and_outside():
    zones = [(1.0, 1.0, 0.35), (-2.0, 0.5, 0.5)]
    assert _pt_in_zones(1.1, 0.9, zones)          # inside first pocket
    assert _pt_in_zones(-2.0, 0.5, zones)         # dead centre of second
    assert not _pt_in_zones(1.5, 1.5, zones)      # between, outside both
    assert not _pt_in_zones(0.0, 0.0, [])         # no zones -> never inside


def test_pt_in_zones_boundary_is_inclusive():
    # a point exactly on the radius counts as inside (<=), so a wedge spot
    # can't slip back in on a floating-point tie
    assert _pt_in_zones(0.35, 0.0, [(0.0, 0.0, 0.35)])


def test_contiguous_runs_splits_on_gaps():
    runs = _contiguous_runs(np.array([1, 2, 3, 7, 8, 12]))
    assert [list(r) for r in runs] == [[1, 2, 3], [7, 8], [12]]


def test_contiguous_runs_empty_and_single():
    assert _contiguous_runs(np.array([], dtype=int)) == []
    assert [list(r) for r in _contiguous_runs(np.array([5]))] == [[5]]


def test_dilate_zero_radius_returns_copy():
    mask = np.zeros((5, 5), dtype=bool)
    mask[2, 2] = True
    out = _dilate(mask, 0)
    assert bool(np.array_equal(out, mask))
    out[0, 0] = True
    assert not bool(mask[0, 0]), 'radius-0 dilation must return a copy'


def test_dilate_radius_reaches_manhattan_ball():
    mask = np.zeros((9, 9), dtype=bool)
    mask[4, 4] = True
    out = _dilate(mask, 2)
    assert bool(out[2, 4]) and bool(out[4, 2]) and bool(out[3, 3])
    assert not bool(out[1, 4])


def test_flood_fill_two_chambers_stay_separate():
    mask = np.ones((6, 8), dtype=bool)
    mask[:, 4] = False                       # wall splits the room in two
    left = _flood_fill(mask, (0, 0))
    assert bool(left[5, 3]) and not left[:, 5:].any()


def compact_room_fixture():
    """Load the real room and pre-change cell tracks as independent reference data."""
    import json
    from pathlib import Path
    import yaml
    from oomwoo_sim_support.keepout_config import _pgm

    root = Path(__file__).resolve().parents[2] / 'oomwoo_sim_support'
    meta = yaml.safe_load((root / 'maps/test_room.yaml').read_text())
    width, height, pixels = _pgm(root / 'maps/test_room.pgm')
    occupancy = (255 - np.flipud(np.frombuffer(pixels, dtype=np.uint8).reshape(
        height, width)).astype(float)) / 255
    grid = np.where(occupancy > meta['occupied_thresh'], 100,
                    np.where(occupancy < meta['free_thresh'], 0, -1))
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.width, msg.info.height = width, height
    msg.info.resolution = meta['resolution']
    msg.info.origin.position.x, msg.info.origin.position.y = meta['origin'][:2]
    msg.info.origin.orientation.w = 1.0
    msg.data = grid.reshape(-1).astype(int).tolist()
    mask = OccupancyGrid()
    mask.header, mask.info = msg.header, msg.info
    mask.data = [0] * (width * height)
    for row in range(height):
        for col in range(width):
            x = meta['origin'][0] + (col + .5) * meta['resolution']
            y = meta['origin'][1] + (row + .5) * meta['resolution']
            if .67 <= x <= 1.14 and -.55 <= y <= .55:
                mask.data[row * width + col] = 100
    node = SimpleNamespace(
        map_msg=msg, keepout=None, _keepout_geometry_value=None, _keepout_fault=None,
        _keepout_required=True, _relay_enabled=True, robot_radius=.18,
        total_free_cells=0, free_mask=None, robot_xy=(0., 0.), cleaning_radius=.20,
        row_overlap=0., min_segment_len=.20, row_substep_m=.4, long_axis_sweep=True,
        global_frame='map', get_logger=lambda: SimpleNamespace(
            info=lambda message: None, warn=lambda message: None))
    node._keepout_geometry = CoveragePlanner._keepout_geometry
    node._invalid_keepout = lambda message: pytest.fail(message)
    node._build_masks = MethodType(CoveragePlanner._build_masks, node)
    CoveragePlanner._on_keepout(node, mask)
    reference_path = root / 'docs/evidence/narrow_local_caps_review_20261030.json'
    receipt = json.loads(reference_path.read_text())
    return node, grid == 0, receipt['result']['tracks']['current']


def track_plan_cost(tracks):
    """Count actual complete track lengths and seed-to-first distance."""
    import math
    points = [(0., 0.)] + [p for track in tracks for p in track]
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def test_normal_room_plan_reduces_measured_redundancy_without_a_supplied_route(monkeypatch):
    from oomwoo_coverage import coverage_planner_node as module
    captured = []
    original = module._compact_sweep

    def observe(*args):
        result = original(*args)
        captured.append((args, result))
        return result

    monkeypatch.setattr(module, '_compact_sweep', observe)
    node, _, reference = compact_room_fixture()
    poses = CoveragePlanner._plan_waypoints(node)
    points = [(p.pose.position.x, p.pose.position.y) for p in poses]
    before = track_plan_cost([t['points'] for t in reference])
    assert before - track_plan_cost([points]) > 5.0
    info = node.map_msg.info
    for x, y in points:
        cx = int((x - info.origin.position.x) / info.resolution)
        cy = int((y - info.origin.position.y) / info.resolution)
        assert node.free_mask[cy, cx]

    args, compact = captured[0]
    tracks, allowed, floor, origin, resolution, radius, _, _ = args
    old_swath = independent_track_swath(
        [points for _, _, points in tracks], allowed, floor, origin, resolution, radius)
    new_swath = independent_track_swath(compact, allowed, floor, origin, resolution, radius)
    assert not (old_swath & ~new_swath).any()
    for mask, vertical, original_points in tracks:
        if (mask.shape[1] if vertical else mask.shape[0]) > 8:
            assert any(points == original_points or points == original_points[::-1]
                       for points in compact)


def independent_track_swath(tracks, allowed, floor, origin, resolution, radius):
    """Use the actual external meter's disks, independently of planner morphology."""
    import math
    from oomwoo_sim_support.coverage_meter_node import _stamp_disk

    swath = np.zeros_like(allowed)
    h, w = allowed.shape

    def cell(point):
        return int((point[0] - origin[0]) / resolution), int((point[1] - origin[1]) / resolution)

    for points in tracks:
        cells = {cell(point) for point in points}
        for a, b in zip(points, points[1:]):
            count = max(1, int(math.ceil(math.dist(a, b) / (resolution / 4))))
            segment = {cell((a[0] + (b[0] - a[0]) * i / count,
                             a[1] + (b[1] - a[1]) * i / count)) for i in range(count + 1)}
            if all(0 <= y < h and 0 <= x < w and allowed[y, x] for x, y in segment):
                cells |= segment
        for x, y in cells:
            _stamp_disk(swath, floor, x, y, radius)
    return swath


@pytest.mark.parametrize('origin', [(0., 0.), (-1.6, -3.), (.03, .07)])
@pytest.mark.parametrize('resolution', [.05, .05000000074505806])
@pytest.mark.parametrize('end_cell', [(18, 13), (11, 29), (27, 30)])
def test_sweep_swath_reversal_preserves_same_geometric_curve(origin, resolution, end_cell):
    allowed = floor = np.ones((45, 45), dtype=bool)
    points = [(origin[0] + (x + .5) * resolution,
               origin[1] + (y + .5) * resolution) for x, y in [(10, 10), end_cell]]
    before = _sweep_swath(points, allowed, floor, origin, resolution, 4)
    after = _sweep_swath(points[::-1], allowed, floor, origin, resolution, 4)
    assert np.array_equal(before, after)


def narrow_track_fixture(*, vertical=False, free_halo=False):
    """Build a two-pass strip with optional free halo that makes deletion unsafe."""
    allowed = np.zeros((25, 40), dtype=bool)
    allowed[10:13, 5:31] = True
    mask = np.ones((3, 26), dtype=bool)
    xs = [5, 13, 21, 29, 30]
    points = [((x + .5) * .05, (row + .5) * .05)
              for row, cols in ((10, xs), (12, xs[::-1])) for x in cols]
    if vertical:
        allowed, mask = allowed.T.copy(), mask.T.copy()
        points = [(y, x) for x, y in points]
    floor = np.ones_like(allowed) if free_halo else allowed.copy()
    return [(mask, vertical, points)], allowed, floor


@pytest.mark.parametrize('vertical', [False, True])
def test_compact_sweep_caps_keep_whole_free_swath_and_do_not_mutate_input(vertical):
    tracks, allowed, floor = narrow_track_fixture(vertical=vertical)
    before = list(tracks[0][2])
    result = _compact_sweep(tracks, allowed, floor, (0., 0.), .05, 4, 8, (0., 0.))
    assert result is not None and sum(map(len, result)) < len(before)
    assert tracks[0][2] == before
    old_swath = independent_track_swath([before], allowed, floor, (0., 0.), .05, 4)
    new_swath = independent_track_swath(result, allowed, floor, (0., 0.), .05, 4)
    assert not (old_swath & ~new_swath).any()


@pytest.mark.parametrize('vertical', [False, True])
def test_compact_sweep_retains_two_passes_when_free_halo_would_be_lost(vertical):
    tracks, allowed, floor = narrow_track_fixture(vertical=vertical, free_halo=True)
    assert _compact_sweep(tracks, allowed, floor, (0., 0.), .05, 4, 8, (0., 0.)) is None


def test_compact_sweep_retains_nonrectangular_cell():
    tracks, allowed, floor = narrow_track_fixture()
    tracks[0][0][1, 1] = False
    assert _compact_sweep(tracks, allowed, floor, (0., 0.), .05, 4, 8, (0., 0.)) is None


@pytest.mark.parametrize('budget', ['cells', 'points', 'radius', 'grid', 'distance'])
def test_compact_sweep_retains_original_before_expensive_work(monkeypatch, budget):
    from oomwoo_coverage import coverage_planner_node as module
    tracks, allowed, floor = narrow_track_fixture()
    radius = 4
    if budget == 'cells':
        tracks *= 33
    elif budget == 'points':
        tracks = [(tracks[0][0], False, [tracks[0][2][0]] * 4097)]
    elif budget == 'radius':
        radius = 17
    elif budget == 'distance':
        tracks = [(tracks[0][0], False, [(0., 0.), (1000., 1000.)])]
    else:
        allowed = floor = np.ones((1024, 1024), dtype=bool)
    monkeypatch.setattr(module, '_sweep_swath', lambda *args: pytest.fail('budget ignored'))
    assert _compact_sweep(tracks, allowed, floor, (0., 0.), .05, radius, 8, (0., 0.)) is None


def test_normal_room_plan_retains_exact_original_when_candidate_cost_is_worse(monkeypatch):
    from oomwoo_coverage import coverage_planner_node as module
    monkeypatch.setattr(module, '_compact_sweep', lambda tracks, *args:
                        [points for _, _, points in tracks] + [tracks[0][2]])
    node, _, reference = compact_room_fixture()
    poses = CoveragePlanner._plan_waypoints(node)
    points = [(p.pose.position.x, p.pose.position.y) for p in poses]
    assert points == [tuple(p) for track in reference for p in track['points']]


def test_decompose_open_room_is_one_cell():
    free = np.zeros((20, 20), dtype=bool)
    free[1:19, 1:19] = True
    cells = _decompose_cells(free)
    assert len(cells) == 1
    assert len(cells[0]) == 18                  # every interior row present


def test_decompose_sofa_splits_into_four_cells():
    # a central obstacle (the "sofa") must yield: below, left-of, right-of,
    # above — four cells, each with exactly ONE interval per row, so a
    # serpentine can sweep each without transiting around the obstacle
    free = np.zeros((30, 30), dtype=bool)
    free[1:29, 1:29] = True
    free[10:20, 10:20] = False                  # the sofa
    cells = _decompose_cells(free)
    assert len(cells) == 4
    for cell in cells:
        rows = [r for r, _, _ in cell]
        assert len(rows) == len(set(rows)), 'one interval per row per cell'
    total = sum(b - a + 1 for cell in cells for _, a, b in cell)
    assert total == int(free.sum()), 'decomposition must cover all free cells'


def test_decompose_two_obstacles_seven_cells():
    free = np.zeros((40, 40), dtype=bool)
    free[1:39, 1:39] = True
    free[8:14, 5:15] = False
    free[22:30, 20:35] = False
    cells = _decompose_cells(free)
    assert len(cells) == 7
    total = sum(b - a + 1 for cell in cells for _, a, b in cell)
    assert total == int(free.sum())


def test_decompose_no_cell_straddles_the_obstacle():
    free = np.zeros((30, 30), dtype=bool)
    free[1:29, 1:29] = True
    free[10:20, 10:20] = False
    for cell in _decompose_cells(free):
        for r, a, b in cell:
            if 10 <= r < 20:
                assert b < 10 or a >= 20, \
                    'a cell interval must never span the obstacle'


def test_nearest_true_prefers_close_cells():
    mask = np.zeros((10, 10), dtype=bool)
    mask[3, 3] = True
    mask[9, 9] = True
    assert _nearest_true(mask, cx=4, cy=4) == (3, 3)
    assert _nearest_true(np.zeros((4, 4), dtype=bool), 1, 1, max_r=3) is None


@pytest.mark.parametrize('stamp_s, cached_amcl, expected_motion', [
    (9.8, False, True),       # timely dynamic TF
    (9.5, False, True),       # inclusive 500 ms budget
    (9.499, False, False),    # just outside the budget
    (9.0, True, False),       # cached AMCL must not mask stale TF
    (10.1, True, False),      # future transform after a clock rewind
    (0.0, True, False),       # no valid dynamic localization timestamp
    (None, True, False),      # missing TF, with a historical AMCL seed
    (None, False, False),
])
def test_reactive_motion_requires_timely_tf(stamp_s, cached_amcl, expected_motion):
    """Drive only from timely TF; cached planning poses cannot revive motion."""
    node, sent, _ = reactive_driver()
    node.global_frame, node.base_frame = 'map', 'base_link'
    node.tf_buffer = Buffer()
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: Time(seconds=10, clock_type=ClockType.ROS_TIME))
    node.robot_xy = (0.0, 0.0) if cached_amcl else None
    node.robot_yaw = 0.0
    node._robot_pose = MethodType(CoveragePlanner._robot_pose, node)
    if stamp_s is not None:
        transform = TransformStamped()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_link'
        transform.header.stamp = Time(seconds=stamp_s).to_msg()
        transform.transform.rotation.w = 1.0
        node.tf_buffer.set_transform(transform, 'focused_localization_test')
    CoveragePlanner._tick_reactive(node)
    assert sent[-1] == ((0.35, 0.0) if expected_motion else (0.0, 0.0))


def test_reactive_tf_dropout_stops_prior_motion_despite_cached_amcl():
    """An aging TF cache must stop the next row tick after earlier movement."""
    node, sent, _ = reactive_driver()
    node.global_frame, node.base_frame = 'map', 'base_link'
    node.tf_buffer = Buffer()
    now = [10.0]
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: Time(seconds=now[0], clock_type=ClockType.ROS_TIME))
    node.robot_xy, node.robot_yaw = (0.0, 0.0), 0.0
    node._elapsed = lambda stamp: now[0] - stamp.nanoseconds * 1e-9
    node._robot_pose = MethodType(CoveragePlanner._robot_pose, node)
    transform = TransformStamped()
    transform.header.frame_id = 'map'
    transform.child_frame_id = 'base_link'
    transform.header.stamp = Time(seconds=10).to_msg()
    transform.transform.rotation.w = 1.0
    node.tf_buffer.set_transform(transform, 'focused_localization_test')
    CoveragePlanner._tick_reactive(node)
    assert sent[-1] == (0.35, 0.0)
    now[0] = 10.6
    CoveragePlanner._tick_reactive(node)
    assert sent == [(0.35, 0.0), (0.0, 0.0)]
    # Time without localization must not exhaust the row's progress budget.
    now[0] = 20.0
    transform.header.stamp = Time(seconds=now[0]).to_msg()
    node.tf_buffer.set_transform(transform, 'focused_localization_test')
    CoveragePlanner._tick_reactive(node)
    assert sent[-1] == (0.35, 0.0)
    assert node.wp_index == 0
    now[0] = 27.0
    transform.header.stamp = Time(seconds=now[0]).to_msg()
    node.tf_buffer.set_transform(transform, 'focused_localization_test')
    CoveragePlanner._tick_reactive(node)
    assert sent[-1] == (0.0, 0.0)  # a genuine fresh-pose stall still skips
    assert node.wp_index == 1


def test_reactive_accepts_current_common_tf_with_future_amcl_edge():
    """Use common map-to-base time, allowing AMCL's future edge tolerance."""
    node, sent, _ = reactive_driver()
    node.global_frame, node.base_frame = 'map', 'base_link'
    node.robot_xy = None
    node.tf_buffer = Buffer()
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: Time(seconds=10, clock_type=ClockType.ROS_TIME))
    node._robot_pose = MethodType(CoveragePlanner._robot_pose, node)
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = 'map', 'odom'
    transform.transform.rotation.w = 1.0
    for stamp in (9, 15):  # AMCL brackets now and leads it by its 5 s tolerance
        transform.header.stamp = Time(seconds=stamp).to_msg()
        node.tf_buffer.set_transform(transform, 'focused_localization_test')
    transform.header.frame_id, transform.child_frame_id = 'odom', 'base_link'
    transform.header.stamp = Time(seconds=9.8).to_msg()
    node.tf_buffer.set_transform(transform, 'focused_localization_test')
    CoveragePlanner._tick_reactive(node)
    assert sent[-1] == (0.35, 0.0)


def private_relay_planner(tmp_path):
    """Bind actual planner methods with asynchronous RPC/action recording peers."""
    journal_module = pytest.importorskip('oomwoo_cleaning_jobs.goal_journal')
    node, sent, now = timed_reactive_driver()
    node._relay_enabled = True
    node._relay_journal = journal_module.GoalJournal(tmp_path / 'goal.json')
    node._relay_goal_pose = None
    node._relay_stage = 'sweep'
    node._relay_fault = None
    node._relay_epoch = 0
    node._relay_owner = 'reactive_sweep'
    node._relay_rpc = None
    node._relay_status_future = None
    node._relay_status_started = None
    node._relay_status_client = SimpleNamespace(service_is_ready=lambda: False)
    node._relay_status_epoch = 0
    node._relay_status_last = None
    node._relay_poll = MethodType(CoveragePlanner._relay_poll, node)
    node._relay_fail = lambda reason: pytest.fail(reason)
    node.map_msg = OccupancyGrid()
    node.map_msg.header.frame_id = 'map'
    node.map_msg.info.width = node.map_msg.info.height = 20
    node.map_msg.info.resolution = 0.25
    node.map_msg.info.origin.orientation.w = 1.0
    node.map_msg.data = [0] * 400
    sends = []

    def actual_send(goal, **options):
        sends.append((node._relay_journal.read(), bytes(options['goal_uuid'].uuid).hex()))
        return SimpleNamespace(add_done_callback=lambda callback: None)

    node.nav_client.send_goal_async = actual_send
    return node, sent, sends


def relay_reply(node, operation, *, ok=True, owner='stopped'):
    node._relay_rpc_operation = operation
    node._relay_rpc = SimpleNamespace(done=lambda: True, result=lambda: SimpleNamespace(
        ok=ok, state=owner, error='quiet interval not complete' if not ok else ''))
    CoveragePlanner._relay_poll(node)


def test_relay_journals_uuid_before_successful_nav_permission_and_send(tmp_path):
    node, sent, sends = private_relay_planner(tmp_path)
    CoveragePlanner._send_goal_to(node, row_target(2.0))
    assert node._relay_stage == 'release_sweep'
    assert sent[-1] == (0.0, 0.0) and sends == []
    relay_reply(node, 'release_motion')
    record = node._relay_journal.read()
    assert record['state'] == 'submitting'
    assert node._relay_stage == 'claim_nav' and sends == []
    relay_reply(node, 'claim_nav', ok=False)
    assert node._relay_journal.read() == record and sends == []
    relay_reply(node, 'claim_nav', owner='navigation')
    assert sends == [(record, record['uuid'])]
    assert node.awaiting and node._relay_stage == 'nav'


def test_preclaim_status_reply_cannot_overwrite_new_navigation_owner(tmp_path):
    import time
    node, _, sends = private_relay_planner(tmp_path)
    CoveragePlanner._send_goal_to(node, row_target(2.0))
    relay_reply(node, 'release_motion')
    old_epoch = node._relay_epoch
    node._relay_status_epoch = old_epoch
    node._relay_status_started = time.monotonic()
    node._relay_status_future = SimpleNamespace(done=lambda: True, result=lambda: SimpleNamespace(
        ok=True, status_json='{"owner":"stopped"}'))
    relay_reply(node, 'claim_nav', owner='navigation')
    assert node._relay_epoch > old_epoch
    assert node._relay_owner == 'navigation' and len(sends) == 1


def status_relay_planner(tmp_path, monkeypatch):
    """Use real rclpy futures with the planner's existing status poll/tick."""
    node, sent, sends = private_relay_planner(tmp_path)
    now, requests, removed = [10.0], [], []
    monkeypatch.setattr('oomwoo_coverage.coverage_planner_node.time.monotonic',
                        lambda: now[0])

    def call_async(request):
        future = Future()
        requests.append(future)
        return future

    node._relay_status_client = SimpleNamespace(
        service_is_ready=lambda: True, call_async=call_async,
        remove_pending_request=removed.append)
    node._relay_status_type = SimpleNamespace(Request=lambda: object())
    node._relay_snapshot = {'owner': 'reactive_sweep'}
    return node, now, requests, removed, sent, sends


@pytest.mark.parametrize('late_done', [False, True])
def test_relay_status_lost_or_late_response_is_replaced_without_refresh(
        tmp_path, monkeypatch, late_done):
    node, now, requests, removed, _, sends = status_relay_planner(tmp_path, monkeypatch)
    node._relay_poll()
    first = requests[0]
    now[0] = 11.4
    node._relay_poll()
    assert requests == [first] and removed == []
    if late_done:
        first.set_result(SimpleNamespace(ok=True, status_json='{"owner":"navigation"}'))
    now[0] = 11.6
    node._relay_poll()
    assert removed == [first] and len(requests) == 2
    assert node._relay_status_last is None
    assert node._relay_snapshot == {'owner': 'reactive_sweep'}
    assert sends == []
    if not late_done:
        assert first.cancelled()
    requests[1].set_result(SimpleNamespace(ok=True, status_json='{"owner":"stopped"}'))
    now[0] = 11.8
    node._relay_poll()
    assert node._relay_owner == 'stopped'
    assert node._relay_status_last == 11.8 and len(requests) == 3


def test_relay_status_late_abandoned_future_cannot_override_current_reply(tmp_path, monkeypatch):
    node, now, requests, removed, _, _ = status_relay_planner(tmp_path, monkeypatch)
    node._relay_poll()
    first = requests[0]
    now[0] = 11.6
    node._relay_poll()
    # Simulate an executor which had already taken the abandoned response.
    first.set_result(SimpleNamespace(ok=True, status_json='{"owner":"navigation"}'))
    requests[1].set_result(SimpleNamespace(ok=True, status_json='{"owner":"stopped"}'))
    node._relay_poll()
    assert node._relay_owner == 'stopped' and removed == [first]


def test_relay_status_reply_before_control_epoch_cannot_renew_health(tmp_path, monkeypatch):
    node, now, requests, _, _, _ = status_relay_planner(tmp_path, monkeypatch)
    node._relay_poll()
    node._relay_epoch += 1
    node._relay_owner = 'navigation'
    requests[0].set_result(SimpleNamespace(ok=True, status_json='{"owner":"stopped"}'))
    now[0] = 10.2
    node._relay_poll()
    assert node._relay_owner == 'navigation' and node._relay_status_last is None
    assert node._relay_status_epoch == node._relay_epoch


def test_relay_status_retries_preserve_active_stale_stop(tmp_path, monkeypatch):
    node, now, requests, removed, sent, _ = status_relay_planner(tmp_path, monkeypatch)
    node._relay_stage = node._relay_last_stage = 'nav'
    node._relay_stage_wall = 10.0
    node._relay_owner = 'navigation'
    node._relay_status_last = 10.0
    node._relay_snapshot = {'owner': 'navigation'}
    node.finished, node.plan_started, node.awaiting = False, True, False
    node._relay_phase = MethodType(CoveragePlanner._relay_phase, node)
    node._relay_phase_pub = SimpleNamespace(publish=lambda msg: None)
    node._relay_nav_contact = lambda: False
    node._relay_fail = MethodType(CoveragePlanner._relay_fail, node)
    node.goal_handle = None
    node._relay_stop_future = None
    node._relay_control = SimpleNamespace(service_is_ready=lambda: False)
    node.get_logger = lambda: SimpleNamespace(error=lambda message: None)
    assert CoveragePlanner._relay_tick(node) is True
    now[0] = 11.6
    assert CoveragePlanner._relay_tick(node) is True
    assert len(requests) == 2 and removed == [requests[0]]
    assert node._relay_status_last == 10.0 and node._relay_fault is None
    now[0] = 12.6
    assert CoveragePlanner._relay_tick(node) is True
    assert node._relay_fault == 'relay status stopped arriving'
    assert node._relay_stage == 'fault' and sent[-1] == (0.0, 0.0)


def test_relay_status_unavailable_server_does_not_accumulate_pending_requests(
        tmp_path, monkeypatch):
    node, now, requests, removed, _, _ = status_relay_planner(tmp_path, monkeypatch)
    node._relay_poll()
    node._relay_status_client.service_is_ready = lambda: False
    now[0] = 11.6
    node._relay_poll()
    assert len(requests) == len(removed) == 1
    assert node._relay_status_future is None
    now[0] = 30.0
    node._relay_poll()
    assert len(requests) == 1 and node._relay_status_last is None


def test_relay_status_real_client_discards_old_dds_response_and_recovers(tmp_path, monkeypatch):
    """Check transport cleanup/requery with a built-in wire type, independent of payload."""
    import time
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from std_srvs.srv import Trigger

    node, now, _, _, _, _ = status_relay_planner(tmp_path, monkeypatch)
    context = Context()
    context.init(args=[])
    ros_node = rclpy.create_node('status_retry_contract', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(ros_node)
    served = []

    def respond(request, response):
        served.append(request)
        response.success = True
        response.message = 'actual response'
        return response

    try:
        service = ros_node.create_service(Trigger, '~/status', respond)
        client = ros_node.create_client(Trigger, '~/status')
        assert client.wait_for_service(timeout_sec=2.0)
        node._relay_status_client = client
        node._relay_status_type = Trigger
        node._relay_poll()
        abandoned = node._relay_status_future
        assert abandoned is not None and not abandoned.done()
        now[0] = 11.6
        node._relay_poll()
        current = node._relay_status_future
        assert abandoned.cancelled() and current is not abandoned
        assert list(client._pending_requests.values()) == [current]
        end = time.perf_counter() + 5.0
        while not current.done() and time.perf_counter() < end:
            executor.spin_once(timeout_sec=0.05)
        assert current.done() and len(served) == 2
        assert abandoned.cancelled() and not abandoned.done()
        assert current.result().success and current.result().message == 'actual response'
        assert node._relay_status_last is None
        # The executor schedules the normal client's cleanup as a separate task.
        while client._pending_requests and time.perf_counter() < end:
            executor.spin_once(timeout_sec=0.05)
        assert client._pending_requests == {}
        ros_node.destroy_service(service)
    finally:
        executor.shutdown(timeout_sec=1.0)
        ros_node.destroy_node()
        context.shutdown()


def test_finished_owned_sweep_requests_release_before_legacy_finished_return():
    node, sent, _ = reactive_driver()
    operations, phases = [], []
    node._relay_poll = lambda: None
    node.finished = node.plan_started = True
    node._relay_stage = 'sweep'
    node._relay_fault = None
    node._relay_owner = 'reactive_sweep'
    node._relay_status_last = None
    node._relay_rpc = None
    node._relay_retry_at = 0.0
    node._relay_last_stage = None
    node._relay_stage_wall = 0.0
    node._relay_phase = MethodType(CoveragePlanner._relay_phase, node)
    node._relay_phase_pub = SimpleNamespace(publish=lambda msg: phases.append(msg.data))
    node._relay_snapshot = {'last_output': [0.0, 0.0],
                            'motion_inputs': {'reactive_sweep': {'zero': True}}}
    node._relay_request = operations.append
    assert CoveragePlanner._relay_tick(node) is True
    assert operations == ['release_motion'] and phases == ['IDLE']
    assert sent[-1] == (0.0, 0.0)


def test_relay_boot_deadline_also_bounds_a_nav_server_that_never_starts(monkeypatch):
    node, sent, _ = reactive_driver()
    node._relay_poll = lambda: None
    node.finished = node.plan_started = False
    node._relay_stage = node._relay_last_stage = 'boot'
    node._relay_fault = None
    node._relay_stage_wall = 0.0
    node.goal_timeout = 30.0
    node._nav_goal_budget_s = 60.0
    node._relay_snapshot = {}
    node._relay_epoch = 0
    node.goal_handle = None
    node._relay_stop_future = None
    node._relay_control = SimpleNamespace(service_is_ready=lambda: False)
    node.get_logger = lambda: SimpleNamespace(error=lambda message: None)
    node._relay_phase = MethodType(CoveragePlanner._relay_phase, node)
    node._relay_fail = MethodType(CoveragePlanner._relay_fail, node)
    node._relay_phase_pub = SimpleNamespace(publish=lambda message: None)
    monkeypatch.setattr('oomwoo_coverage.coverage_planner_node.time.monotonic', lambda: 31.0)
    assert CoveragePlanner._relay_tick(node) is True
    assert node._relay_stage == 'fault'
    assert node._relay_fault.startswith('handoff timed out in boot')
    assert sent == [(0.0, 0.0)]


def contact_relay_planner(tmp_path, *, accepted=True):
    """Use the actual journal, dispatcher, reflex and asynchronous cancel methods."""
    node, sent, _ = private_relay_planner(tmp_path)
    node.cached_poses = [row_target(2.0)]
    CoveragePlanner._send_goal_to(node, node.cached_poses[0])
    relay_reply(node, 'release_motion')
    relay_reply(node, 'claim_nav', owner='navigation')
    canceled, phases, operations = [], [], []
    reply = {'ready': False, 'response': None}
    future = SimpleNamespace(done=lambda: reply['ready'], result=lambda: reply['response'])
    handle = SimpleNamespace(
        accepted=True,
        get_result_async=lambda: SimpleNamespace(add_done_callback=lambda callback: None),
        cancel_goal_async=lambda: canceled.append(node._relay_uuid) or future)
    node._on_result = MethodType(CoveragePlanner._on_result, node)
    node.goal_handle = None
    if accepted:
        CoveragePlanner._on_goal_response(node, SimpleNamespace(result=lambda: handle))
    node._relay_bumper_escape = node._relay_contact_cancel = node._relay_contact_wall = None
    node._relay_nav_contact = MethodType(CoveragePlanner._relay_nav_contact, node)
    node._relay_phase = MethodType(CoveragePlanner._relay_phase, node)
    node._relay_phase_pub = SimpleNamespace(publish=lambda message: phases.append(message.data))
    node._begin_bumper_escape = MethodType(CoveragePlanner._begin_bumper_escape, node)
    node._relay_request = operations.append
    node._relay_last_stage = None
    node._relay_stage_wall = 0.0
    node._relay_snapshot = {}
    node._relay_retry_at = 0.0
    node._relay_stop_future = None
    node._relay_control = SimpleNamespace(service_is_ready=lambda: False)
    node._relay_fail = MethodType(CoveragePlanner._relay_fail, node)
    node.plan_started = True
    node.goal_timeout = 30.0
    node.escape_until = None
    node.escapes_done = 0
    node.max_escapes = 25
    node.escape_sec = 2.5
    node._bumper_held_long = lambda: True
    node._bumper_fresh = MethodType(CoveragePlanner._bumper_fresh, node)
    node._escape_angular = lambda: -0.6
    node.wedge_zones = []
    node.wedge_avoid_radius = 0.45
    node.get_logger = lambda: SimpleNamespace(
        info=lambda message: None, warn=lambda message: None, error=lambda message: None)
    return node, sent, canceled, phases, operations, reply, future, handle


def test_relay_contact_waits_for_terminal_release_and_sweep_ack_before_escape(tmp_path):
    node, sent, canceled, phases, operations, reply, _, _ = contact_relay_planner(tmp_path)
    assert CoveragePlanner._relay_tick(node) is True
    assert canceled == [node._relay_uuid] and phases[-1] == 'TRANSIT_CANCEL'
    assert node.escape_until is None and sent[-1] == (0.0, 0.0)
    assert CoveragePlanner._relay_tick(node) is True
    assert len(canceled) == 1  # no repeated cancel while the future is pending
    goal = SimpleNamespace(goal_id=SimpleNamespace(uuid=list(bytes.fromhex(node._relay_uuid))))
    reply.update(ready=True, response=SimpleNamespace(return_code=0, goals_canceling=[goal]))
    assert CoveragePlanner._relay_tick(node) is True
    assert node._relay_stage == 'nav' and node.escape_until is None
    node._bumper_held_long = lambda: False
    node._escape_angular = lambda: 0.6  # retain the direction actually observed at contact
    CoveragePlanner._on_result(node, SimpleNamespace(result=lambda: SimpleNamespace(status=5)))
    assert node._relay_journal.read()['outcome'] == 'canceled'
    node._relay_snapshot = {'last_output': [0.0, 0.0],
                            'motion_inputs': {'navigation': {'zero': True}}}
    assert CoveragePlanner._relay_tick(node) is True
    assert operations == ['release_nav'] and node.escape_until is None
    relay_reply(node, 'release_nav')
    assert node._relay_stage == 'claim_sweep' and node.escape_until is None
    relay_reply(node, 'claim_sweep', owner='reactive_sweep')
    assert CoveragePlanner._relay_tick(node) is True
    assert node.escape_until is not None and node._escape_az == -0.6
    assert node.wedge_zones == [(0.0, 0.0, 0.45)] and node.escapes_done == 1
    assert node._relay_bumper_escape is None and sent[-1] == (0.0, 0.0)


@pytest.mark.parametrize('failure', [
    'wrong_uuid', 'duplicate_uuid', 'rejected', 'timeout', 'transport',
])
def test_relay_contact_uncertain_cancel_latches_fault(tmp_path, monkeypatch, failure):
    node, sent, _, _, _, reply, _, _ = contact_relay_planner(tmp_path)
    wall = [1.0]
    monkeypatch.setattr('oomwoo_coverage.coverage_planner_node.time.monotonic', lambda: wall[0])
    assert node._relay_nav_contact() is True
    if failure == 'timeout':
        wall[0] = 4.01
    elif failure == 'rejected':
        reply.update(ready=True, response=SimpleNamespace(return_code=1, goals_canceling=[]))
    elif failure in ('wrong_uuid', 'duplicate_uuid'):
        uid = [255] * 16 if failure == 'wrong_uuid' else list(bytes.fromhex(node._relay_uuid))
        goal = SimpleNamespace(goal_id=SimpleNamespace(uuid=uid))
        reply.update(ready=True, response=SimpleNamespace(
            return_code=0, goals_canceling=[goal] * (2 if failure == 'duplicate_uuid' else 1)))
    else:
        def broken():
            raise RuntimeError('disconnected cancellation service')
        node._relay_contact_cancel = SimpleNamespace(done=lambda: True, result=broken)
    assert node._relay_nav_contact() is True
    assert node._relay_stage == 'fault' and node.escape_until is None
    assert sent[-1] == (0.0, 0.0)


def test_contact_cancel_send_failure_stays_faulted_without_a_request_loop(tmp_path):
    node, sent, _, _, _, _, _, handle = contact_relay_planner(tmp_path)
    attempts, stop_attempts = [], []

    def broken():
        attempts.append(True)
        raise RuntimeError('disconnected cancel request')

    handle.cancel_goal_async = broken

    def broken_stop(request):
        stop_attempts.append(request.operation)
        raise RuntimeError('disconnected STOP request')

    node._relay_control_type = SimpleNamespace(Request=SimpleNamespace)
    node._relay_control = SimpleNamespace(service_is_ready=lambda: True, call_async=broken_stop)
    assert node._relay_nav_contact() is True
    assert node._relay_stage == 'fault' and node.escape_until is None
    epoch = node._relay_epoch
    count = len(attempts)
    node._relay_fail('later transport report')
    assert len(attempts) == count and node._relay_epoch == epoch
    assert stop_attempts == ['stop'] and node._relay_stop_future is False
    assert node._relay_fault.startswith('contact cancellation send failed')
    assert sent[-1] == (0.0, 0.0)


def test_relay_contact_before_acceptance_cancels_the_accepted_exact_goal(tmp_path):
    node, _, canceled, _, _, _, future, handle = contact_relay_planner(tmp_path, accepted=False)
    assert node._relay_nav_contact() is True
    assert canceled == [] and node.cancel_on_accept
    CoveragePlanner._on_goal_response(node, SimpleNamespace(result=lambda: handle))
    assert canceled == [node._relay_uuid] and node._relay_contact_cancel is future
    assert node.escape_until is None


def test_contact_does_not_turn_an_actual_aborted_goal_into_recovery_permission(tmp_path):
    node, sent, _, _, _, _, _, _ = contact_relay_planner(tmp_path)
    node._relay_nav_contact()
    node.next_send = node.get_clock().now()
    progress = (node.wp_index, node.wp_retries, node.consecutive_skips, node.next_send)
    CoveragePlanner._on_result(node, SimpleNamespace(result=lambda: SimpleNamespace(status=6)))
    assert node._relay_journal.read()['outcome'] == 'aborted'
    assert node._relay_stage == 'fault' and node.escape_until is None
    assert sent[-1] == (0.0, 0.0)
    assert (node.wp_index, node.wp_retries, node.consecutive_skips, node.next_send) == progress
    assert node.goal_handle is None and not node.awaiting and node.goal_deadline is None


@pytest.mark.parametrize('status,outcome', [(4, 'succeeded'), (5, 'canceled'), (6, 'aborted')])
def test_faulted_nav_terminal_preserves_progress_and_fault_until_reconciliation(
        tmp_path, status, outcome):
    node, sent, canceled, phases, _, _, _, _ = contact_relay_planner(tmp_path)
    node.next_send = node.get_clock().now()
    progress = (node.wp_index, node.wp_retries, node.consecutive_skips, node.next_send)
    node._relay_fail('original lease failure')
    epoch = node._relay_epoch
    CoveragePlanner._on_result(node, SimpleNamespace(
        result=lambda: SimpleNamespace(status=status)))
    assert node._relay_journal.read()['outcome'] == outcome
    assert node._relay_fault == 'original lease failure' and node._relay_epoch == epoch
    assert node._relay_stage == 'fault'
    assert (node.wp_index, node.wp_retries, node.consecutive_skips, node.next_send) == progress
    assert node.goal_handle is None and not node.awaiting and node.goal_deadline is None
    assert CoveragePlanner._relay_tick(node) is True
    assert phases[-1] == 'FAULT' and sent[-1] == (0.0, 0.0)
    assert len(canceled) == 1


def test_cancel_already_terminal_response_waits_for_the_actual_result(tmp_path):
    node, sent, _, _, _, reply, _, _ = contact_relay_planner(tmp_path)
    assert node._relay_nav_contact() is True
    reply.update(ready=True, response=SimpleNamespace(return_code=3, goals_canceling=[]))
    assert node._relay_nav_contact() is True
    assert node._relay_stage == 'nav' and node.escape_until is None
    assert node._relay_journal.read()['state'] == 'accepted'
    CoveragePlanner._on_result(node, SimpleNamespace(result=lambda: SimpleNamespace(status=4)))
    assert node._relay_stage == 'release_nav' and node.escape_until is None
    assert sent[-1] == (0.0, 0.0)


@pytest.mark.parametrize('occupancy', [100, -1])
def test_keepout_ordinary_obstacle_corners_respect_the_robot_disk(occupancy):
    node, faults = keepout_driver()
    width = node.map_msg.info.width
    node.map_msg.data[32 * width + 32] = occupancy
    mask = keepout_grid()
    mask.data[32 * width + 32] = 0
    mask.data[8 * width + 8] = 100
    node._on_keepout(mask)
    # Cell 35's centre is only .125 m from each edge of obstacle cell 32.
    # Its disk intersects that corner, although cardinal inflation misses it.
    assert np.hypot(.125, .125) < node.robot_radius
    assert not node.free_mask[35, 35]
    assert node.free_mask[43, 43]
    assert faults == []


def test_unmasked_obstacle_inflation_retains_its_cardinal_contract():
    node, faults = keepout_driver()
    node._keepout_required = False
    node.map_msg.data[32 * node.map_msg.info.width + 32] = 100
    node._build_masks()
    assert node.free_mask[35, 35]
    assert not node.free_mask[32, 35]
    assert faults == []


def transit_budget_driver(speed=.14):
    node, _, now = timed_reactive_driver()
    node.nav_linear_speed = speed
    node.robot_xy = (0.0, 0.0)
    node._robot_pose = lambda: (*node.robot_xy, 0.0)
    node.cached_poses = [row_target(6.0)]
    node._dispatch_goal(node.cached_poses[0])
    canceled = []
    node.goal_handle = SimpleNamespace(cancel_goal_async=lambda: canceled.append(True))
    return node, now, canceled


def test_goal_watchdog_uses_fresh_tf_when_amcl_cache_stays_unchanged():
    node, now, canceled = transit_budget_driver()
    target = row_target(1.5250000465661286)
    target.pose.position.y = -2.52499999217689
    node.cached_poses = [target]
    # These fresh TF positions were observed in the full-room canceled goal.
    # Hold the separate AMCL cache to discriminate the source-selection bug;
    # the historical cache value itself was not retained in that recording.
    current = [(-0.9626235757767924, -2.5448281134335393, 0.0)]
    node.robot_xy = current[0][:2]
    node._robot_pose = lambda: current[0]
    node._dispatch_goal(target)
    CoveragePlanner._goal_watchdog(node)
    now[0] = 7.0
    current[0] = (-0.6879962285218605, -2.271197422214184, 0.0)
    CoveragePlanner._goal_watchdog(node)
    assert not canceled
    assert node._goal_best_dist == pytest.approx(2.2275026954131816)
    assert node._elapsed(node._goal_progress_t) == pytest.approx(0.0)


def test_goal_watchdog_does_not_credit_amcl_changes_when_tf_is_stationary():
    node, now, canceled = transit_budget_driver()
    node._robot_pose = lambda: (0.0, 0.0, 0.0)
    CoveragePlanner._goal_watchdog(node)
    now[0] = 7.1
    node.robot_xy = (0.2, 0.0)
    CoveragePlanner._goal_watchdog(node)
    assert canceled == [True]


def test_goal_watchdog_missing_tf_does_not_renew_progress_or_remove_deadline():
    node, now, canceled = transit_budget_driver()
    node._robot_pose = lambda: None
    node.robot_xy = (5.9, 0.0)  # A cached near-goal pose is not current evidence.
    original_progress = node._goal_progress_t
    now[0] = 2.0
    CoveragePlanner._goal_watchdog(node)
    assert node._goal_progress_t == original_progress
    assert not canceled
    now[0] = node.goal_deadline.nanoseconds * 1e-9 + node._nav_goal_budget_s + 0.1
    CoveragePlanner._goal_watchdog(node)
    assert canceled == [True]


@pytest.mark.parametrize('stamp_s, progress_expected', [
    (1.5, True),       # inclusive 500 ms source-age limit
    (1.499, False),    # outside that same existing limit
    (2.1, False),      # future TF cannot justify progress
    (0.0, False),
    (None, False),
])
def test_goal_watchdog_uses_actual_tf_age_admission(stamp_s, progress_expected):
    node, now, canceled = transit_budget_driver()
    node.global_frame, node.base_frame = 'map', 'base_link'
    node.tf_buffer = Buffer()
    node._robot_pose = MethodType(CoveragePlanner._robot_pose, node)
    node.robot_xy = (5.9, 0.0)
    if stamp_s is not None:
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = 'map', 'base_link'
        transform.header.stamp = Time(seconds=stamp_s).to_msg()
        transform.transform.translation.x = 5.9
        transform.transform.rotation.w = 1.0
        node.tf_buffer.set_transform(transform, 'watchdog_source_age_test')
    original_progress = node._goal_progress_t
    now[0] = 2.0
    CoveragePlanner._goal_watchdog(node)
    assert (node._goal_progress_t != original_progress) is progress_expected
    assert not canceled


def test_transit_budget_allows_long_trip_that_is_still_progressing():
    node, now, canceled = transit_budget_driver()
    now[0] = 32.0
    node.robot_xy = (4.0, 0.0)
    CoveragePlanner._goal_watchdog(node)
    assert not canceled
    assert node.awaiting
    assert node.goal_timeout == 30.0


def test_transit_budget_keeps_absolute_deadline_even_with_fresh_progress():
    node, now, canceled = transit_budget_driver()
    assert node._nav_goal_budget_s == pytest.approx(6.0 / .14 + 6.0)
    now[0] += node._nav_goal_budget_s + .1
    node.robot_xy = (5.0, 0.0)
    CoveragePlanner._goal_watchdog(node)
    assert canceled == [True]


def test_transit_budget_does_not_extend_no_progress_window():
    node, now, canceled = transit_budget_driver()
    now[0] = 2.0
    node.robot_xy = (.1, 0.0)
    CoveragePlanner._goal_watchdog(node)
    assert not canceled
    now[0] = 8.1
    CoveragePlanner._goal_watchdog(node)
    assert canceled == [True]


def test_transit_budget_zero_speed_preserves_legacy_absolute_deadline():
    node, now, canceled = transit_budget_driver(speed=0.0)
    now[0] = 32.0
    node.robot_xy = (4.0, 0.0)
    CoveragePlanner._goal_watchdog(node)
    assert canceled == [True]
    assert node._nav_goal_budget_s == 30.0


def test_transit_budget_recomputed_for_each_dispatched_goal():
    node, now, canceled = transit_budget_driver()
    assert node._nav_goal_budget_s > 30.0
    node.cached_poses = [row_target(1.0)]
    node._dispatch_goal(node.cached_poses[0])
    assert node._nav_goal_budget_s == 30.0
    now[0] = 32.0
    node.robot_xy = (.5, 0.0)
    CoveragePlanner._goal_watchdog(node)
    assert canceled == [True]


def test_transit_budget_slow_speed_remains_capped():
    node, now, canceled = transit_budget_driver(speed=.001)
    assert node._nav_goal_budget_s == 60.0
    now[0] = 62.0
    node.robot_xy = (1.0, 0.0)
    CoveragePlanner._goal_watchdog(node)
    assert canceled == [True]


@pytest.mark.parametrize('speed', ['-0.14', '.nan', '.inf'])
def test_transit_budget_rejects_invalid_configured_speed(speed):
    import rclpy

    rclpy.init(args=['--ros-args', '-p', 'nav_linear_speed:=' + speed])
    node = CoveragePlanner.__new__(CoveragePlanner)
    try:
        with pytest.raises(ValueError, match='nav_linear_speed'):
            node.__init__()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_sweep_skips_a_meter_credited_disk_next_to_the_wall():
    from oomwoo_sim_support.coverage_meter_node import CoverageMeter, _stamp_disk

    grid = OccupancyGrid()
    grid.info.width, grid.info.height, grid.info.resolution = 40, 30, .05
    free = np.ones((30, 40), dtype=bool)
    free[:4] = False
    meter = SimpleNamespace(
        info=grid.info, free=free, reachable=None, total_reachable=0,
        robot_radius=.175, cleaning_radius=.20, edge_margin=.15,
        get_logger=lambda: SimpleNamespace(info=lambda message: None))
    CoverageMeter._ensure_reachable(meter, 20, 8)
    credited = np.zeros_like(free)
    _stamp_disk(credited, meter.reachable, 20, 8, 4)
    footprint = np.zeros_like(free)
    _stamp_disk(footprint, np.ones_like(free), 20, 8, 4)
    assert credited.sum() / footprint.sum() > .6
    node = SimpleNamespace(
        map_msg=grid, covered_grid=credited.astype(np.int8) * 100,
        cleaning_radius=.20, gapfill_passes=0)
    target = row_target(20.5 * .05)
    target.pose.position.y = 8.5 * .05
    assert CoveragePlanner._covered_at(node, target)


def test_sweep_does_not_count_clean_square_corners_outside_its_disk():
    from oomwoo_sim_support.coverage_meter_node import _stamp_disk

    grid = OccupancyGrid()
    grid.info.width = grid.info.height = 21
    grid.info.resolution = .05
    disk = np.zeros((21, 21), dtype=bool)
    _stamp_disk(disk, np.ones_like(disk), 10, 10, 4)
    covered = ~disk
    inside_old_square = np.zeros_like(disk)
    inside_old_square[6:14, 6:14] = True
    for y, x in np.argwhere(disk & inside_old_square)[:24]:
        covered[y, x] = True
    assert np.count_nonzero(covered & disk) / disk.sum() < .6
    node = SimpleNamespace(
        map_msg=grid, covered_grid=covered.astype(np.int8) * 100,
        cleaning_radius=.20, gapfill_passes=0)
    target = row_target(10.5 * .05)
    target.pose.position.y = 10.5 * .05
    assert not CoveragePlanner._covered_at(node, target)


def test_gapfill_nav_success_finishes_uncredited_cell_before_leaving():
    from oomwoo_sim_support.coverage_meter_node import _stamp_disk

    node = gapfill_with_small_unclean_hole()
    node.exec_mode = 'reactive'
    target = node.cached_poses[0].pose.position
    node._robot_pose = lambda: (target.x + .20, target.y, float(np.pi))
    node.awaiting = True
    CoveragePlanner._on_result(node, SimpleNamespace(
        result=lambda: SimpleNamespace(status=4)))
    assert not node.awaiting
    assert not node._covered_at(node.cached_poses[0])
    assert node.wp_index == 0, 'Nav2 tolerance was mistaken for cleaning credit'
    node._advance_to_target = MethodType(CoveragePlanner._advance_to_target, node)
    commands = []
    node.cmd_pub.publish = lambda tw: commands.append((tw.linear.x, tw.angular.z))
    CoveragePlanner._tick_reactive(node)
    assert commands[-1][0] > 0, 'existing reactive drive must finish the short approach'
    assert node.wp_index == 0 and not node.finished
    # Feed the unchanged meter disk from a closer virtual pose, rather than
    # inventing credit because Nav2 succeeded. This is geometry, not a plant.
    stamped = np.zeros_like(node.free_mask)
    _stamp_disk(stamped, node.free_mask, 8, 6, 4)
    node.covered_grid[stamped] = 100
    CoveragePlanner._tick_reactive(node)
    assert commands[-1] == (0.0, 0.0) and node.finished


@pytest.mark.parametrize('mode,passes,clean', [
    ('reactive', 0, False), ('nav2', 1, False), ('reactive', 1, True),
])
def test_nav_success_keeps_existing_main_sweep_and_credited_gap_progress(mode, passes, clean):
    node = gapfill_with_small_unclean_hole()
    node.exec_mode, node.gapfill_passes = mode, passes
    if clean:
        node.covered_grid[6, 6] = 100
    CoveragePlanner._on_result(node, SimpleNamespace(
        result=lambda: SimpleNamespace(status=4)))
    assert node.wp_index == 1 and node.wp_retries == 0


def test_gapfill_nav_success_keeps_terminal_lease_release_before_short_approach(tmp_path):
    node, _, _, _, _, _, _, _ = contact_relay_planner(tmp_path)
    node.exec_mode, node.gapfill_passes, node.ext_ratio = 'reactive', 1, .95
    node._covered_at = lambda pose: False
    CoveragePlanner._on_result(node, SimpleNamespace(
        result=lambda: SimpleNamespace(status=4)))
    assert node._relay_stage == 'release_nav'
    assert node._relay_journal.read()['state'] == 'terminal'
    assert node.wp_index == 0 and not node.awaiting
