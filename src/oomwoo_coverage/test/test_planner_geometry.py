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
    _contiguous_runs,
    _decompose_cells,
    _dilate,
    _flood_fill,
    _nearest_true,
    _pt_in_zones,
    CoveragePlanner,
)
import pytest
from rclpy.clock import ClockType
from rclpy.time import Time
from tf2_ros import Buffer


def reactive_driver():
    """Exercise the actual row tick with a recording publisher and pose source."""
    sent, dispatched = [], []
    target = SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=0.5, y=0.0)))
    node = SimpleNamespace(
        awaiting=False, goal_deadline=None,
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
    node, _, sends = private_relay_planner(tmp_path)
    CoveragePlanner._send_goal_to(node, row_target(2.0))
    relay_reply(node, 'release_motion')
    old_epoch = node._relay_epoch
    node._relay_status_epoch = old_epoch
    node._relay_status_future = SimpleNamespace(done=lambda: True, result=lambda: SimpleNamespace(
        ok=True, status_json='{"owner":"stopped"}'))
    relay_reply(node, 'claim_nav', owner='navigation')
    assert node._relay_epoch > old_epoch
    assert node._relay_owner == 'navigation' and len(sends) == 1


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
    assert node.wedge_zones == [(2.0, 0.0, 0.45)] and node.escapes_done == 1
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
    CoveragePlanner._on_result(node, SimpleNamespace(result=lambda: SimpleNamespace(status=6)))
    assert node._relay_journal.read()['outcome'] == 'aborted'
    assert node._relay_stage == 'fault' and node.escape_until is None
    assert sent[-1] == (0.0, 0.0)


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
