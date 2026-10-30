# Copyright 2026 Víctor Izquierdo González
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
"""Clock and startup bounds of the real coverage scorer, with deterministic input."""

import json
from types import MethodType, SimpleNamespace

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from oomwoo_sim_support import coverage_regression_runner as runner
import pytest


@pytest.fixture
def score(tmp_path, monkeypatch):
    def run(*, clock=True, started=False, wall=2.0, dead=5.0,
            plateau=20.0, update=None):
        ticks = [0]
        elapsed = [0.0]
        sim = [0.0]
        node = SimpleNamespace(
            cov_target=0.90, eff_target=0.80, max_t=20.0,
            max_wall=wall, clock_dead_s=dead, plateau_s=plateau,
            plateau_eps=0.005, coverage=0.0, efficiency=0.0,
            revisit_ratio=0.0, best=0.0, sim_unstable=False,
            measurement_invalid=False, target_crossed=False,
            eff_at_target=0.0, t_to_target=0.0,
            sweep_started=started, sweep_complete=False,
            report_path=str(tmp_path / 'report.json'),
            _sim_now=lambda: sim[0],
            get_logger=lambda: SimpleNamespace(
                info=lambda *a: None, warn=lambda *a: None,
                error=lambda *a: None))

        def spin(actual_node, timeout_sec):
            assert actual_node is node
            ticks[0] += 1
            assert ticks[0] < 80, 'scorer exceeded its bounded input schedule'
            elapsed[0] += 0.25
            if clock:
                sim[0] += 0.25
            if update:
                update(node, ticks[0])

        monkeypatch.setattr(runner, 'time', SimpleNamespace(
            monotonic=lambda: elapsed[0],
            time=lambda: pytest.fail('wall clock must not govern a duration')))
        monkeypatch.setattr(runner.rclpy, 'ok', lambda: True)
        monkeypatch.setattr(runner.rclpy, 'spin_once', spin)
        code = runner.CoverageRunner.run(node)
        return code, json.loads((tmp_path / 'report.json').read_text()), elapsed[0]

    return run


def test_missing_clock_is_invalid_during_startup(score):
    code, report, elapsed = score(clock=False, wall=10.0, dead=1.0)
    assert code == 2
    assert report['end_reason'] == 'clock_dead'
    assert report['measurement_valid'] is False
    assert elapsed <= 1.5


def test_no_coverage_cannot_extend_the_total_wall_budget(score):
    code, report, elapsed = score()
    assert code == 1
    assert report['end_reason'] == 'wall_timeout'
    assert report['measurement_valid'] is True
    assert report['coverage'] == 0.0
    assert elapsed <= 2.5


def test_active_sweep_at_zero_coverage_can_plateau(score):
    code, report, elapsed = score(started=True, wall=10.0, plateau=1.0)
    assert code == 1
    assert report['end_reason'] == 'plateau'
    assert elapsed < 2.0


def test_unstable_startup_stops_without_waiting_for_coverage(score):
    def update(node, tick):
        node.sim_unstable = True
    code, report, elapsed = score(update=update)
    assert code == 2
    assert report['end_reason'] == 'sim_unstable'
    assert elapsed == 0.25


def test_completion_grace_respects_remaining_total_wall_budget(score):
    def update(node, tick):
        node.sweep_started = True
        node.coverage = 0.95
        node.efficiency = 0.85
        node.sweep_complete = tick >= 4
    code, report, elapsed = score(update=update)
    assert code == 0
    assert report['end_reason'] == 'sweep_complete'
    assert elapsed <= 2.5


def test_completion_keeps_efficiency_at_first_target_crossing(score):
    def update(node, tick):
        node.sweep_started = True
        node.coverage = 0.91 if tick == 1 else 0.96
        node.efficiency = 0.82 if tick == 1 else 0.70
        node.sweep_complete = tick >= 4
    code, report, elapsed = score(wall=10.0, update=update)
    assert code == 0
    assert report['efficiency_at_target'] == 0.82
    assert report['efficiency_final'] == 0.70
    assert report['coverage'] == 0.96
    assert elapsed >= 4.0


def test_late_unstable_measurement_during_completion_grace_is_invalid(score):
    def update(node, tick):
        node.coverage = 0.95
        node.efficiency = 0.85
        node.sweep_complete = True
        node.sim_unstable = tick >= 2
    code, report, elapsed = score(update=update)
    assert code == 2
    assert report['end_reason'] == 'sim_unstable'
    assert report['pass'] is False


def test_motion_fault_cannot_pass_even_after_coverage_target(score):
    def update(node, tick):
        node.sweep_started = True
        node.coverage = 0.99
        node.efficiency = 0.90
        node._motion_fault = True
    code, report, elapsed = score(update=update)
    assert code == 1
    assert report['pass'] is False and report['end_reason'] == 'motion_fault'
    assert report['measurement_valid'] is True and elapsed == 0.25


def truth_audit():
    """Bind the existing scorer callbacks to a fixed original-stamp clock."""
    now = [10_200_000_000]
    node = SimpleNamespace(
        _keepout_audit_mask=None, _keepout_audit_previous=None,
        _keepout_audit={'poses_before_mask': 0, 'poses': 0, 'segments': 0,
                        'footprint_crossings': 0, 'max_pose_gap_s': 0., 'errors': []},
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=now[0])))
    for name in ('_audit_keepout_mask', '_audit_keepout_truth',
                 '_audit_keepout_error', '_audit_keepout_result'):
        setattr(node, name, MethodType(getattr(runner.CoverageRunner, name), node))
    mask = OccupancyGrid()
    mask.header.frame_id = 'map'
    mask.info.width = mask.info.height = 20
    mask.info.resolution = .25
    mask.info.origin.position.x = mask.info.origin.position.y = -2.5
    mask.info.origin.orientation.w = 1.
    mask.data = [0] * 400
    mask.data[10 * 20 + 13] = 100  # x in[.75,1.0], y in[0,.25]
    return node, mask, now


def truth_pose(x, nanoseconds):
    """Build a ground-truth sample carrying its original ROS stamp."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp.sec = nanoseconds // 1_000_000_000
    pose.header.stamp.nanosec = nanoseconds % 1_000_000_000
    pose.pose.position.x, pose.pose.position.y = x, .125
    return pose


def test_ground_truth_audit_detects_crossing_between_two_clear_endpoints():
    node, mask, _ = truth_audit()
    node._audit_keepout_mask(mask)
    node._audit_keepout_truth(truth_pose(.2, 10_000_000_000))
    node._audit_keepout_truth(truth_pose(1.4, 10_100_000_000))
    result = node._audit_keepout_result()
    assert result['valid'] and result['poses'] == 2
    assert result['footprint_crossings'] == 1


def test_ground_truth_audit_retains_a_clear_motion_segment():
    node, mask, _ = truth_audit()
    node._audit_keepout_mask(mask)
    node._audit_keepout_truth(truth_pose(-.3, 10_000_000_000))
    node._audit_keepout_truth(truth_pose(-.25, 10_050_000_000))
    result = node._audit_keepout_result()
    assert result['valid'] and result['segments'] == 1
    assert result['footprint_crossings'] == 0
    assert result['min_sampled_center_to_mask_bbox_m'] == pytest.approx(1.)


def test_ground_truth_audit_never_certifies_missing_or_empty_mask():
    node, mask, _ = truth_audit()
    node._audit_keepout_truth(truth_pose(0., 10_000_000_000))
    assert not node._audit_keepout_result()['valid']
    mask.data = [0] * 400
    node._audit_keepout_mask(mask)
    assert not node._audit_keepout_result()['valid']


def test_ground_truth_audit_preserves_stale_stamp_instead_of_refreshing_receipt():
    node, mask, _ = truth_audit()
    node._audit_keepout_mask(mask)
    node._audit_keepout_truth(truth_pose(0., 8_000_000_000))
    result = node._audit_keepout_result()
    assert not result['valid'] and result['errors'] and result['poses'] == 0


def test_ground_truth_audit_reports_insufficient_sample_cadence():
    node, mask, now = truth_audit()
    now[0] = 10_500_000_000
    node._audit_keepout_mask(mask)
    node._audit_keepout_truth(truth_pose(-.3, 10_000_000_000))
    node._audit_keepout_truth(truth_pose(-.2, 10_400_000_000))
    result = node._audit_keepout_result()
    assert result['max_pose_gap_s'] == .4
    assert not result['valid']
