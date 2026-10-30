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
"""Coverage scorer input coherence, launch wiring and bounded termination."""

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


@pytest.mark.parametrize('score_code,child_code,child_delay,expected', [
    (0, 139, 0.4, 3),
    (0, 0, 0.4, 0),
    (1, 0, 0.4, 1),
    (2, 0, 0.4, 2),
    (0, 139, 0.01, 3),
    (139, 0, 0.4, 3),
])
def test_wrapper_retains_owned_process_failures(
        tmp_path, monkeypatch, score_code, child_code, child_delay, expected):
    """Exercise the embedded launcher with real child exits and Launch events."""
    from pathlib import Path
    import subprocess
    import sys

    import ament_index_python.packages
    import launch.actions
    from launch.actions import EmitEvent, ExecuteProcess, GroupAction, RegisterEventHandler
    from launch.event_handlers import OnProcessExit
    from launch.events import Shutdown
    import launch_ros.actions

    root = Path(__file__).resolve().parents[3]
    wrapper = (root / 'deploy/run_coverage_regression.sh').read_text()
    embedded = wrapper.split("<<'PY'\n", 1)[1].split('\nPY\n', 1)[0]
    scorer = ExecuteProcess(
        cmd=[sys.executable, '-c',
             f'import time; time.sleep(.05); raise SystemExit({score_code})'],
        name='fixture_scorer', output='screen')
    child = ExecuteProcess(
        cmd=[sys.executable, '-c',
             f'import time; time.sleep({child_delay}); raise SystemExit({child_code})'],
        name='fixture_required_server', output='screen')
    stack = GroupAction(actions=[
        RegisterEventHandler(OnProcessExit(target_action=child, on_exit=[
            EmitEvent(event=Shutdown(reason='owned required server exited'))])),
        child,
    ])
    monkeypatch.setattr(ament_index_python.packages, 'get_package_share_directory',
                        lambda _: str(tmp_path))

    def scorer_node(**kwargs):
        params = kwargs['parameters'][0]
        assert params['robot_radius'] == 0.1745
        assert params['cleaning_radius'] == 0.20
        return scorer

    def included_stack(*args, **kwargs):
        arguments = dict(kwargs['launch_arguments'])
        assert arguments['with_meter'] == 'false'
        assert arguments['robot_radius'] == '0.30'
        return stack

    monkeypatch.setattr(launch_ros.actions, 'Node', scorer_node)
    monkeypatch.setattr(launch.actions, 'IncludeLaunchDescription', included_stack)
    monkeypatch.setattr(subprocess, 'run', lambda *a, **k: SimpleNamespace(
        returncode=0, stdout='data: true\n', stderr=''))
    monkeypatch.setattr(sys, 'argv', ['coverage-wrapper', str(tmp_path / 'report.json'),
                                      'with_meter:=true robot_radius:=0.30'])
    with pytest.raises(SystemExit) as outcome:
        exec(compile(embedded, 'run_coverage_regression.sh:embedded', 'exec'), {})
    assert outcome.value.code == expected


def test_phase_trace_preserves_original_pose_sources_across_contact_escape():
    from nav2_msgs.action import NavigateToPose

    node = SimpleNamespace(
        motion_samples={'status': [], 'phase': [], 'feedback': []},
        _motion_fault=False, _sim_now=lambda: 12.0)
    feedback = NavigateToPose.Impl.FeedbackMessage()
    feedback.goal_id.uuid = list(bytes.fromhex('01' * 16))
    pose = feedback.feedback.current_pose
    pose.header.frame_id = 'map'
    pose.header.stamp.sec = 11
    pose.header.stamp.nanosec = 500000000
    pose.pose.position.x = 3.56
    pose.pose.orientation.w = 1.0
    runner.CoverageRunner._audit_feedback(node, feedback)
    node._keepout_audit_previous = (3.40, 3.18, 11980000000)
    runner.CoverageRunner._audit_phase(node, SimpleNamespace(data='ESCAPE'))
    escape = node.motion_samples['phase'][-1]
    assert escape['truth_position'] == {
        'frame': 'map', 'position': [3.40, 3.18], 'stamp_ns': 11980000000}
    assert escape['nav_feedback']['uuid'] == '01' * 16
    assert escape['nav_feedback']['stamp_ns'] == 11500000000
    assert escape['nav_feedback']['position'][0] == 3.56
    assert escape['nav_feedback']['frame'] == 'map'
    pose.pose.position.x = 3.20
    pose.header.stamp.sec = 12
    runner.CoverageRunner._audit_feedback(node, feedback)
    node._keepout_audit_previous = (3.16, 3.18, 12980000000)
    runner.CoverageRunner._audit_phase(node, SimpleNamespace(data='TRANSIT'))
    assert escape['nav_feedback']['position'][0] == 3.56
    assert escape['truth_position']['position'][0] == 3.40
    assert node.motion_samples['phase'][-1]['nav_feedback']['position'][0] == 3.20
    for i in range(130):
        runner.CoverageRunner._audit_phase(node, SimpleNamespace(data=f'phase{i}'))
    assert len(node.motion_samples['phase']) == 128


def test_phase_trace_does_not_invent_missing_pose_evidence():
    node = SimpleNamespace(
        motion_samples={'phase': []}, _motion_fault=False, _sim_now=lambda: 1.0)
    runner.CoverageRunner._audit_phase(node, SimpleNamespace(data='FAULT'))
    assert node._motion_fault
    entry = node.motion_samples['phase'][0]
    assert 'truth_position' not in entry and 'nav_feedback' not in entry


@pytest.fixture
def fused_meter():
    runner.rclpy.init(args=[])
    node = None
    try:
        node = runner.CoverageRunner()
        yield node
    finally:
        if node is not None:
            node.destroy_node()
        runner.rclpy.shutdown()


def measured_frame(node, covered_cells, efficiency):
    import numpy as np
    from nav_msgs.msg import MapMetaData

    node.info = MapMetaData()
    node.info.width, node.info.height, node.info.resolution = 50, 20, 0.05
    node.total_reachable = 1000
    node.reachable = np.ones((20, 50), dtype=bool)
    node.covered = np.zeros((20, 50), dtype=bool)
    node.covered.flat[:covered_cells] = True
    node.path_len = node._ideal_path_len() / efficiency
    node.revisit_len = 0.0


@pytest.mark.parametrize('previous,current,expected', [(.805, .799, 1), (.90, .81, 0)])
def test_grade_uses_the_same_measured_frame(fused_meter, score, previous, current, expected):
    measured_frame(fused_meter, 901, current)
    fused_meter.coverage = .898
    fused_meter.efficiency = previous
    snapshot = fused_meter._report()
    assert snapshot == pytest.approx((.901, current, 0.0))
    assert fused_meter.coverage == pytest.approx(snapshot[0])
    assert fused_meter.efficiency == pytest.approx(snapshot[1])

    def update(node, tick):
        node.sweep_started = True
        node.coverage, node.efficiency, node.revisit_ratio = snapshot
        node.sweep_complete = tick >= 4

    code, report, _ = score(wall=10.0, update=update)
    assert code == expected
    assert report['efficiency_at_target'] == current


def test_fused_runner_has_one_truth_and_active_input_and_no_scalar_metric_inputs(fused_meter):
    topics = [subscription.topic_name for subscription in fused_meter.subscriptions]
    assert topics.count('/ground_truth/pose') == 1
    assert topics.count('/coverage_planner/cleaning_active') == 1
    assert not set(topics) & {
        '/coverage_meter/ratio', '/coverage_meter/efficiency',
        '/coverage_meter/revisit_ratio', '/coverage_meter/sim_unstable'}
    assert fused_meter.get_name() == 'coverage_meter'
    pubs = [publisher.topic_name for publisher in fused_meter.publishers]
    assert pubs.count('/coverage_meter/ratio') == 1
    assert pubs.count('/coverage_meter/efficiency') == 1


def test_active_callback_drives_both_existing_meter_and_scorer_states(fused_meter):
    from std_msgs.msg import Bool

    assert not fused_meter.job_active and not fused_meter.sweep_started
    fused_meter._on_active(Bool(data=True))
    assert fused_meter.job_active and fused_meter.sweep_started
    fused_meter._on_active(Bool(data=False))
    assert fused_meter.sweep_complete
    # The original meter keeps accounting through the stopping/grace interval.
    assert fused_meter.job_active


@pytest.mark.parametrize('crossing,later,expected', [(.801, .799, 0), (.799, .801, 1)])
def test_completion_grace_keeps_first_paired_target_crossing(score, crossing, later, expected):
    def update(node, tick):
        node.sweep_started = True
        node.sweep_complete = True
        node.coverage = .899 if tick == 1 else .901
        node.efficiency = .805 if tick == 1 else crossing if tick == 2 else later

    code, report, _ = score(wall=10.0, update=update)
    assert code == expected
    assert report['efficiency_at_target'] == crossing
    assert report['efficiency_final'] == later
    assert report['end_reason'] == 'sweep_complete'


def test_regression_launch_can_disable_the_separate_meter():
    import importlib.util
    from pathlib import Path

    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch.conditions import IfCondition
    from launch.utilities import perform_substitutions
    from launch_ros.actions import Node

    path = Path(__file__).resolve().parents[1] / 'launch/coverage_regression.launch.py'
    spec = importlib.util.spec_from_file_location('coverage_regression_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    argument = next(action for action in description.entities
                    if isinstance(action, DeclareLaunchArgument) and action.name == 'with_meter')
    context = LaunchContext()
    assert perform_substitutions(context, argument.default_value) == 'true'
    meters = [action for action in description.entities
              if isinstance(action, Node) and isinstance(action.condition, IfCondition)]
    assert len(meters) == 1
    context.launch_configurations['with_meter'] = 'false'
    assert not meters[0].condition.evaluate(context)
    context.launch_configurations['with_meter'] = 'true'
    assert meters[0].condition.evaluate(context)


def test_motion_fault_during_completion_grace_cannot_pass(score):
    def update(node, tick):
        node.sweep_started = True
        node.sweep_complete = True
        node.coverage = .95
        node.efficiency = .85
        node._motion_fault = tick >= 2

    code, report, _ = score(wall=10.0, update=update)
    assert code == 1
    assert report['end_reason'] == 'motion_fault'
    assert not report['pass']


def test_regression_launch_passes_the_configured_nav_speed_to_planner(monkeypatch):
    import importlib.util
    from pathlib import Path

    from launch import LaunchContext

    path = Path(__file__).resolve().parents[1] / 'launch/coverage_regression.launch.py'
    spec = importlib.util.spec_from_file_location('coverage_speed_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    real_node = module.Node
    planner_parameters = []

    def record_node(**kwargs):
        if kwargs['executable'] == 'coverage_planner':
            planner_parameters.extend(kwargs['parameters'])
        return real_node(**kwargs)

    monkeypatch.setattr(module, 'Node', record_node)
    module.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations['nav_linear_speed'] = '0.14'
    assert planner_parameters[0]['nav_linear_speed'].evaluate(context) == .14


@pytest.mark.parametrize('mode,expected_code,measurement_valid', [
    ('crossing', 1, True),
    ('late_crossing', 1, True),
    ('late_stale', 2, False),
    ('stale', 2, False),
    ('gap', 2, False),
    ('bad_mask', 2, False),
    ('empty_trace', 2, False),
    ('clear', 0, True),
    ('no_mask', 0, True),
])
def test_masked_safety_audit_participates_in_existing_verdict(
        score, mode, expected_code, measurement_valid):
    """Positive coverage cannot conceal a measured crossing or untrusted audit."""
    audit, mask, now = truth_audit()

    def update(node, tick):
        node.sweep_started = True
        node.sweep_complete = True
        node.coverage = 0.95
        node.efficiency = 0.85
        if tick == 2 and mode in ('late_crossing', 'late_stale'):
            node._audit_keepout_truth(truth_pose(
                1.4 if mode == 'late_crossing' else -.25,
                10_150_000_000 if mode == 'late_crossing' else 8_000_000_000))
        if tick != 1:
            return
        node.motion_audit = True
        node.motion_graph = {}
        node.motion_samples = {'phase': [], 'status': [], 'feedback': []}
        node._motion_status_count = 0
        node._audit_graph = lambda: None
        node.get_clock = audit.get_clock
        node._keepout_audit = audit._keepout_audit
        node._keepout_audit_mask = None
        node._keepout_audit_previous = None
        node._keepout_audit_bbox = None
        for name in ('_audit_keepout_mask', '_audit_keepout_truth',
                     '_audit_keepout_error', '_audit_keepout_result'):
            setattr(node, name, MethodType(
                getattr(runner.CoverageRunner, name), node))
        if mode == 'no_mask':
            return
        if mode == 'bad_mask':
            mask.data[0] = 50
        node._audit_keepout_mask(mask)
        if mode == 'empty_trace' or mode == 'bad_mask':
            return
        if mode == 'stale':
            node._audit_keepout_truth(truth_pose(-.3, 8_000_000_000))
        elif mode == 'gap':
            now[0] = 10_500_000_000
            node._audit_keepout_truth(truth_pose(-.3, 10_000_000_000))
            node._audit_keepout_truth(truth_pose(-.2, 10_400_000_000))
        else:
            node._audit_keepout_truth(truth_pose(
                .2 if mode == 'crossing' else -.3, 10_000_000_000))
            node._audit_keepout_truth(truth_pose(
                1.4 if mode == 'crossing' else -.25, 10_100_000_000))

    code, report, _ = score(wall=10.0, update=update)
    if mode in ('crossing', 'late_crossing'):
        assert report['motion_audit']['keepout']['footprint_crossings'] == 1
    assert code == expected_code, report
    assert report['measurement_valid'] is measurement_valid
    assert report['pass'] is (expected_code == 0)
