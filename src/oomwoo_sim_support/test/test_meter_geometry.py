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
"""
Unit tests for the coverage meter's grid geometry.

The load-bearing property here is DENOMINATOR MONOTONICITY: the serviceable
denominator built from a smaller robot radius must never be smaller than the
one built from a larger radius. This is the regression test for the
meter/planner radius-coupling review finding — with the meter pinned to the
true body radius, no planner clearance choice can shrink the denominator and
inflate the score.
"""

import numpy as np

from oomwoo_sim_support.coverage_meter_node import (
    _dilate,
    _flood_fill,
    _nearest_free,
    _stamp_disk,
)

import pytest


def _room_with_alcove():
    """Build a walled room with a 3-cell-wide slot only a slim robot enters."""
    free = np.zeros((30, 30), dtype=bool)
    free[1:29, 1:29] = True                 # room interior, 1-cell wall ring
    free[10:20, 10:20] = False              # solid block in the middle
    free[13:16, 10:20] = True               # 3-cell-wide slot through the block
    return free


def _denominator(free, r_robot, r_clean, edge_margin):
    """Mirror coverage_meter_node._ensure_reachable's serviceable-cell formula."""
    start = _nearest_free(free, 2, 2)
    reach = _flood_fill(free, start)
    drivable = reach & ~_dilate(~free, r_robot)
    cleanable = _dilate(drivable, r_clean) & reach
    if edge_margin > 0:
        cleanable &= ~_dilate(~free, edge_margin)
    return cleanable


def test_denominator_monotone_in_robot_radius():
    """A more timid robot radius must never shrink the denominator."""
    free = _room_with_alcove()
    prev = None
    for r in (6, 5, 4, 3, 2, 1):            # shrinking robot radius
        cur = _denominator(free, r_robot=r, r_clean=4, edge_margin=3)
        if prev is not None:
            # strict superset-or-equal, cell by cell — not just the count
            assert bool(np.all(cur[prev])), \
                'shrinking robot_radius must never remove serviceable cells'
            assert int(cur.sum()) >= int(prev.sum())
        prev = cur


def test_denominator_alcove_counts_only_for_slim_robot():
    """The 3-cell slot is serviceable for a slim robot and unreachable fat."""
    free = _room_with_alcove()
    slim = _denominator(free, r_robot=1, r_clean=2, edge_margin=0)
    fat = _denominator(free, r_robot=4, r_clean=2, edge_margin=0)
    slot = np.zeros_like(free)
    slot[14, 12:18] = True                   # slot centerline
    assert bool((slim & slot).any()), 'slim robot must service the slot'
    assert int(slim.sum()) > int(fat.sum())


def test_dilate_grows_and_preserves_input():
    seed = np.zeros((9, 9), dtype=bool)
    seed[4, 4] = True
    grown = _dilate(seed, 2)
    assert bool(grown[4, 4]) and bool(grown[2, 4]) and bool(grown[4, 6])
    assert not bool(grown[1, 4]), 'radius-2 dilation must not reach distance 3'
    assert int(seed.sum()) == 1, 'input mask must not be mutated'


def test_flood_fill_respects_walls():
    free = np.ones((7, 7), dtype=bool)
    free[:, 3] = False                       # full-height wall
    filled = _flood_fill(free, (0, 0))
    assert bool(filled[6, 2]) and not bool(filled[0, 4]), \
        'flood fill must not cross a solid wall'


def test_nearest_free_finds_closest_cell():
    free = np.zeros((10, 10), dtype=bool)
    free[7, 7] = True
    assert _nearest_free(free, 2, 2, max_r=3) is None
    assert _nearest_free(free, 7, 6) == (7, 7)


def test_stamp_disk_masks_walls():
    covered = np.zeros((11, 11), dtype=bool)
    mask = np.ones((11, 11), dtype=bool)
    mask[:, 6:] = False                      # wall from column 6 on
    _stamp_disk(covered, mask, cx=5, cy=5, rad=3)
    assert bool(covered[5, 5]) and bool(covered[5, 3])
    assert not covered[:, 6:].any(), 'disk must never stamp non-free cells'


@pytest.fixture
def map_generator():
    import importlib.util
    from pathlib import Path

    path = Path(__file__).resolve().parents[3] / 'tools/gen_livingroom_map.py'
    spec = importlib.util.spec_from_file_location('existing_map_generator', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.W = module.H = 21
    module.OX = module.OY = -.525
    return module


def test_map_generator_preserves_the_shifted_foot_of_a_tilted_leg(map_generator):
    import math

    module = map_generator
    grid = np.zeros((module.H, module.W), dtype=bool)
    pose = module.T_of([0., 0., .4, 0., math.pi / 4., 0.])
    module.add_box(grid, [.08, .08, .8], pose)
    # At z=.15, the tilted leg crosses x=-.25; x=0 is above the body band.
    assert grid[10, 5]
    assert not grid[10, 10]


def test_map_generator_applies_roll_before_pitch_and_yaw(map_generator):
    import math

    transform = map_generator.T_of([1., 2., 3., math.pi / 2., math.pi / 2., 0.])
    assert transform @ np.array([0., 1., 0., 1.]) == pytest.approx([2., 2., 3., 1.])


def test_map_generator_retains_planar_rotation_of_box_cells(map_generator):
    import math

    module = map_generator
    unturned = np.zeros((module.H, module.W), dtype=bool)
    quarter = np.zeros_like(unturned)
    module.add_box(unturned, [.31, .61, .4], module.T_of([0., 0., .1, 0., 0., 0.]))
    module.add_box(quarter, [.31, .61, .4], module.T_of([0., 0., .1, 0., 0., math.pi / 2.]))
    assert np.array_equal(quarter, np.rot90(unturned))
    assert unturned[10, 10] and int(unturned.sum()) > 0


@pytest.mark.parametrize('height', [-1., 1.])
def test_map_generator_excludes_boxes_outside_body_height(map_generator, height):
    module = map_generator
    grid = np.zeros((module.H, module.W), dtype=bool)
    module.add_box(grid, [.3, .6, .2], module.T_of([0., 0., height, .2, .3, .4]))
    assert not grid.any()
