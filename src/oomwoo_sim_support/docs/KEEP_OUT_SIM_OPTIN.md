# Simulation-only Nav2 virtual walls (opt-in)

This experimental Nav2 KeepoutFilter integration is OFF by default.
It is a sim-only experiment, not a motor / bumper / MCU safety controller.

The caller must create an absolute-path P5 binary PGM mask, with a
YAML metadata file matching the selected simulation map resolution,
origin, width and height exactly. Black 0 means forbidden; white 255
means free. No ambiguous grey, empty mask, or full-map mask is accepted.

Run ONLY in an isolated offline Gazebo sim:
  ros2 launch oomwoo_sim_support sim_bringup.launch.py \
    map:=/absolute/test_room.yaml keepout_mask:=/absolute/keepout.yaml

The overlay is derived from the chosen Nav2 params file at launch,
using two costmap filter plugins (KeepoutFilter, inflation) on BOTH
global and local costmaps. Stock Nav2 params are unchanged; the overlay
is removed at launch shutdown. Dedicated mask and type-0 info servers
are lifecycle managed. Omit keepout_mask to retain stock behavior.

## Reproducible validation (2026-10-29)

At this contribution's exact commit, from the repository root:

```bash
PYTHONPATH=src/oomwoo_sim_support python3 -m unittest discover \
  -s src/oomwoo_sim_support/test -p 'test_keepout*.py' -v
PYTHONPATH=src/oomwoo_sim_support python3 -O -m unittest discover \
  -s src/oomwoo_sim_support/test -p 'test_keepout*.py' -v
```

Host Python (without ROS installed): 10/10 config tests PASS, two launch
checks SKIP (in both ordinary and -O). Inside the existing offline
`makerspet/oomwoo:jazzy-dev` Podman image with ROS2 Jazzy sourced,
`--network none --rm --cpus 2 --memory 1g` and read-only source mount:
**12/12 PASS** (10 config + 2 actual ROS2 launch-context tests),
plus `generate_launch_description()` PASS (16 entities). A separate
real official test_room map (140x140) overlay check PASS, verifying that
ONLY the new filter blocks differ from stock Nav2 configuration.
No ROS2 actions, DDS graph, Gazebo world or physical hardware were started.

Specific follow-up before deployment: launch a fresh isolated Gazebo
instance and prove both local/odom and global/map costmap observations
change to lethal at the forbidden cells with frame-correct TF, lifecycle
ACTIVE and no unexpected motion. Repeat with a forbidden Nav2 destination;
reject a partial successful route that stops short. Then validate that
reactive /cmd_vel coverage respects an independent keepout admission gate.

LIMITS: current validation is static/synthetic until a fresh Gazebo
trial actually verifies both costmaps, TF conversion, lifecycle, and
absence of forbidden entries. A prior Valdi320 private Jazzy/Gazebo
KeepoutFilter proof uses a different integration; it does not establish
this launch working. Nav2 can report a successful partial plan short
of a forbidden destination, so a separate goal-admission check is
mandatory. Reactive room coverage via cmd_vel bypasses Nav2 filters:
it needs its OWN no-go guard and exclusive, watchdog-protected motor
command source. Never use on a physical robot before safety review.

## Output preservation follow-up (2026-10-29)

The generated overlay now acquires its filename exclusively, after YAML
serialization. A file created by another writer during serialization is
preserved; an existing dangling symlink is rejected without creating its
target. A serialization error leaves no output file. This enforces the
existing no-overwrite contract; it does not provide atomic publication
of a fully written file or durability after an interrupted write.

The focused `test_keepout_config` module passes 13/13 host Python tests,
including three new cases for these behaviors. The concurrent-writer case
failed on the original implementation before the correction. Reproduce
from the repository root with an existing PyYAML installation:

```bash
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH=src/oomwoo_sim_support:src/oomwoo_sim_support/test \
  python3 -m unittest test_keepout_config -v
```

This follow-up used Python 3.14 and the existing host PyYAML 6.0.2 exposed
to Toolbx through `/run/host/usr/lib64/python3.14/site-packages`, added to
the process-local PYTHONPATH. No dependency was installed. Machine lock
and resource limits were applied; see `evidence/keepout_exclusive_output_codex_20261029.json`.
These 13 tests do not repeat the earlier ROS2 launch-context checks and
do not establish ROS2/DDS, Gazebo or physical integration.

## Existing plugin namespace preservation (2026-10-29)

The existing filter-conflict check also rejects parameter blocks already
named `keepout_filter` or `keepout_inflation`, in either costmap. Otherwise
the overlay could silently replace a custom ordinary layer with a different
plugin while leaving its name in the original `plugins` list. Both the
input parameters and output directory are preserved on rejection.

The same focused config module now passes 15/15 host tests: four conflicting
costmap/name combinations, preservation of an unrelated custom layer, and
the previous cases. This extends the existing no-overwrite validation and
adds no node, timer or runtime control. See
`evidence/keepout_plugin_names_codex_20261029.json` for the reproduction and
its host-only limits. ROS2, Gazebo and physical integration remain pending.


## Ordinary obstacle corners with a mask (2026-10-30)

Masked planning now reuses the existing square dilation for ordinary occupied
and unknown cells as well as the virtual mask. The additional Nav2 inflation
also affects ordinary obstacles; cardinal-only planner clearance admitted
waypoints whose 0.18 m disk overlapped furniture corners.

On the unchanged stock test_room map/world, fixed seed (0,0), and 220-cell
mask, the original plan had 11 such targets among 329. The corrected plan has
326 targets, none intersecting the conservative SDF box bounds, and minimum
box clearance improves from 0.1458 m to 0.20 m. Eligible planner cells decrease
from 12,456 to 12,360 (0.24 square metres). The meter's denominator is unchanged.
This only checks target disks; it does not certify intermediate trajectories,
localization, physical contacts, or completed room coverage. The 0.18 m
planning proxy is larger than the simulator's 0.1745 m body radius.

The causal corner case fails on the original source; 16 selected mask and
dilation cases pass after correction, with 57 unrelated cases deselected.
The two changed files pass flake8. The no-mask cardinal contract and generic
helper default remain unchanged. Narrow diagonal passages can be excluded;
full masked 90%/80% acceptance remains pending. No new mode, subscription,
node, timer, world edit, reduced filter clearance or ABORTED permission was
introduced. Detailed source, commands, fixture and limits:
[ordinary_corner_delivery_20261030.json](evidence/ordinary_corner_delivery_20261030.json),
[static plan comparison](evidence/ordinary_corner_static_plan_20261030.json).


The subsequent native masked room run ended after 185.201 s with valid
13.02% coverage and a motion fault, not a pass. Its first failure was a
pose arriving 5 ms ahead of the received simulation clock; the relay
revoked ownership and Nav2 confirmed cancellation. It did not reach the
previous sofa/contact episode, so this is not a controlled comparison of
room coverage before/after the corner correction. The seven read-only
inputs were unchanged. The sampled Keepout audit remained valid with
8,003 poses, 8,002 segments, no forbidden crossings and 0.10 s maximum gap.
Gazebo and 19 auxiliaries exited cleanly; only the scorer returned 1.
The phase audit now includes actual original pose stamps and action UUIDs.
See [full native result](evidence/gazebo_keepout_corner_fixed_20261030.json).
Clock/TF arrival ordering must be handled without accepting future poses or
extending their freshness budgets before another full-room attempt.


### Completed room after the TF ordering correction (2026-10-30)

With contributor relay fa8b7af, the same native trial completed the sweep at
95.60% coverage in 953.064 seconds of process time. All seven read-only inputs
were unchanged. The audit was valid for 46,507 truth poses and 46,506 chords,
with no forbidden disk crossings and a maximum pose gap of 0.04 seconds.
Twenty non-scorer processes, including Gazebo, exited cleanly; the scorer
returned 1 because efficiency remained below the unchanged 80% requirement.
There were 24 terminal Nav2 UUIDs (20 succeeded, 4 canceled), each with an
accepted NAV release, and no motion-lease revocations. DockDrive stayed IDLE.

The scorer reported 78.88% efficiency at its first target crossing, but this
matches the **preceding** meter report: coverage 89.87%, efficiency 78.88%.
The meter's first target report pairs coverage 90.11% with efficiency 78.72%.
The meter publishes these scalars separately and the scorer can latch coverage
before its matching efficiency callback. Both values fail the gate. Preserve
the raw result; neither is a full acceptance pass. The two observations use
different start-time origins, so their elapsed times must not be conflated.

Next, fuse the existing meter and scorer into one regression process so the
grade consumes their same atomic calculation. Keep the standalone meter path,
original 0.1745 m meter footprint, denominator, 90%/80% criteria and native
shutdown checks. Then assess the small remaining distance-efficiency margin.
See [full native result](evidence/gazebo_keepout_tf_fixed_20261030.json) and
[receipt and limits](evidence/gazebo_keepout_tf_fixed_delivery_20261030.json).
