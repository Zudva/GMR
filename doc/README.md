# GMR Documentation

This directory contains project documentation and the changelog.

## Contents
- `CHANGELOG.md` – chronological list of notable changes.
- `pipeline.md` (planned) – overview of data flow FBX -> BVH -> frames -> IK / robot.
- `retarget_flags.md` (planned) – detailed explanation of CLI flags for `scripts/fbx_to_robot.py`.

## Quick Start
1. Provide an FBX file: `python scripts/fbx_to_robot.py --fbx_file input.fbx --always_overwrite`
2. Inspect BVH header: `--dump_bvh_header 80`
3. Enable error diagnostics & trajectory logging:
```
python scripts/fbx_to_robot.py --fbx_file input.fbx \
  --always_overwrite --log_errors --errors_csv errors.csv \
  --orient_fix auto --normalize_root --dump_bvh_header 60
```
4. Optional Ubisoft axis behavior: add `--ubisoft_axes`.

## Diagnostics Features
- Error CSV: frame,error1,error2,pelvis_pos_err,left_hand_pos_err,right_hand_pos_err
- Trajectory CSV: frame,time,x,y,z,vx,vy,vz,qw,qx,qy,qz,yaw_deg,pitch_deg,roll_deg
- Viewer overlay: XYZ, velocity, yaw/pitch/roll.

## Orientation & Axes
- `--orient_fix` presets: x90, x-90, y90, y-90, z180, auto.
- `--auto_forward_axis` sets desired forward (x or y) in auto mode.
- `--ubisoft_axes` skips extra quaternion composition preserving original Ubisoft conversion semantics.
- `--normalize_root` recenters Hips at origin and sets floor to z=0 estimated from feet.

## BVH Parsing Robustness
- Primary: lafan vendor loader.
- Fallback: generic hierarchical parser.
- Loose: permissive channel-order parser for non-standard exports.

## Viewer (macOS Note)
On macOS the interactive MuJoCo viewer must be launched with `mjpython`. Example:
```
mjpython scripts/fbx_to_robot.py --fbx_file input.fbx --always_overwrite
```
If you run `python` instead of `mjpython` you may see: `launch_passive requires that the Python script be run under mjpython on macOS`.

Smoke test:
```
mjpython scripts/smoke_test.py   # full viewer
python  scripts/smoke_test.py   # headless fallback
```

## Planned Docs
- `pipeline.md` will illustrate transform composition order (axis fix -> orient fix -> auto orientation -> normalization).
- `retarget_flags.md` will map flags to internal code sections for easier maintenance.

## License
See root `LICENSE`.

