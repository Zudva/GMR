# Changelog

All notable changes to this project will be documented in this file.

The format is based on Keep a Changelog, and this project (currently) does not yet follow semantic versioning strictly.

## [Unreleased]
- Future: per-joint orientation error logging, acceleration overlay, latency stats, unit tests for parsers.

## [0.2.0] - 2025-08-22
### Added
- Robot viewer overlay: root XYZ, velocity vector, yaw/pitch/roll.
- Trajectory CSV logging (position, velocity, quaternion, Euler angles).
- CLI flags in `scripts/fbx_to_robot.py`:
  - `--always_overwrite` force reconversion of FBX to BVH.
  - `--log_errors`, `--errors_csv` per-frame IK/task error metrics & key joint position errors.
  - `--dump_bvh_header` for BVH header inspection.
  - `--ubisoft_axes` axis convention override; `--normalize_root` root & floor normalization.
  - `--orient_fix` presets plus `auto` detection and `--auto_forward_axis`.
  - `--force_generic_loader` and internal loose BVH parser fallback for non-lafan channel orders.
- Auto-orientation detection using joint basis reconstruction.
- Root normalization & Ubisoft dataset emulation mode.

### Changed
- Quaternion composition in retarget pipeline (consistent scalar-first, correct order for orientation fixes).
- Blender conversion: robust temp script generation, multi-attempt export, streaming output option.

### Fixed
- FBX export previously producing static T-pose (now baking actions with proper evaluation range).
- Reshape errors in strict lafan parser (fallback generic + loose parser).
- Incorrect prone / rotated orientation (preset & auto fixes, axis handling, normalization).
- Various template string escaping & temporary file handling issues.

## [0.1.0] - 2025-08-15
### Added
- Initial FBX -> BVH retarget pipeline.
- Basic BVH parsing via lafan vendor code.
- Unitree G1 motion retarget + viewer.

