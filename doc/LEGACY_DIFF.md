# Legacy Commit 1b9f50c vs Current (HEAD)

This document captures the historical delta so the main README can stay concise.

## Summary
Legacy = “load BVH & retarget”. Current = Legacy + automated orientation, normalization & root restoration, dynamic IK tuning, morphological offset heuristics, extensive diagnostics, FBX conversion, camera & trajectory tooling.

## High-Level Themes
- Input Expansion: Added FBX (via Blender conversion) alongside BVH, SMPL-X.
- Robust Orientation: Presets, auto basis detection, quick preset scan, optional Ubisoft axes handling.
- Root Processing: Normalization + optional reinjection of original root trajectory; pelvis offset heuristics.
- Morphology Adaptation: Heuristic offset suggestion & optional runtime apply, auto pelvis offset.
- Diagnostics & Logging: Per-frame error logs, CSV exports, task error breakdown, initial target dumps, pelvis anchor insight.
- Data Dumps: First-frame JSON, full target trajectory CSV, trajectory export for plotting.
- Viewer Enhancements: Camera distance override, trajectory overlays, stability fixes.
- Tuning Convenience: Runtime pelvis weight overrides; toggle human scaling.
- Documentation: Expanded quick start, diagnostics, differences vs original video pipeline.

## Detailed Table
| Area | Legacy (1b9f50c) | Current | Impact |
|------|------------------|---------|--------|
| Scripts | `scripts/bvh_to_robot.py` only | + `fbx_to_robot.py`, dataset scripts, smoke tests | FBX & richer workflows |
| FBX Support | None | Blender-based FBX→BVH + inspection (`--use_blender`, `--inspect_fbx`) | Direct use of OptiTrack / other FBX |
| Orientation | Implicit BVH basis | Presets + `--orient_fix auto` + `--quick_orient_scan` + `--ubisoft_axes` | Automatic upright & forward alignment |
| Root Handling | Raw root only | `--normalize_root`, `--use_root_motion`, `--align_root_xy`, `--pelvis_z_offset`, `--auto_pelvis_offset` | Clean floor + preserved trajectory + bias removal |
| IK Weights | Static JSON | Runtime overrides `--pelvis_pos_w1/2` | Faster tuning of root stability |
| Morphology Offsets | Manual edits only | Heuristic `--suggest_offsets` / apply, plus auto pelvis offset | Rapid coarse calibration |
| Scaling | Always on (internal) | Toggle `--no_scale_human` | Diagnostics of morphology mismatches |
| Diagnostics | None (basic FPS print) | Errors/logging: `--log_errors`, `--errors_csv`, `--task_error_breakdown`, `--debug_alignment`, `--debug_initial_targets`, `--debug_pelvis_anchor` | Quantitative iteration |
| Data Dumps | None | `--dump_first_frame_json`, `--dump_targets_csv`, `--traj_csv` | Offline analysis & plotting |
| Viewer | Basic pose display | Camera distance override, target overlays, trajectory CSV, stability fixes | Reproducible visualization |
| Camera Control | Fixed distance | `--camera_dist` | Consistent framing for comparisons |
| Code Size | ~1 small script + viewer | +1200 lines (FBX pipeline) + diagnostics code | Feature breadth |
| Docs | Minimal README | Expanded README (quick start, diagnostics, comparisons) | Lower onboarding cost |

## Minimal Legacy Command (for Reference)
```
python scripts/bvh_to_robot.py --bvh_file path/to/file.bvh --robot unitree_g1
```
No orientation scan, no root normalization, no diagnostics.

## Representative Current FBX Command
```
python scripts/fbx_to_robot.py --fbx_file sample.fbx --robot unitree_g1 \
  --orient_fix auto --quick_orient_scan --normalize_root --use_root_motion \
  --suggest_offsets --auto_pelvis_offset --log_errors --errors_csv errors.csv \
  --dump_first_frame_json first_frame.json --dump_targets_csv targets.csv
```

## Migration Notes
1. Start with just `--orient_fix auto --quick_orient_scan --normalize_root`.
2. Add `--use_root_motion` if you need original trajectory preserved.
3. Use `--log_errors --dump_first_frame_json` when a limb misaligns.
4. Apply offsets heuristically (`--suggest_offsets`) then persist manually in the JSON mapping once satisfied.
5. Keep pelvis stable by tuning `--pelvis_pos_w1/2` or letting `--auto_pelvis_offset` run.

## Rationale for Separation
The table is historical reference; regular users only need the quick start paths. Keeping this doc separate avoids overwhelming new users scanning the README.

---
Generated for traceability; update when adding major pipeline phases (e.g., real-time streaming, inverse dynamics coupling, hand pose retargeting).
