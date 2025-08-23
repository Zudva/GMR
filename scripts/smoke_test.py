"""Simple smoke test for core modules.

Usage:
  macOS interactive viewer (required): mjpython scripts/smoke_test.py
  Headless fallback (no GUI if not using mjpython on macOS): python scripts/smoke_test.py
"""
import sys
import pathlib
import os
import pickle

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT))

from general_motion_retargeting import ROBOT_XML_DICT  # noqa
import mujoco as mj


def _validate_motion_file(pkl_path: pathlib.Path, model, viewer, max_frames=2):
    required = {"fps", "root_pos", "root_rot", "dof_pos"}
    try:
        with open(pkl_path, "rb") as f:
            motion = pickle.load(f)
    except Exception as e:
        print(f"[red]FAIL load {pkl_path.name}: {e}[/red]")
        return False
    missing = required - motion.keys()
    if missing:
        print(f"[red]FAIL {pkl_path.name}: missing keys {missing}[/red]")
        return False
    import numpy as np
    root_pos = np.asarray(motion["root_pos"])  # (T,3)
    root_rot_xyzw = np.asarray(motion["root_rot"])  # (T,4) xyzw
    dof_pos = np.asarray(motion["dof_pos"])  # (T,Ndof)
    if root_pos.ndim != 2 or root_pos.shape[1] != 3:
        print(f"[red]FAIL {pkl_path.name}: root_pos shape {root_pos.shape}")
        return False
    if root_rot_xyzw.ndim != 2 or root_rot_xyzw.shape[1] != 4:
        print(f"[red]FAIL {pkl_path.name}: root_rot shape {root_rot_xyzw.shape}")
        return False
    if dof_pos.ndim != 2:
        print(f"[red]FAIL {pkl_path.name}: dof_pos shape {dof_pos.shape}")
        return False
    ndof = model.nq - 7
    if dof_pos.shape[1] != ndof:
        print(f"[yellow]WARN {pkl_path.name}: dof mismatch motion({dof_pos.shape[1]}) model({ndof}) skipping playback[/yellow]")
        return True  # structural pass, just skip playback
    T = dof_pos.shape[0]
    if T == 0:
        print(f"[red]FAIL {pkl_path.name}: zero frames")
        return False
    # Quick quaternion norm check (convert to wxyz first)
    rot_wxyz = root_rot_xyzw[:, [3,0,1,2]]
    norms = np.linalg.norm(rot_wxyz, axis=1)
    bad = np.sum((norms < 0.5) | (norms > 1.5))
    if bad:
        print(f"[yellow]WARN {pkl_path.name}: {bad} suspect quaternion norms")
    # Optional minimal playback
    if viewer is not None:
        frames = min(T, max_frames)
        for i in range(frames):
            viewer.step(root_pos[i], rot_wxyz[i], dof_pos[i], rate_limit=False)
    else:
        data = mj.MjData(model)
        frames = min(T, max_frames)
        for i in range(frames):
            data.qpos[:3] = root_pos[i]
            data.qpos[3:7] = rot_wxyz[i]
            data.qpos[7:] = dof_pos[i]
            mj.mj_forward(model, data)
    print(f"[green]OK {pkl_path.name}: frames={T} ndof={dof_pos.shape[1]}[/green]")
    return True


def main():
    robot = next(iter(ROBOT_XML_DICT.keys()))
    xml_path = ROBOT_XML_DICT[robot]
    model = mj.MjModel.from_xml_path(str(xml_path))
    viewer = None
    try:
        from general_motion_retargeting.robot_motion_viewer import RobotMotionViewer  # noqa
        viewer = RobotMotionViewer(robot_type=robot, show_robot_coords=False, show_orientation=False)
        gui_mode = 'viewer'
    except RuntimeError as e:
        print(f"[yellow]Viewer unavailable ({e}). Falling back to headless mode.[/yellow]")
        gui_mode = 'headless'
    except Exception as e:
        print(f"[yellow]Unexpected viewer init error ({e}). Headless fallback.[/yellow]")
        gui_mode = 'headless'

    # Basic viewer sanity (3 blank steps)
    import numpy as np
    zero_root_pos = np.zeros(3)
    root_rot = np.array([1,0,0,0])
    dof = np.zeros(model.nq - 7)
    for _ in range(3):
        if viewer:
            viewer.step(zero_root_pos, root_rot, dof, rate_limit=False)
        else:
            data = mj.MjData(model)
            data.qpos[:3] = zero_root_pos
            data.qpos[3:7] = root_rot
            data.qpos[7:] = dof
            mj.mj_forward(model, data)

    # Validate motion files in out/
    out_dir = ROOT / 'out'
    if out_dir.exists():
        pkls = sorted([p for p in out_dir.iterdir() if p.suffix == '.pkl'])
        if pkls:
            print(f"Scanning {len(pkls)} motion pkl files in 'out/' ...")
            passed = 0
            for p in pkls:
                if _validate_motion_file(p, model, viewer, max_frames=2):
                    passed += 1
            print(f"Motion validation: {passed}/{len(pkls)} passed structurally.")
        else:
            print("No .pkl motion files found in out/.")
    else:
        print("No out/ directory (skipping motion validation).")

    if viewer:
        viewer.close()
    print(f"Smoke test completed ({gui_mode} mode).")

if __name__ == "__main__":
    main()
