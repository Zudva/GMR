"""Simple smoke test for core modules.

Usage:
  macOS interactive viewer (required): mjpython scripts/smoke_test.py
  Headless fallback (no GUI if not using mjpython on macOS): python scripts/smoke_test.py
"""
import sys
import pathlib

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT))

from general_motion_retargeting import ROBOT_XML_DICT  # noqa
import mujoco as mj


def main():
    robot = next(iter(ROBOT_XML_DICT.keys()))
    xml_path = ROBOT_XML_DICT[robot]
    model = mj.MjModel.from_xml_path(str(xml_path))
    data = mj.MjData(model)
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
    import numpy as np
    zero_root_pos = np.zeros(3)
    root_rot = np.array([1,0,0,0])
    dof = np.zeros(model.nq - 7)
    for _ in range(3):
        if viewer:
            viewer.step(zero_root_pos, root_rot, dof, rate_limit=False)
        else:
            # Simple physics advance
            data.qpos[:3] = zero_root_pos
            data.qpos[3:7] = root_rot
            data.qpos[7:] = dof
            mj.mj_forward(model, data)
    if viewer:
        viewer.close()
    print(f"Smoke test passed ({gui_mode} mode).")

if __name__ == "__main__":
    main()
