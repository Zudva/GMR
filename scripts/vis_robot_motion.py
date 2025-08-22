import pickle
import argparse
import numpy as np
from pathlib import Path
from general_motion_retargeting import RobotMotionViewer

try:
    import torch  # optional
except Exception:  # pragma: no cover
    torch = None

parser = argparse.ArgumentParser()
parser.add_argument("--motion", required=True, help="Path to motion file (.pkl or .pt)")
parser.add_argument("--robot", default="unitree_g1", help="Robot name")
parser.add_argument("--video", default=None, help="Path to save video (mp4)")
parser.add_argument("--headless", action="store_true", help="Do not open viewer (just validate frames)")
parser.add_argument("--summary", action="store_true", help="Print summary and exit")
parser.add_argument("--no_rate_limit", action="store_true", help="Play as fast as possible (disable real-time sleep)")
args = parser.parse_args()

motion_path = Path(args.motion)

def _load_motion(p: Path):
    if p.suffix == ".pkl":
        with open(p, "rb") as f:
            return pickle.load(f)
    if p.suffix == ".pt":
        if torch is None:
            raise RuntimeError("Torch not available to load .pt file")
        obj = torch.load(p, map_location="cpu")
        # convert tensors to numpy for uniform processing
        conv = {}
        for k, v in obj.items():
            if torch.is_tensor(v):
                conv[k] = v.detach().cpu().numpy()
            else:
                conv[k] = v
        return conv
    raise ValueError("Unsupported extension: use .pkl or .pt")

motion = _load_motion(motion_path)

if args.summary:
    print("--- Motion Summary ---")
    for k, v in motion.items():
        if isinstance(v, np.ndarray):
            print(f"{k}: shape={v.shape} dtype={v.dtype}")
        else:
            print(f"{k}: type={type(v)}")
    # continue if user also wants to render
    if args.headless and not args.video:
        exit(0)

# Stored root_rot in saved pickle is in xyzw order (see bvh_to_robot.py saving code),
# but RobotMotionViewer expects wxyz. Convert once here.
root_rot_xyzw = motion["root_rot"]  # shape (T,4) xyzw
if root_rot_xyzw.ndim != 2 or root_rot_xyzw.shape[1] != 4:
    raise ValueError("root_rot must have shape (T,4)")
root_rot_wxyz = np.concatenate([root_rot_xyzw[:, 3:4], root_rot_xyzw[:, :3]], axis=1)

# Normalize to avoid accumulation of numeric drift.
norms = np.linalg.norm(root_rot_wxyz, axis=1, keepdims=True)
norms[norms == 0] = 1.0
root_rot_wxyz /= norms

motion_fps = motion.get("fps", 30)
root_pos = motion["root_pos"]
dof_pos = motion["dof_pos"]
num_frames = len(root_pos)

if args.headless:
    # simple validation loop (no viewer)
    if root_pos.shape[0] != num_frames or dof_pos.shape[0] != num_frames:
        raise ValueError("Frame count mismatch among arrays")
    # Quaternion norm check
    qn_err = np.abs(np.linalg.norm(root_rot_wxyz, axis=1) - 1).max()
    print(f"Headless validation OK. Frames={num_frames} max_quat_norm_error={qn_err:.2e}")
    if args.video:
        print("[warn] --video ignored in headless mode")
else:
    viewer = RobotMotionViewer(
        robot_type=args.robot,
        motion_fps=motion_fps,
        record_video=bool(args.video),
        video_path=args.video,
        show_orientation=False,
        show_robot_coords=False,
    )
    for i in range(num_frames):
        viewer.step(
            root_pos=root_pos[i],
            root_rot=root_rot_wxyz[i],
            dof_pos=dof_pos[i],
            rate_limit=not args.no_rate_limit,
        )
    viewer.close()