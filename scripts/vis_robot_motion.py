import pickle
import argparse
import numpy as np
from general_motion_retargeting import RobotMotionViewer

parser = argparse.ArgumentParser()
parser.add_argument("--motion", required=True, help="Path to .pkl motion file")
parser.add_argument("--robot", default="unitree_g1", help="Robot name")
parser.add_argument("--video", default=None, help="Path to save video (mp4)")
args = parser.parse_args()

with open(args.motion, "rb") as f:
    motion = pickle.load(f)

# Stored root_rot in saved pickle is in xyzw order (see bvh_to_robot.py saving code),
# but RobotMotionViewer expects wxyz. Convert once here.
root_rot_xyzw = motion["root_rot"]  # shape (T,4) xyzw
if root_rot_xyzw.shape[1] != 4:
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
        rate_limit=True,
    )

viewer.close()