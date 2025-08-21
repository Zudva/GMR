import argparse
import time
import numpy as np
import mujoco as mj

from general_motion_retargeting import RobotMotionViewer
from general_motion_retargeting.params import ROBOT_XML_DICT


def main():
    parser = argparse.ArgumentParser(description="Minimal Mujoco viewer smoke test for GMR robots")
    parser.add_argument("--robot", type=str, default="unitree_g1",
                        choices=list(ROBOT_XML_DICT.keys()))
    parser.add_argument("--seconds", type=float, default=5.0, help="Duration to run the viewer")
    parser.add_argument("--fps", type=int, default=30, help="Playback FPS")
    parser.add_argument("--record_video", action="store_true", help="Record an MP4 during the run")
    parser.add_argument("--video_path", type=str, default=None, help="Path to save the MP4 if recording")
    args = parser.parse_args()

    xml_path = ROBOT_XML_DICT[args.robot]
    model = mj.MjModel.from_xml_path(str(xml_path))

    dof = model.nq - 7  # qpos = [root(7) + joints(dof)]
    fps = int(args.fps)
    frames = int(args.seconds * fps)

    # Setup viewer
    video_path = args.video_path
    if args.record_video and video_path is None:
        video_path = f"videos/smoke_{args.robot}.mp4"

    viewer = RobotMotionViewer(robot_type=args.robot,
                               motion_fps=fps,
                               record_video=args.record_video,
                               video_path=video_path)

    # Zero base pose
    root_pos = np.zeros(3)
    root_rot = np.array([1.0, 0.0, 0.0, 0.0])  # wxyz for mujoco

    # Make a tiny sinusoid on the first few joints if available
    t0 = time.time()
    for i in range(frames):
        t = i / float(fps)
        dof_pos = np.zeros(dof)
        if dof > 0:
            dof_pos[0] = 0.2 * np.sin(2 * np.pi * 0.5 * t)
        if dof > 1:
            dof_pos[1] = 0.2 * np.sin(2 * np.pi * 0.33 * t)
        if dof > 2:
            dof_pos[2] = 0.15 * np.sin(2 * np.pi * 0.25 * t)

        viewer.step(root_pos=root_pos,
                    root_rot=root_rot,
                    dof_pos=dof_pos,
                    human_motion_data=None,
                    rate_limit=True,
                    follow_camera=True)

    viewer.close()


if __name__ == "__main__":
    main()
