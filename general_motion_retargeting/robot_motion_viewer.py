import os
import time
import mujoco as mj
import mujoco.viewer as mjv
import imageio
from scipy.spatial.transform import Rotation as R
from general_motion_retargeting import ROBOT_XML_DICT, ROBOT_BASE_DICT, VIEWER_CAM_DISTANCE_DICT
from loop_rate_limiters import RateLimiter
import numpy as np
from rich import print


def draw_frame(
    pos,
    mat,
    v,
    size,
    joint_name=None,
    orientation_correction=R.from_euler("xyz", [0, 0, 0]),
    pos_offset=np.array([0, 0, 0]),
):
    rgba_list = [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1]]
    for i in range(3):
        geom = v.user_scn.geoms[v.user_scn.ngeom]
        mj.mjv_initGeom(
            geom,
            type=mj.mjtGeom.mjGEOM_ARROW,
            size=[0.01, 0.01, 0.01],
            pos=pos + pos_offset,
            mat=mat.flatten(),
            rgba=rgba_list[i],
        )
        if joint_name is not None:
            geom.label = joint_name  # 这里赋名字
        fix = orientation_correction.as_matrix()
        mj.mjv_connector(
            v.user_scn.geoms[v.user_scn.ngeom],
            type=mj.mjtGeom.mjGEOM_ARROW,
            width=0.005,
            from_=pos + pos_offset,
            to=pos + pos_offset + size * (mat @ fix)[:, i],
        )
        v.user_scn.ngeom += 1

class RobotMotionViewer:
    def __init__(self,
                robot_type,
                camera_follow=True,
                motion_fps=30,
                transparent_robot=0,
                # video recording
                record_video=False,
                video_path=None,
                video_width=640,
                video_height=480,
                show_robot_coords=True,
                traj_csv_path=None,
                show_orientation=True):
        
        self.robot_type = robot_type
        self.xml_path = ROBOT_XML_DICT[robot_type]
        self.model = mj.MjModel.from_xml_path(str(self.xml_path))
        self.data = mj.MjData(self.model)
        self.robot_base = ROBOT_BASE_DICT[robot_type]
        self.viewer_cam_distance = VIEWER_CAM_DISTANCE_DICT[robot_type]
        mj.mj_step(self.model, self.data)
        
        self.motion_fps = motion_fps
        self.rate_limiter = RateLimiter(frequency=self.motion_fps, warn=False)
        self.camera_follow = camera_follow
        self.record_video = record_video
        self.show_robot_coords = show_robot_coords
        self.show_orientation = show_orientation
        self.traj_csv_path = traj_csv_path
        self._traj_fp = None
        self._frame_idx = 0
        self._last_pos = None
        self._last_vel = np.zeros(3)
        if self.traj_csv_path:
            try:
                first = not os.path.exists(self.traj_csv_path)
                self._traj_fp = open(self.traj_csv_path, 'a')
                if first:
                    self._traj_fp.write('frame,time,x,y,z,vx,vy,vz,qw,qx,qy,qz,yaw_deg,pitch_deg,roll_deg\n')
                print(f"[cyan]Logging trajectory to {self.traj_csv_path}[/cyan]")
            except Exception as e:
                print(f"[red]Failed to open traj CSV: {e}[/red]")
                self._traj_fp = None

        self.viewer = mjv.launch_passive(
            model=self.model,
            data=self.data,
            show_left_ui=False,
            show_right_ui=False)

        self.viewer.opt.flags[mj.mjtVisFlag.mjVIS_TRANSPARENT] = transparent_robot
        
        if self.record_video:
            assert video_path is not None, "Please provide video path for recording"
            self.video_path = video_path
            video_dir = os.path.dirname(self.video_path)
            
            if not os.path.exists(video_dir):
                os.makedirs(video_dir)
            self.mp4_writer = imageio.get_writer(self.video_path, fps=self.motion_fps)
            print(f"Recording video to {self.video_path}")
            
            # Initialize renderer for video recording
            self.renderer = mj.Renderer(self.model, height=video_height, width=video_width)
        
    def step(self, 
            # robot data
            root_pos, root_rot, dof_pos, 
            # human data
            human_motion_data=None, 
            show_human_body_name=False,
            # scale for human point visualization
            human_point_scale=0.1,
            # human pos offset add for visualization    
            human_pos_offset=np.array([0.0, 0.0, 0]),
            # rate limit
            rate_limit=True, 
            follow_camera=True,
            show_robot_coords=None,
            show_orientation=None,
            ):
        """
        by default visualize robot motion.
        also support visualize human motion by providing human_motion_data, to compare with robot motion.
        
        human_motion_data is a dict of {"human body name": (3d global translation, 3d global rotation)}.

        if rate_limit is True, the motion will be visualized at the same rate as the motion data.
        else, the motion will be visualized as fast as possible.
        """
        
        self.data.qpos[:3] = root_pos
        self.data.qpos[3:7] = root_rot # quat need to be scalar first! for mujoco
        self.data.qpos[7:] = dof_pos
        
        mj.mj_forward(self.model, self.data)
        
        if follow_camera:
            self.viewer.cam.lookat = self.data.xpos[self.model.body(self.robot_base).id]
            self.viewer.cam.distance = self.viewer_cam_distance
            self.viewer.cam.elevation = -10  # 正面视角，轻微向下看
            # self.viewer.cam.azimuth = 180    # 正面朝向机器人

        if human_motion_data is not None:
            # Clean custom geometry
            self.viewer.user_scn.ngeom = 0
            # Draw the task targets for reference
            for human_body_name, (pos, rot) in human_motion_data.items():
                draw_frame(
                    pos,
                    R.from_quat(rot, scalar_first=True).as_matrix(),
                    self.viewer,
                    human_point_scale,
                    pos_offset=human_pos_offset,
                    joint_name=human_body_name if show_human_body_name else None
                )

        # Overlay robot coordinates (root) in viewer
        if show_robot_coords is None:
            show_robot_coords = self.show_robot_coords
        if show_orientation is None:
            show_orientation = self.show_orientation

        overlay_lines = []
        # Compute velocity
        if self._last_pos is None:
            vel = np.zeros(3)
        else:
            dt = 1.0 / self.motion_fps if self.motion_fps > 0 else 0.0
            vel = (root_pos - self._last_pos) / dt if dt > 0 else np.zeros(3)
        self._last_pos = root_pos.copy()
        self._last_vel = vel

        if show_robot_coords:
            overlay_lines.append(f"XYZ: {root_pos[0]:.3f} {root_pos[1]:.3f} {root_pos[2]:.3f}")
            overlay_lines.append(f"Vel: {vel[0]:.3f} {vel[1]:.3f} {vel[2]:.3f}")
        if show_orientation:
            # root_rot is (w,x,y,z) scalar first; convert to yaw/pitch/roll (Z,Y,X) or use intrinsic 'zyx'
            try:
                eul = R.from_quat(root_rot, scalar_first=True).as_euler('zyx', degrees=True)
                yaw, pitch, roll = eul[0], eul[1], eul[2]
                overlay_lines.append(f"Yaw/Pitch/Roll: {yaw:+.1f} {pitch:+.1f} {roll:+.1f}")
            except Exception:
                pass
        if overlay_lines:
            try:
                self.viewer.add_overlay(mj.mjtGridPos.mjGRID_TOPLEFT, "Robot", "\n".join(overlay_lines))
            except Exception:
                pass

        # Trajectory CSV logging
        if self._traj_fp:
            t = self._frame_idx / self.motion_fps if self.motion_fps > 0 else 0.0
            try:
                eul = R.from_quat(root_rot, scalar_first=True).as_euler('zyx', degrees=True)
                yaw, pitch, roll = eul[0], eul[1], eul[2]
            except Exception:
                yaw = pitch = roll = 0.0
            self._traj_fp.write(f"{self._frame_idx},{t:.6f},{root_pos[0]:.6f},{root_pos[1]:.6f},{root_pos[2]:.6f},"
                                 f"{vel[0]:.6f},{vel[1]:.6f},{vel[2]:.6f},{root_rot[0]:.6f},{root_rot[1]:.6f},"
                                 f"{root_rot[2]:.6f},{root_rot[3]:.6f},{yaw:.6f},{pitch:.6f},{roll:.6f}\n")
        self._frame_idx += 1

        self.viewer.sync()
        if rate_limit is True:
            self.rate_limiter.sleep()

        if self.record_video:
            # Use renderer for proper offscreen rendering
            self.renderer.update_scene(self.data, camera=self.viewer.cam)
            img = self.renderer.render()
            self.mp4_writer.append_data(img)
    
    def close(self):
        self.viewer.close()
        time.sleep(0.5)
        if self.record_video:
            self.mp4_writer.close()
            print(f"Video saved to {self.video_path}")
        if self._traj_fp:
            try:
                self._traj_fp.close()
            except Exception:
                pass
