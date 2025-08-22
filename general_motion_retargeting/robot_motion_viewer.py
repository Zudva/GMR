"""MuJoCo robot motion viewer (clean consolidated version).

Contains overlay options (coords, orientation, quaternion, root diff), optional
video recording with text annotation, and optional human reference frame axes.
"""

from typing import Dict, Tuple, Optional
import os
import time
import imageio
import numpy as np
import mujoco as mj
import mujoco.viewer as mjv
from scipy.spatial.transform import Rotation as R
from rich import print

from .params import (
    ROBOT_XML_DICT,
    ROBOT_BASE_DICT,
    VIEWER_CAM_DISTANCE_DICT,
)
from loop_rate_limiters import RateLimiter


def draw_frame(
    pos: np.ndarray,
    rotm: np.ndarray,
    viewer,
    size: float,
    joint_name: Optional[str] = None,
    orientation_correction: R = R.from_euler("xyz", [0, 0, 0]),
    pos_offset: Optional[np.ndarray] = None,
):
    """Draws 3 arrows (RGB) for a local frame."""
    if pos_offset is None:
        pos_offset = np.zeros(3)
    rgba_list = [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1]]
    fix = orientation_correction.as_matrix()
    for i in range(3):
        geom = viewer.user_scn.geoms[viewer.user_scn.ngeom]
        mj.mjv_initGeom(
            geom,
            type=mj.mjtGeom.mjGEOM_ARROW,
            size=[0.01, 0.01, 0.01],
            pos=pos + pos_offset,
            mat=rotm.flatten(),
            rgba=rgba_list[i],
        )
        if joint_name is not None:
            geom.label = joint_name
        mj.mjv_connector(
            viewer.user_scn.geoms[viewer.user_scn.ngeom],
            type=mj.mjtGeom.mjGEOM_ARROW,
            width=0.005,
            from_=pos + pos_offset,
            to=pos + pos_offset + size * (rotm @ fix)[:, i],
        )
        viewer.user_scn.ngeom += 1


class RobotMotionViewer:
    def __init__(
        self,
        robot_type: str,
        camera_follow: bool = True,
        motion_fps: int = 30,
        transparent_robot: int = 0,
        record_video: bool = False,
        video_path: Optional[str] = None,
        video_width: int = 640,
        video_height: int = 480,
        show_robot_coords: bool = True,
        show_orientation: bool = True,
        show_quat: bool = False,
        show_root_diff: bool = False,
        record_overlay_text: bool = True,
        traj_csv_path: Optional[str] = None,
        camera_distance_override: Optional[float] = None,
    ):
        self.robot_type = robot_type
        self.xml_path = ROBOT_XML_DICT[robot_type]
        self.model = mj.MjModel.from_xml_path(str(self.xml_path))
        self.data = mj.MjData(self.model)
        self.robot_base = ROBOT_BASE_DICT[robot_type]
        self.viewer_cam_distance = VIEWER_CAM_DISTANCE_DICT[robot_type]
        self.camera_distance_override = camera_distance_override
        mj.mj_step(self.model, self.data)

        self.motion_fps = motion_fps
        self.rate_limiter = RateLimiter(frequency=self.motion_fps, warn=False)
        self.camera_follow = camera_follow
        self.record_video = record_video
        self.show_robot_coords = show_robot_coords
        self.show_orientation = show_orientation
        self.show_quat = show_quat
        self.show_root_diff = show_root_diff
        self.record_overlay_text = record_overlay_text
        self.traj_csv_path = traj_csv_path
        self._traj_fp = None
        self._frame_idx = 0
        self._last_pos = None
        self._last_vel = np.zeros(3)
        if self.traj_csv_path:
            try:
                first = not os.path.exists(self.traj_csv_path)
                self._traj_fp = open(self.traj_csv_path, "a")
                if first:
                    self._traj_fp.write(
                        "frame,time,x,y,z,vx,vy,vz,qw,qx,qy,qz,yaw_deg,pitch_deg,roll_deg\n"
                    )
                print(f"[cyan]Logging trajectory to {self.traj_csv_path}[/cyan]")
            except Exception as e:
                print(f"[red]Failed to open traj CSV: {e}[/red]")
                self._traj_fp = None

        self.viewer = mjv.launch_passive(
            model=self.model,
            data=self.data,
            show_left_ui=False,
            show_right_ui=False,
        )
        self.viewer.opt.flags[mj.mjtVisFlag.mjVIS_TRANSPARENT] = transparent_robot

        if self.record_video:
            assert video_path is not None, "Provide --video_path when --record_video"
            self.video_path = video_path
            os.makedirs(os.path.dirname(self.video_path), exist_ok=True)
            self.mp4_writer = imageio.get_writer(self.video_path, fps=self.motion_fps)
            print(f"Recording video to {self.video_path}")
            self.renderer = mj.Renderer(self.model, height=video_height, width=video_width)
            self._annotate_warned = False
            try:
                from PIL import Image, ImageDraw, ImageFont  # noqa: F401
                self._have_pil = True
            except Exception:
                self._have_pil = False
        else:
            self._have_pil = False

    def step(
        self,
        root_pos: np.ndarray,
        root_rot: np.ndarray,
        dof_pos: np.ndarray,
        human_motion_data: Optional[Dict[str, Tuple[np.ndarray, np.ndarray]]] = None,
        show_human_body_name: bool = False,
        human_point_scale: float = 0.1,
        human_pos_offset: Optional[np.ndarray] = None,
        rate_limit: bool = True,
        follow_camera: bool = True,
        show_robot_coords: Optional[bool] = None,
        show_orientation: Optional[bool] = None,
        show_quat: Optional[bool] = None,
    ):
        # State update
        self.data.qpos[:3] = root_pos
        self.data.qpos[3:7] = root_rot  # w,x,y,z
        self.data.qpos[7:] = dof_pos
        mj.mj_forward(self.model, self.data)

        # Camera
        if follow_camera and self.camera_follow:
            self.viewer.cam.lookat = self.data.xpos[self.model.body(self.robot_base).id]
            self.viewer.cam.distance = (
                self.camera_distance_override
                if self.camera_distance_override is not None
                else self.viewer_cam_distance
            )
            self.viewer.cam.elevation = -10

        # Human reference
        if human_motion_data is not None:
            self.viewer.user_scn.ngeom = 0
            offset = human_pos_offset if human_pos_offset is not None else np.zeros(3)
            for name, (pos, quat_wxyz) in human_motion_data.items():
                rotm = R.from_quat(quat_wxyz, scalar_first=True).as_matrix()
                draw_frame(
                    pos,
                    rotm,
                    self.viewer,
                    human_point_scale,
                    joint_name=name if show_human_body_name else None,
                    pos_offset=offset,
                )

        # Effective flags
        if show_robot_coords is None:
            show_robot_coords = self.show_robot_coords
        if show_orientation is None:
            show_orientation = self.show_orientation
        if show_quat is None:
            show_quat = self.show_quat

        # Velocity
        if self._last_pos is None:
            vel = np.zeros(3)
        else:
            dt = 1.0 / self.motion_fps if self.motion_fps > 0 else 0.0
            vel = (root_pos - self._last_pos) / dt if dt > 0 else np.zeros(3)
        self._last_pos = root_pos.copy()
        self._last_vel = vel

        # Overlay lines
        lines = []
        if show_robot_coords:
            lines.append(f"XYZ: {root_pos[0]:.3f} {root_pos[1]:.3f} {root_pos[2]:.3f}")
            lines.append(f"Vel: {vel[0]:.3f} {vel[1]:.3f} {vel[2]:.3f}")
        if show_orientation:
            try:
                yaw, pitch, roll = R.from_quat(root_rot, scalar_first=True).as_euler(
                    "zyx", degrees=True
                )
                lines.append(f"Yaw/Pitch/Roll: {yaw:+.1f} {pitch:+.1f} {roll:+.1f}")
            except Exception:
                pass
        if show_quat:
            lines.append(
                f"Quat wxyz: {root_rot[0]:+.3f} {root_rot[1]:+.3f} {root_rot[2]:+.3f} {root_rot[3]:+.3f}"
            )
        if self.show_root_diff and human_motion_data is not None:
            hips_name = next(
                (
                    n
                    for n in [
                        "Hips",
                        "CC_Base_Pelvis",
                        "CC_Base_Hip",
                        "CC_Base_BoneRoot",
                    ]
                    if n in human_motion_data
                ),
                None,
            )
            if hips_name:
                offset = human_pos_offset if human_pos_offset is not None else np.zeros(3)
                h_pos = human_motion_data[hips_name][0] + offset
                d = root_pos - h_pos
                lines.append(f"Δroot (R-H) XYZ: {d[0]:+.3f} {d[1]:+.3f} {d[2]:+.3f}")

        if lines:
            try:
                self.viewer.add_overlay(
                    mj.mjtGridPos.mjGRID_TOPLEFT, "Robot", "\n".join(lines)
                )
            except Exception:
                pass

        # CSV
        if self._traj_fp:
            t = self._frame_idx / self.motion_fps if self.motion_fps > 0 else 0.0
            try:
                yaw, pitch, roll = R.from_quat(root_rot, scalar_first=True).as_euler(
                    "zyx", degrees=True
                )
            except Exception:
                yaw = pitch = roll = 0.0
            self._traj_fp.write(
                f"{self._frame_idx},{t:.6f},{root_pos[0]:.6f},{root_pos[1]:.6f},{root_pos[2]:.6f},"
                f"{vel[0]:.6f},{vel[1]:.6f},{vel[2]:.6f},{root_rot[0]:.6f},{root_rot[1]:.6f},"
                f"{root_rot[2]:.6f},{root_rot[3]:.6f},{yaw:.6f},{pitch:.6f},{roll:.6f}\n"
            )
        self._frame_idx += 1

        # Sync & rate limit
        self.viewer.sync()
        if rate_limit:
            self.rate_limiter.sleep()

        # Video
        if self.record_video:
            self.renderer.update_scene(self.data, camera=self.viewer.cam)
            img = self.renderer.render()
            if self.record_overlay_text and lines:
                if self._have_pil:
                    try:
                        from PIL import Image, ImageDraw, ImageFont

                        im = Image.fromarray(img)
                        draw = ImageDraw.Draw(im)
                        try:
                            font = ImageFont.truetype("Arial.ttf", 14)
                        except Exception:
                            font = ImageFont.load_default()
                        text = "\n".join(lines)
                        pad = 4
                        bbox = draw.multiline_textbbox((0, 0), text, font=font)
                        w = bbox[2] - bbox[0] + pad * 2
                        h = bbox[3] - bbox[1] + pad * 2
                        draw.rectangle([0, 0, w, h], fill=(0, 0, 0, 160))
                        draw.multiline_text((pad, pad), text, fill=(255, 255, 255), font=font)
                        img = np.asarray(im)
                    except Exception as e:
                        if not getattr(self, "_annotate_warned", False):
                            print(f"[yellow]Video overlay text failed: {e}[/yellow]")
                            self._annotate_warned = True
                else:
                    if not getattr(self, "_annotate_warned", False):
                        print(
                            "[yellow]Pillow не установлен: текст не войдёт в видео (pip install Pillow).[/yellow]"
                        )
                        self._annotate_warned = True
            self.mp4_writer.append_data(img)

    def close(self):
        self.viewer.close()
        time.sleep(0.5)
        if self.record_video:
            self.mp4_writer.close()
            print(f"Video saved to {getattr(self, 'video_path', '?')}")
        if self._traj_fp:
            try:
                self._traj_fp.close()
            except Exception:
                pass

__all__ = ["RobotMotionViewer", "draw_frame"]
