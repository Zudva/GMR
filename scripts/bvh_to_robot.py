import argparse
import pathlib
import time
from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting import RobotMotionViewer
from general_motion_retargeting.utils.lafan1 import load_lafan1_file
from rich import print
from tqdm import tqdm
import os
import numpy as np

# --- Synonym mapping for generic / ActorCore / CC_Base style BVH names ---
SYNONYM_MAP = {
    "Hips": ["CC_Base_Pelvis","CC_Base_Hip","CC_Base_BoneRoot","Pelvis","Root","root"],
    "Spine1": ["CC_Base_Waist","CC_Base_Spine01","CC_Base_Spine02","Spine","Spine01","Spine02"],
    "LeftUpLeg": ["CC_Base_L_Thigh","LeftUpLeg","LeftThigh"],
    "RightUpLeg": ["CC_Base_R_Thigh","RightUpLeg","RightThigh"],
    "LeftLeg": ["CC_Base_L_Calf","LeftLeg","LeftCalf","LeftLowerLeg"],
    "RightLeg": ["CC_Base_R_Calf","RightLeg","RightCalf","RightLowerLeg"],
    "LeftToeBase": ["CC_Base_L_ToeBase","LeftToeBase","LeftToe"],
    "RightToeBase": ["CC_Base_R_ToeBase","RightToeBase","RightToe"],
    "LeftArm": ["CC_Base_L_Upperarm","LeftArm","LeftUpperArm"],
    "RightArm": ["CC_Base_R_Upperarm","RightArm","RightUpperArm"],
    "LeftForeArm": ["CC_Base_L_Forearm","LeftForeArm","LeftLowerArm","LeftElbow"],
    "RightForeArm": ["CC_Base_R_Forearm","RightForeArm","RightLowerArm","RightElbow"],
    "LeftHand": ["CC_Base_L_Hand","LeftHand","LeftWrist"],
    "RightHand": ["CC_Base_R_Hand","RightHand","RightWrist"],
}

REQUIRED_FOR_IK = [
    "Hips","Spine1","LeftUpLeg","RightUpLeg","LeftLeg","RightLeg",
    "LeftToeBase","RightToeBase","LeftArm","RightArm","LeftForeArm","RightForeArm","LeftHand","RightHand"
]

def _fill_synonyms(frames):
    missing_any = False
    for frame in frames:
        for tgt in REQUIRED_FOR_IK:
            if tgt in frame:
                continue
            for cand in SYNONYM_MAP.get(tgt, []):
                if cand in frame:
                    frame[tgt] = frame[cand]
                    break
            if tgt not in frame:
                missing_any = True
    if missing_any:
        # Only print once (first frame) to avoid spam
        present = frames[0].keys() if frames else []
        missing = [t for t in REQUIRED_FOR_IK if t not in frames[0]] if frames else REQUIRED_FOR_IK
        print(f"[yellow]Warning: some required joints still missing after synonym fill: {missing}[/yellow]")
    return frames

def _synthesize_foot_mod(frames):
    """Create LeftFootMod / RightFootMod if absent by combining foot position with toe orientation.
    Expect patterns like CC_Base_L_Foot + CC_Base_L_ToeBase. Fallback: use foot's own orientation.
    """
    if not frames:
        return frames
    left_foot_names = ["LeftFoot","CC_Base_L_Foot"]
    left_toe_names = ["LeftToe","LeftToeBase","CC_Base_L_ToeBase"]
    right_foot_names = ["RightFoot","CC_Base_R_Foot"]
    right_toe_names = ["RightToe","RightToeBase","CC_Base_R_ToeBase"]
    created = {"L": False, "R": False}
    for f in frames:
        if "LeftFootMod" not in f:
            foot = next((n for n in left_foot_names if n in f), None)
            toe = next((n for n in left_toe_names if n in f), None)
            if foot:
                pos = f[foot][0]
                rot = f[toe][1] if toe else f[foot][1]
                f["LeftFootMod"] = (pos, rot)
                created["L"] = True
        if "RightFootMod" not in f:
            foot = next((n for n in right_foot_names if n in f), None)
            toe = next((n for n in right_toe_names if n in f), None)
            if foot:
                pos = f[foot][0]
                rot = f[toe][1] if toe else f[foot][1]
                f["RightFootMod"] = (pos, rot)
                created["R"] = True
    if created["L"] or created["R"]:
        print(f"[cyan]Synthesized foot mods: Left={created['L']} Right={created['R']}[/cyan]")
    return frames

def _synthesize_spine2(frames):
    """Ensure a Spine2 joint exists if IK config expects it (copy Spine1 / closest)."""
    if not frames:
        return frames
    created = False
    for f in frames:
        if "Spine2" not in f:
            # candidate sources in priority order
            cand_names = ["Spine1","Spine","CC_Base_Spine02","CC_Base_Spine01","CC_Base_Waist"]
            src = next((n for n in cand_names if n in f), None)
            if src:
                f["Spine2"] = f[src]
                created = True
    if created:
        print("[cyan]Synthesized Spine2 from available spine bone(s).[cyan]")
    return frames

if __name__ == "__main__":
    
    HERE = pathlib.Path(__file__).parent

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--bvh_file",
        help="BVH motion file to load.",
        required=True,
        type=str,
    )
    
    parser.add_argument(
        "--robot",
        choices=["unitree_g1", "unitree_g1_with_hands", "booster_t1", "stanford_toddy", "fourier_n1", "engineai_pm01"],
        default="unitree_g1",
    )
        
    parser.add_argument(
        "--record_video",
        action="store_true",
        default=False,
    )

    parser.add_argument(
        "--video_path",
        type=str,
        default="videos/example.mp4",
    )

    parser.add_argument(
        "--rate_limit",
        action="store_true",
        default=False,
    )

    parser.add_argument(
        "--solver",
        type=str,
        default="daqp",
        choices=["daqp", "proxqp", "osqp", "quadprog", "cvxopt"],
        help="QP solver to use inside IK (use 'osqp' if you hit install issues on macOS arm64)",
    )

    parser.add_argument(
        "--save_path",
        default=None,
        help="Path to save the robot motion.",
    )

    parser.add_argument(
        "--print_bvh_bones",
        action="store_true",
        help="Print the list of bone (joint) names found in the BVH file and exit (useful for mapping/debug).",
    )

    parser.add_argument(
        "--no_viewer",
        action="store_true",
        help="Run headless (no MuJoCo viewer). Useful for generating motion file on macOS without mjpython.",
    )
    parser.add_argument(
        "--dump_first_frame_json",
        type=str,
        default=None,
        help="Path to write first processed frame (after synonym & synthesis) as JSON for debugging joint names/orientation.",
    )
    
    args = parser.parse_args()

    # Quick bone listing / debug mode
    if args.print_bvh_bones:
        from general_motion_retargeting.utils.lafan_vendor.extract import read_bvh as _read_bvh
        data_dbg = _read_bvh(args.bvh_file)
        print("[cyan]BVH bone names (in order):[/cyan]")
        for i, b in enumerate(data_dbg.bones):
            print(f"{i:03d}: {b}")
        print("Total bones:", len(data_dbg.bones))
        exit(0)

    if args.save_path is not None:
        save_dir = os.path.dirname(args.save_path)
        if save_dir:  # Only create directory if it's not empty
            os.makedirs(save_dir, exist_ok=True)
        qpos_list = []

    
    # Load SMPLX trajectory
    lafan1_data_frames, actual_human_height = load_lafan1_file(args.bvh_file)
    lafan1_data_frames = _fill_synonyms(lafan1_data_frames)
    lafan1_data_frames = _synthesize_foot_mod(lafan1_data_frames)
    lafan1_data_frames = _synthesize_spine2(lafan1_data_frames)

    if args.dump_first_frame_json and lafan1_data_frames:
        import json as _json
        f0 = lafan1_data_frames[0]
        # Convert numpy arrays to lists
        serializable = {k: {"pos": list(map(float, v[0])), "rot(wxyz)": list(map(float, v[1]))} for k,v in f0.items()}
        with open(args.dump_first_frame_json, 'w') as jf:
            _json.dump(serializable, jf, indent=2)
        print(f"[cyan]Dumped first frame to {args.dump_first_frame_json}[/cyan]")
    
    
    # Initialize the retargeting system
    retargeter = GMR(
        src_human="bvh",
        tgt_robot=args.robot,
        actual_human_height=actual_human_height,
    solver=args.solver,
    )

    motion_fps = 30
    
    robot_motion_viewer = None
    if not args.no_viewer:
        try:
            robot_motion_viewer = RobotMotionViewer(robot_type=args.robot,
                                                    motion_fps=motion_fps,
                                                    transparent_robot=0,
                                                    record_video=args.record_video,
                                                    video_path=args.video_path,
                                                    )
        except RuntimeError as e:
            print(f"[yellow]Viewer disabled due to error: {e}. Continuing headless. (Tip: run with 'mjpython' on macOS to enable viewer.)[/yellow]")
            robot_motion_viewer = None
    # FPS measurement variables
    fps_counter = 0
    fps_start_time = time.time()
    fps_display_interval = 2.0  # Display FPS every 2 seconds
    
    print(f"mocap_frame_rate: {motion_fps}")
    
    # Create tqdm progress bar for the total number of frames
    pbar = tqdm(total=len(lafan1_data_frames), desc="Retargeting")
    
    # Start the viewer
    i = 0

    while i < len(lafan1_data_frames):
        
        # FPS measurement
        fps_counter += 1
        current_time = time.time()
        if current_time - fps_start_time >= fps_display_interval:
            actual_fps = fps_counter / (current_time - fps_start_time)
            print(f"Actual rendering FPS: {actual_fps:.2f}")
            fps_counter = 0
            fps_start_time = current_time
            
        # Update progress bar
        pbar.update(1)

        # Update task targets.
        smplx_data = lafan1_data_frames[i]

        # retarget
        qpos = retargeter.retarget(smplx_data)

        # visualize
        if robot_motion_viewer is not None:
            robot_motion_viewer.step(
                root_pos=qpos[:3],
                root_rot=qpos[3:7],
                dof_pos=qpos[7:],
                human_motion_data=retargeter.scaled_human_data,
                rate_limit=args.rate_limit,
            )

        i += 1

        if args.save_path is not None:
            qpos_list.append(qpos)
    
    if args.save_path is not None:
        import pickle
        root_pos = np.array([qpos[:3] for qpos in qpos_list])
        # save from wxyz to xyzw
        root_rot = np.array([qpos[3:7][[1,2,3,0]] for qpos in qpos_list])
        dof_pos = np.array([qpos[7:] for qpos in qpos_list])
        local_body_pos = None
        body_names = None
        
        motion_data = {
            "fps": motion_fps,
            "root_pos": root_pos,
            "root_rot": root_rot,
            "dof_pos": dof_pos,
            "local_body_pos": local_body_pos,
            "link_body_list": body_names,
        }
        with open(args.save_path, "wb") as f:
            pickle.dump(motion_data, f)
        print(f"Saved to {args.save_path}")

    # Close progress bar
    pbar.close()
    
    if robot_motion_viewer is not None:
        robot_motion_viewer.close()
       
