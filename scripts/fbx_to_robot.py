"""Offline FBX -> Robot retargeting helper.

This script provides a pragmatic path to retarget an OptiTrack (or other) FBX file
to a supported robot (e.g. unitree_g1) using the existing GMR IK configs for "fbx".

Because a native pure-Python FBX skeletal animation extractor is not bundled in
this repo, the reliable cross‑platform approach here is:
  1. (Optionally) invoke Blender in background to convert FBX -> BVH.
  2. Parse the produced BVH with the existing LAFAN1 BVH parser utilities.
  3. Feed frames into GeneralMotionRetargeting with src_human="fbx" so that the
     fbx_to_<robot>.json IK config is used.

If you ALREADY have a BVH exported from the FBX (with matching joint names to
those in fbx_to_g1.json) you can skip Blender and pass --bvh_file directly.

Joint naming expectations (see ik_configs/fbx_to_g1.json):
  Hips, Spine1, LeftUpLeg, RightUpLeg, LeftLeg, RightLeg, LeftToeBase, RightToeBase,
  LeftArm, RightArm, LeftForeArm, RightForeArm, LeftHand, RightHand.

Tested path on macOS (arm64) with Blender 4.x.

Usage examples:

  # Convert FBX with Blender auto and retarget (viewer pops up)
  MUJOCO_GL=glfw mjpython scripts/fbx_to_robot.py \
    --fbx_file data/optitrack/sample.fbx \
    --robot unitree_g1 --solver osqp --rate_limit --use_blender

  # If you already converted
  MUJOCO_GL=glfw mjpython scripts/fbx_to_robot.py \
    --bvh_file data/optitrack/sample_converted.bvh \
    --robot unitree_g1 --solver osqp

Notes:
  * On macOS you MUST use mjpython for the viewer (MuJoCo requirement).
  * Blender path defaults to /Applications/Blender.app/Contents/MacOS/Blender
    (override via --blender_path if different / on Linux / Windows).
  * Conversion produces a temporary BVH unless you set --out_bvh.
  * If conversion fails, you'll get a detailed hint. You can still convert
    manually inside Blender GUI and then re-run with --bvh_file.
"""
from __future__ import annotations

import argparse
import subprocess
import sys
import tempfile
import pathlib
import os
from typing import Optional
import numpy as np
from scipy.spatial.transform import Rotation as R
import math, re

from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting import RobotMotionViewer
from general_motion_retargeting.params import ROBOT_XML_DICT
from general_motion_retargeting.utils.lafan1 import load_lafan1_file
import general_motion_retargeting.utils.lafan_vendor.utils as _lafan_utils
from general_motion_retargeting.utils.lafan_vendor.extract import read_bvh as _read_bvh
from rich import print

def convert_fbx_to_bvh_with_blender(
    blender_exec: str,
    fbx_path: pathlib.Path,
    out_bvh: pathlib.Path,
    force: bool = False,
    inspect_only: bool = False,
    debug: bool = False,
    armature_name: Optional[str] = None,
    factory_startup: bool = False,
    keep_temp_script: bool = False,
    stream_output: bool = False,
) -> None:
    """Invoke Blender in background mode to convert an FBX to BVH or just inspect it.

    We drop a temporary Python script instead of using --python-expr to make the logic
    clearer and more robust (long expressions were getting truncated). The script:
      * Imports the FBX with animation baking enabled.
      * Expands scene frame range to cover all action fcurves.
      * Optionally prints diagnostic info (object types, actions, frame ranges).
      * Bakes POSE actions again explicitly (visual transforms) to ensure keyframes.
      * Exports BVH unless inspect_only is True.
    """
    if out_bvh.exists() and not force and not inspect_only:
        print(f"[yellow]BVH already exists, skipping conversion: {out_bvh}[/yellow]")
        return
    tmp_script = tempfile.NamedTemporaryFile("w", suffix="_fbx2bvh.py", delete=False)
    script_path = pathlib.Path(tmp_script.name)
    INSPECT = str(bool(inspect_only))
    # NOTE: Inside this f-string we must escape braces for the inner Python code by doubling them.
    content = f"""import bpy, json, os, sys, traceback
fbx=r"{fbx_path.as_posix()}"
out=r"{out_bvh.as_posix()}"
arm_name={repr(armature_name)}
INSPECT_ONLY={INSPECT}
print('[BLENDER] Importing FBX:', fbx)
try:
    ok=bpy.ops.import_scene.fbx(filepath=fbx, automatic_bone_orientation=True, use_anim=True)
    if 'FINISHED' not in ok: raise RuntimeError(ok)
except Exception as e:
    traceback.print_exc()
    print('[BLENDER_DIAG]', json.dumps({{'error':'import_failed','exc':str(e)}}))
    sys.exit(0)
scn=bpy.context.scene
# Collect original actions info
orig_actions=[a.name for a in bpy.data.actions]
print('[BLENDER] Actions detected after import:', orig_actions)
if bpy.data.actions:
    fmins=[]; fmaxs=[]
    for act in bpy.data.actions:
        for fcu in act.fcurves:
            r=fcu.range(); fmins.append(r[0]); fmaxs.append(r[1])
    if fmins and fmaxs:
        scn.frame_start=int(min(fmins)); scn.frame_end=int(max(fmaxs))
print('[BLENDER] Frame range', scn.frame_start, scn.frame_end)
arms=[o for o in bpy.context.scene.objects if o.type=='ARMATURE']
if arm_name is not None:
    sel=[a for a in arms if a.name==arm_name]
    if sel: arms=sel
    else: print('[BLENDER] WARNING requested armature not found; using all')
print('[BLENDER] Armatures:', [a.name for a in arms])
if not arms: print('[BLENDER] WARNING no armature found')
for a in arms:
    bpy.context.view_layer.objects.active=a; a.select_set(True)
# Ensure each armature has an active action (some FBX imports leave NLA strips only)
for a in arms:
    ad=a.animation_data
    if not ad or not ad.action:
        candidate=None
        bone_names={{b.name for b in a.data.bones}}
        for act in bpy.data.actions:
            for fcu in act.fcurves:
                dp=fcu.data_path
                if dp.startswith('pose.bones'):
                    try:
                        bn=dp.split('\"')[1]
                    except Exception:
                        continue
                    if bn in bone_names:
                        candidate=act; break
            if candidate: break
        if candidate is not None:
            if not a.animation_data:
                a.animation_data_create()
            a.animation_data.action=candidate
            print('[BLENDER] Assigned action', candidate.name, 'to armature', a.name)
        else:
            print('[BLENDER] WARNING: No candidate action found for armature', a.name)
if arms:
    try:
        print('[BLENDER] Baking (POSE)...')
        bpy.ops.nla.bake(frame_start=scn.frame_start, frame_end=scn.frame_end, only_selected=False, visual_keying=True, clear_constraints=False, use_current_action=True, bake_types={{'POSE'}})
    except Exception as e:
        print('[BLENDER] BAKE ERROR', e)
# Report active actions after bake
for a in arms:
    ad=a.animation_data
    print('[BLENDER] Armature', a.name, 'active action:', (ad.action.name if (ad and ad.action) else 'NONE'))
info={{'objects':{{}}}}
for o in bpy.context.scene.objects:
    od={{'type':o.type}}
    if o.animation_data and o.animation_data.action: od['action']=o.animation_data.action.name
    info['objects'][o.name]=od
info['actions']=[a.name for a in bpy.data.actions]
print('[BLENDER_DIAG]', json.dumps(info))
if not INSPECT_ONLY:
    print('[BLENDER] Export phase...')
    try:
        bpy.ops.object.mode_set(mode='OBJECT')
    except Exception: pass
    for o in bpy.context.selected_objects: o.select_set(False)
    for a in arms: a.select_set(True); bpy.context.view_layer.objects.active=a
    attempts=[
        {{ }},
        {{'root_transform_only':False,'frame_start':scn.frame_start,'frame_end':scn.frame_end,'rotate_mode':'NATIVE'}},
        {{'root_transform_only':False,'frame_start':scn.frame_start,'frame_end':scn.frame_end,'rotate_mode':'XYZ'}},
    ]
    success=False
    for idx,kw in enumerate(attempts):
        try:
            r=bpy.ops.export_anim.bvh(filepath=out, **kw)
            print('[BLENDER] export attempt', idx, 'result', r, kw)
            if 'FINISHED' in r:
                success=True; break
        except Exception as e:
            print('[BLENDER] export attempt', idx, 'exception', e, kw)
    if not success:
        print('[BLENDER] ERROR: All export attempts failed.')
    print('[BLENDER] Exists?', os.path.exists(out))
"""
    tmp_script.write(content)
    tmp_script.flush(); tmp_script.close()
    cmd = [blender_exec]
    if factory_startup:
        cmd.append("--factory-startup")
    cmd += ["-b", "--python", str(script_path)]
    print(f"[cyan]Converting FBX -> BVH with Blender: {' '.join(cmd)}[/cyan]")
    print(f"[cyan]Temp Blender script: {script_path} (keep={keep_temp_script})[/cyan]")
    try:
        if stream_output:
            # Let Blender output stream directly to terminal for full visibility
            res = subprocess.run(cmd, text=True)
        else:
            res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    except FileNotFoundError:
        raise RuntimeError(
            f"Blender executable not found: {blender_exec}. Provide --blender_path or install Blender." )
    if res.returncode != 0:
        print("[red]Blender conversion failed[/red]")
        if not stream_output:
            print("--- stdout ---\n" + res.stdout)
            print("--- stderr ---\n" + res.stderr)
        raise RuntimeError("FBX->BVH conversion failed; see logs above.")
    if inspect_only:
        print("[green]Inspection finished (no BVH export requested). See [BLENDER_DIAG] output above.[/green]")
    else:
        if not out_bvh.exists():
            if not stream_output:
                print(res.stdout)
            raise RuntimeError("Conversion reported success but BVH file not found.")
        print(f"[green]FBX converted to BVH: {out_bvh}[/green]")
    if debug and not stream_output:
        # Provide a larger slice for debugging
        print("--- Blender stdout (first 12k) ---")
        print(res.stdout[:12000])
    # Cleanup temp script unless kept
    if not keep_temp_script:
        try:
            script_path.unlink()
        except Exception:
            pass
def _load_bvh_generic(bvh_file: pathlib.Path, axis_fix: bool=True):
    """Generic BVH loader tolerant to differing bone naming (Blender export).

    Returns frames (list[dict[name] = (pos, quat_wxyz)]) and estimated human height.
    """
    try:
        data = _read_bvh(str(bvh_file))
        global_data = _lafan_utils.quat_fk(data.quats, data.pos, data.parents)
        use_loose = False
    except Exception as e:
        print(f"[yellow]lafan vendor parser failed ({e!r}); using loose BVH parser fallback[/yellow]")
        return _parse_bvh_loose(bvh_file, axis_fix=axis_fix)

    # Coordinate adjustment similar to lafan1 (Y-up to Z-up etc.)
    if axis_fix:
        base_matrix = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])  # Y-up -> Z-up
    else:
        base_matrix = np.eye(3)
    global ORIENT_FIX_QUAT
    if 'ORIENT_FIX_QUAT' not in globals() or ORIENT_FIX_QUAT is None:
        extra_matrix = np.eye(3)
    else:
        # ORIENT_FIX_QUAT stored scalar-first (w,x,y,z)
        q = ORIENT_FIX_QUAT
        rq = R.from_quat([q[1],q[2],q[3],q[0]])  # convert to (x,y,z,w)
        extra_matrix = rq.as_matrix()
    # Apply base (axis) then extra orientation: combined = extra * base
    combined_matrix = extra_matrix @ base_matrix
    combined_quat = R.from_matrix(combined_matrix).as_quat(scalar_first=True)
    frames = []
    bones = data.bones

    for frame_idx in range(data.pos.shape[0]):
        result = {}
        for i, bone in enumerate(bones):
            # Rotate orientation into target axis convention
            # quat_mul(x,y) returns y ⊗ x (see lafan utils); we want combined ⊗ original -> pass original as first arg
            orientation = _lafan_utils.quat_mul(global_data[0][frame_idx, i], combined_quat)
            position = global_data[1][frame_idx, i] @ combined_matrix.T / 100.0
            result[bone] = (position, orientation)
        frames.append(result)

    # Estimate height using available bones
    last_frame = frames[-1]
    hip_name = next((n for n in ["Hips","hips","Root","root"] if n in last_frame), None)
    head_name = next((n for n in ["Head","head"] if n in last_frame), None)
    if hip_name and head_name:
        human_height = last_frame[head_name][0][2] - last_frame[hip_name][0][2] + 0.9
    else:
        human_height = 1.75
    return frames, human_height

def _parse_bvh_loose(bvh_file: pathlib.Path, axis_fix: bool=True):
    """Very permissive BVH parser producing world positions & quaternions.

    Handles arbitrary channel counts/order; only uses rotation & (root) translation.
    """
    txt = bvh_file.read_text(encoding='utf-8', errors='ignore').splitlines()
    joints = []  # list of dicts: name, parent, channels(list), offset(np.array)
    parents = []
    channel_specs = []
    offsets = []
    stack = []
    motion_start_idx = None
    i = 0
    while i < len(txt):
        line = txt[i].strip()
        if line.upper().startswith('MOTION'):
            motion_start_idx = i + 1
            break
        if line.startswith('ROOT') or line.startswith('JOINT'):
            parts = line.split()
            name = parts[1]
            parent = stack[-1] if stack else -1
            joints.append(name)
            parents.append(parent)
            offsets.append(np.zeros(3))
            channel_specs.append([])
            stack.append(len(joints)-1)
            i += 1
            continue
        if line.startswith('End Site'):
            # skip End Sites completely
            stack.append(-2)  # marker
            i += 1
            continue
        if line.startswith('OFFSET'):
            vals = [float(v) for v in line.split()[1:4]]
            if stack and stack[-1] >= 0:
                offsets[stack[-1]] = np.array(vals, dtype=float)
            i += 1
            continue
        if line.startswith('CHANNELS'):
            parts = line.split()
            nchan = int(parts[1])
            chans = parts[2:2+nchan]
            if stack and stack[-1] >= 0:
                channel_specs[stack[-1]] = chans
            i += 1
            continue
        if line.startswith('}'):
            if stack:
                stack.pop()
            i += 1
            continue
        i += 1
    if motion_start_idx is None:
        raise RuntimeError('BVH missing MOTION section')
    # Parse MOTION header
    frame_count = 0
    frame_time = 1/30
    for j in range(motion_start_idx, min(motion_start_idx+5, len(txt))):
        l = txt[j].strip()
        if l.startswith('Frames:'):
            frame_count = int(l.split()[1])
        if l.lower().startswith('frame time:'):
            frame_time = float(l.split()[2])
    # Find first frame line index
    first_frame_line = None
    for j in range(motion_start_idx, len(txt)):
        if re.match(r'^\s*Frames:', txt[j]):
            continue
        if re.match(r'^\s*Frame Time:', txt[j], re.IGNORECASE):
            first_frame_line = j+1
            break
    if first_frame_line is None:
        raise RuntimeError('Could not locate frame data start')
    # Precompute channel offsets per joint
    joint_channel_start = []
    cursor = 0
    for chans in channel_specs:
        joint_channel_start.append(cursor)
        cursor += len(chans)
    total_channels = cursor
    frames = []
    rx = lambda a: R.from_rotvec([math.radians(a),0,0])
    ry = lambda a: R.from_rotvec([0,math.radians(a),0])
    rz = lambda a: R.from_rotvec([0,0,math.radians(a)])
    axis_map = {'Xrotation': ('x', rx), 'Yrotation': ('y', ry), 'Zrotation': ('z', rz)}
    # Axis fix rotation quaternion/matrix
    if axis_fix:
        base_m = np.array([[1,0,0],[0,0,-1],[0,1,0]])
    else:
        base_m = np.eye(3)
    global ORIENT_FIX_QUAT
    if 'ORIENT_FIX_QUAT' not in globals() or ORIENT_FIX_QUAT is None:
        extra_m = np.eye(3)
    else:
        q = ORIENT_FIX_QUAT
        rq = R.from_quat([q[1],q[2],q[3],q[0]])
        extra_m = rq.as_matrix()
    comb_m = extra_m @ base_m
    comb_q = R.from_matrix(comb_m).as_quat(scalar_first=True)
    for fidx in range(frame_count):
        line_idx = first_frame_line + fidx
        if line_idx >= len(txt):
            break
        vals_str = txt[line_idx].strip().split()
        if len(vals_str) < total_channels:
            # Possibly blank or truncated line; skip
            continue
        vals = list(map(float, vals_str[:total_channels]))
        world_pos = [None]*len(joints)
        world_rot = [None]*len(joints)
        for j_idx, name in enumerate(joints):
            chans = channel_specs[j_idx]
            cstart = joint_channel_start[j_idx]
            cvals = vals[cstart:cstart+len(chans)]
            t = np.zeros(3)
            rot = R.identity()
            # Build local transform
            for cname, cval in zip(chans, cvals):
                if cname.endswith('position'):
                    axis = cname[0].upper()
                    if axis == 'X': t[0] = cval
                    elif axis == 'Y': t[1] = cval
                    elif axis == 'Z': t[2] = cval
                elif cname in axis_map:
                    rot = rot * axis_map[cname][1](cval)
            # Append static offset (converted to meters later)
            offset = offsets[j_idx]
            if parents[j_idx] == -1:  # root
                local_pos = t + offset
                local_rot = rot
                parent_pos = np.zeros(3)
                parent_rot = R.identity()
            else:
                parent_pos = world_pos[parents[j_idx]]
                parent_rot = world_rot[parents[j_idx]]
                local_pos = parent_pos + parent_rot.apply(offset)
                local_rot = parent_rot * rot
            world_pos[j_idx] = local_pos
            world_rot[j_idx] = local_rot
        frame_dict = {}
        for j_idx, name in enumerate(joints):
            # Axis fix transform
            p = (world_pos[j_idx] @ comb_m.T)/100.0
            q = world_rot[j_idx].as_quat(scalar_first=True)
            q_fixed = _lafan_utils.quat_mul(q, comb_q)  # combined ⊗ original
            frame_dict[name] = (p, q_fixed)
        frames.append(frame_dict)
    # Height estimation
    if frames:
        lf = frames[-1]
        hip = next((n for n in ["Hips","hips","Root","root"] if n in lf), None)
        head = next((n for n in ["Head","head"] if n in lf), None)
        if hip and head:
            human_height = lf[head][0][2] - lf[hip][0][2] + 0.9
        else:
            human_height = 1.75
    else:
        human_height = 1.75
    return frames, human_height


SYNONYM_MAP = {
    "LeftUpLeg": ["LeftUpLeg","LeftUpperLeg","L_UpLeg","LeftThigh"],
    "RightUpLeg": ["RightUpLeg","RightUpperLeg","R_UpLeg","RightThigh"],
    "LeftLeg": ["LeftLeg","LeftLowerLeg","L_Leg","LeftShin"],
    "RightLeg": ["RightLeg","RightLowerLeg","R_Leg","RightShin"],
    "LeftToeBase": ["LeftToeBase","LeftToe","LeftToe_End","L_ToeBase"],
    "RightToeBase": ["RightToeBase","RightToe","RightToe_End","R_ToeBase"],
    "Spine1": ["Spine1","Spine","Spine2","Chest","Spine01"],
    "LeftArm": ["LeftArm","LeftUpperArm","L_Arm"],
    "RightArm": ["RightArm","RightUpperArm","R_Arm"],
    "LeftForeArm": ["LeftForeArm","LeftLowerArm","L_ForeArm","LeftElbow"],
    "RightForeArm": ["RightForeArm","RightLowerArm","R_ForeArm","RightElbow"],
    "LeftHand": ["LeftHand","L_Hand","LeftWrist"],
    "RightHand": ["RightHand","R_Hand","RightWrist"],
    "Hips": ["Hips","Root","hips","root","Pelvis","pelvis"],
}

# Extend synonyms for "dummy" / ActorCore / Reallusion style naming seen in provided JSON
_EXTENDED_SYNONYMS = {
    "Hips": ["pelvis","cc_base_pelvis"],
    "Spine1": ["spine_01","spine_02","spine_03","cc_base_body"],
    "LeftUpLeg": ["thigh_l","Thigh_L"],
    "RightUpLeg": ["thigh_r","Thigh_R"],
    "LeftLeg": ["calf_l","Calf_L"],
    "RightLeg": ["calf_r","Calf_R"],
    # For toes we map foot / ball as fallback (less accurate)
    "LeftToeBase": ["foot_l","ball_l"],
    "RightToeBase": ["foot_r","ball_r"],
    "LeftArm": ["upperarm_l","clavicle_l"],
    "RightArm": ["upperarm_r","clavicle_r"],
    "LeftForeArm": ["lowerarm_l"],
    "RightForeArm": ["lowerarm_r"],
    "LeftHand": ["hand_l"],
    "RightHand": ["hand_r"],
}
for k,v in _EXTENDED_SYNONYMS.items():
    SYNONYM_MAP.setdefault(k, [])
    # Prepend new synonyms so they are prioritized
    for name in v:
        if name not in SYNONYM_MAP[k]:
            SYNONYM_MAP[k].insert(0, name)

# Additional ActorCore CC_Base_* synonyms
_CC_BASE = {
    # Reordered preference: Pelvis first (better anatomical root), then Hip, then BoneRoot
    "Hips": ["CC_Base_Pelvis","CC_Base_Hip","CC_Base_BoneRoot"],
    "Spine1": ["CC_Base_Waist","CC_Base_Spine01","CC_Base_Spine02"],
    "LeftUpLeg": ["CC_Base_L_Thigh"],
    "RightUpLeg": ["CC_Base_R_Thigh"],
    "LeftLeg": ["CC_Base_L_Calf"],
    "RightLeg": ["CC_Base_R_Calf"],
    "LeftToeBase": ["CC_Base_L_ToeBase"],
    "RightToeBase": ["CC_Base_R_ToeBase"],
    "LeftArm": ["CC_Base_L_Upperarm"],
    "RightArm": ["CC_Base_R_Upperarm"],
    "LeftForeArm": ["CC_Base_L_Forearm"],
    "RightForeArm": ["CC_Base_R_Forearm"],
    "LeftHand": ["CC_Base_L_Hand"],
    "RightHand": ["CC_Base_R_Hand"],
}
for k,v in _CC_BASE.items():
    SYNONYM_MAP.setdefault(k, [])
    for name in v:
        if name not in SYNONYM_MAP[k]:
            SYNONYM_MAP[k].insert(0, name)

REQUIRED_FOR_IK = [
    "Hips","Spine1","LeftUpLeg","RightUpLeg","LeftLeg","RightLeg",
    "LeftToeBase","RightToeBase","LeftArm","RightArm","LeftForeArm","RightForeArm","LeftHand","RightHand"
]

def _fill_synonyms(frames):
    for frame in frames:
        for target in REQUIRED_FOR_IK:
            if target in frame:  # already present
                continue
            synonyms = SYNONYM_MAP.get(target, [])
            found = None
            for s in synonyms:
                if s in frame:
                    found = s
                    break
            if found is not None:
                frame[target] = frame[found]
    return frames

def load_frames_from_bvh(bvh_file: pathlib.Path, allow_missing: bool=False, axis_fix: bool=True):
    try:
        frames, human_height = load_lafan1_file(str(bvh_file))
    except Exception as e:
        print(f"[yellow]lafan1 loader unusable ({e!r}). Using generic BVH parser...[/yellow]")
        frames, human_height = _load_bvh_generic(bvh_file, axis_fix=axis_fix)
    frames = _fill_synonyms(frames)
    # Stats on missing
    missing_counts = {k:0 for k in REQUIRED_FOR_IK}
    for fr in frames:
        for k in REQUIRED_FOR_IK:
            if k not in fr:
                missing_counts[k]+=1
    if not allow_missing:
        filtered = [fr for fr in frames if all(k in fr for k in REQUIRED_FOR_IK)]
        if not filtered:
            union_names = sorted(set().union(*[set(fr.keys()) for fr in frames]))
            print("[red]No frames contain all required IK joints.[/red]")
            print("[cyan]Required:[/cyan]", REQUIRED_FOR_IK)
            print("[cyan]Available (union):[/cyan]", union_names)
            print("[cyan]Missing counts:[/cyan]", missing_counts)
            print("Hint: rename BVH bones OR re-run with --allow_missing_joints to fill placeholders.")
            raise RuntimeError("No frames contain all required IK joints after fallback mapping.")
        return filtered, human_height
    # Fill placeholders
    for fr in frames:
        if "Hips" in fr:
            hips_pos, hips_rot = fr["Hips"]
            for k in REQUIRED_FOR_IK:
                if k not in fr:
                    fr[k] = (hips_pos.copy(), hips_rot.copy())
    print("[yellow]Filled missing joints with Hips pose placeholders (reduced accuracy).[/yellow]")
    return frames, human_height

def retarget_sequence(
    frames, human_height: float, robot: str, solver: str, rate_limit: bool,
    record_video: bool, video_path: Optional[str]
):
    retargeter = GMR(
        src_human="fbx",
        tgt_robot=robot,
        actual_human_height=human_height,
        solver=solver,
        verbose=False,
    )
    fps = 30
    viewer = RobotMotionViewer(robot_type=robot,
                               motion_fps=fps,
                               record_video=record_video,
                               video_path=video_path)
    for frame in frames:
        qpos = retargeter.retarget(frame)
        viewer.step(
            root_pos=qpos[:3],
            root_rot=qpos[3:7],
            dof_pos=qpos[7:],
            human_motion_data=retargeter.scaled_human_data,
            rate_limit=rate_limit,
            follow_camera=True,
        )
    viewer.close()

def parse_args(argv=None):
    p = argparse.ArgumentParser(description="Retarget an FBX human motion to a robot")
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--fbx_file", type=str, help="Path to source FBX file")
    src.add_argument("--bvh_file", type=str, help="Path to pre-converted BVH file (skip conversion)")
    p.add_argument("--robot", type=str, default="unitree_g1", choices=list(ROBOT_XML_DICT.keys()))
    p.add_argument("--solver", type=str, default="osqp", choices=["osqp", "proxqp", "daqp", "quadprog", "cvxopt"], help="QP solver")
    p.add_argument("--rate_limit", action="store_true", help="Play at real-time rate")
    p.add_argument("--record_video", action="store_true")
    p.add_argument("--video_path", type=str, default=None)
    p.add_argument("--use_blender", action="store_true", help="Force Blender conversion even if --fbx_file given")
    p.add_argument("--blender_path", type=str, default="/Applications/Blender.app/Contents/MacOS/Blender")
    p.add_argument("--out_bvh", type=str, default=None, help="Explicit output BVH path (else temp beside FBX)")
    p.add_argument("--overwrite", action="store_true", help="Overwrite existing output BVH")
    p.add_argument("--print_bones", action="store_true", help="Print BVH joint list and exit")
    p.add_argument("--allow_missing_joints", action="store_true", help="Fill missing IK joints with placeholders instead of error")
    p.add_argument("--no_axis_fix", action="store_true", help="Disable axis (Y-up->Z-up) rotation; try if model lies down")
    p.add_argument("--inspect_fbx", action="store_true", help="Only inspect FBX (list objects/actions) via Blender and exit")
    p.add_argument("--blender_debug", action="store_true", help="Print first part of Blender stdout for debugging")
    p.add_argument("--armature_name", type=str, default=None, help="Name of armature to bake/export (if multiple present)")
    p.add_argument("--factory_startup", action="store_true", help="Run Blender with --factory-startup to avoid addon side-effects")
    p.add_argument("--debug_stats", action="store_true", help="Print motion variance stats for key joints before retargeting")
    p.add_argument("--keep_temp_script", action="store_true", help="Do not delete generated temporary Blender script (for debugging)")
    p.add_argument("--stream_blender", action="store_true", help="Stream Blender stdout/stderr directly instead of capturing")
    p.add_argument("--force_generic_loader", action="store_true", help="Skip lafan1 loader and use generic BVH parser always")
    p.add_argument("--pelvis_z_offset", type=str, default=None, help="Add constant Z offset (meters) to all human joints AFTER normalization to better match robot pelvis height; pass 'auto' to auto-compute from robot pelvis vs human hips first-frame heights")
    p.add_argument("--no_scale_human", action="store_true", help="Disable human limb scaling (set all human_scale_table factors to 1.0) for diagnostics")
    p.add_argument("--pelvis_pos_w1", type=float, default=None, help="Override pelvis position weight in ik_match_table1 (stage1) to pull root earlier")
    p.add_argument("--pelvis_pos_w2", type=float, default=None, help="Override pelvis position weight in ik_match_table2 (stage2)")
    p.add_argument("--task_error_breakdown", action="store_true", help="Print per-task error contributions every 60 frames for debugging")
    p.add_argument("--align_root_xy", type=str, default=None, help="Horizontally align (X,Y) human root to robot pelvis before IK. Use 'auto' to match robot pelvis first-frame xy; or provide 'x,y' numeric target world coords (meters). Applied AFTER pelvis_z_offset.")
    p.add_argument("--orient_fix", type=str, default="none", choices=["none","x90","x-90","y90","y-90","z180","auto"], help="Orientation fix: preset rotation or auto (detect up/forward)")
    p.add_argument("--auto_forward_axis", type=str, default="x", choices=["x","y"], help="Desired forward axis for --orient_fix auto (default x)")
    p.add_argument("--ubisoft_axes", action="store_true", help="Force lafan1(Ubisoft) axis convention only (Y-up->Z-up) and skip extra orient composition")
    p.add_argument("--normalize_root", action="store_true", help="Shift initial Hips to origin and set floor (min foot z) to 0")
    p.add_argument("--always_overwrite", action="store_true", help="Always re-convert FBX -> BVH even if output exists")
    p.add_argument("--log_errors", action="store_true", help="Print per-frame IK task errors and positional discrepancies")
    p.add_argument("--errors_csv", type=str, default=None, help="Path to CSV file to append per-frame errors")
    p.add_argument("--dump_bvh_header", type=int, default=0, help="Print first N lines of BVH file then continue")
    p.add_argument("--debug_alignment", action="store_true", help="Print joint positions at stages: initial -> after auto orient -> after normalization")
    p.add_argument("--traj_csv", type=str, default=None, help="Write per-frame root trajectory & velocity to CSV (viewer-level)")
    return p.parse_args(argv)

def main(argv=None):
    args = parse_args(argv)
    if args.fbx_file:
        fbx_path = pathlib.Path(args.fbx_file).expanduser().resolve()
        if not fbx_path.exists():
            if str(fbx_path).startswith('/path/to'):
                print(f"[red]FBX file not found: {fbx_path}[/red]  Hint: replace placeholder /path/to/file.fbx with a real path.")
            else:
                print(f"[red]FBX file not found: {fbx_path}[/red]")
            sys.exit(1)
        if args.out_bvh:
            out_bvh = pathlib.Path(args.out_bvh).expanduser().resolve()
        else:
            out_bvh = fbx_path.with_suffix(".converted.bvh")
        force_conv = args.use_blender or args.always_overwrite or not out_bvh.exists() or args.inspect_fbx
        if force_conv:
            convert_fbx_to_bvh_with_blender(
                blender_exec=args.blender_path,
                fbx_path=fbx_path,
                out_bvh=out_bvh,
                force=True if args.always_overwrite else args.overwrite,
                inspect_only=args.inspect_fbx,
                debug=args.blender_debug,
                armature_name=args.armature_name,
                factory_startup=args.factory_startup,
                keep_temp_script=args.keep_temp_script,
                stream_output=args.stream_blender,
            )
            if args.inspect_fbx:
                return
        bvh_file = out_bvh
    else:
        bvh_file = pathlib.Path(args.bvh_file).expanduser().resolve()
        if not bvh_file.exists():
            print(f"[red]BVH file not found: {bvh_file}[/red]")
            sys.exit(1)
    print(f"[green]Using BVH file: {bvh_file}[/green]")
    # Optional BVH header dump
    if args.dump_bvh_header > 0:
        try:
            with open(bvh_file, 'r', encoding='utf-8', errors='ignore') as fh:
                for i in range(args.dump_bvh_header):
                    line = fh.readline()
                    if not line:
                        break
                    print(f"[magenta]BVH[{i:03d}]:[/magenta] {line.rstrip()}")
        except Exception as e:
            print(f"[red]Failed to dump BVH header: {e}[/red]")
    # Compute orientation correction quaternion
    orient_map = {
        'none': np.array([1,0,0,0]),
        'x90': R.from_euler('x',90, degrees=True).as_quat(scalar_first=True),
        'x-90': R.from_euler('x',-90, degrees=True).as_quat(scalar_first=True),
        'y90': R.from_euler('y',90, degrees=True).as_quat(scalar_first=True),
        'y-90': R.from_euler('y',-90, degrees=True).as_quat(scalar_first=True),
        'z180': R.from_euler('z',180, degrees=True).as_quat(scalar_first=True),
        'auto': np.array([1,0,0,0]),  # placeholder; computed later
    }
    global ORIENT_FIX_QUAT
    ORIENT_FIX_QUAT = orient_map.get(args.orient_fix, np.array([1,0,0,0]))
    if args.orient_fix not in ('none','auto'):
        print(f"[cyan]Applying orientation fix quaternion preset ({args.orient_fix}): {ORIENT_FIX_QUAT}")
    if args.print_bones:
        d = _read_bvh(str(bvh_file))
        print("[cyan]Bones in BVH ({}):[/cyan]".format(len(d.bones)))
        for name in d.bones:
            print(name)
        return
    if args.force_generic_loader:
        print("[cyan]Forcing generic BVH loader...[/cyan]")
        frames, human_height = _load_bvh_generic(bvh_file, axis_fix=not args.no_axis_fix)
        frames = _fill_synonyms(frames)
    else:
        frames, human_height = load_frames_from_bvh(bvh_file, allow_missing=args.allow_missing_joints, axis_fix=not args.no_axis_fix)

    # Ubisoft axis mode: override any orient fix (keeps pure lafan1 transform)
    if args.ubisoft_axes:
        ORIENT_FIX_QUAT = np.array([1,0,0,0])
        if args.orient_fix not in ('none','auto'):
            print("[yellow]--ubisoft_axes overrides --orient_fix preset; ignoring preset rotation[/yellow]")
        # No post rotation adjustment applied (frames already rotated by axis fix earlier)

    # (Normalization moved after auto orientation below)

    # Auto orientation (post-load rotation). Rotate about initial hips pivot to avoid large translational drift.
    if args.orient_fix == 'auto':
        def _get(name, fr):
            return fr.get(name, (None,None))[0]
        ref = frames[min(10, len(frames)-1)]  # early frame for basis detection
        hips = _get('Hips', ref)
        head = _get('Head', ref) or _get('Spine1', ref)
        lhip = _get('LeftUpLeg', ref)
        rhip = _get('RightUpLeg', ref)
        if hips is not None and head is not None:
            up_vec = head - hips
        else:
            up_vec = np.array([0,0,1.0])
        if lhip is not None and rhip is not None:
            side_vec = lhip - rhip
        else:
            side_vec = np.array([1.0,0,0])
        # Normalize
        def _norm(v):
            n = np.linalg.norm(v)
            return v if n < 1e-8 else v / n
        up_vec = _norm(up_vec)
        # provisional forward = side x up
        fwd = np.cross(side_vec, up_vec)
        fwd = _norm(fwd)
        # Desired axes
        desired_up = np.array([0,0,1.0])
        desired_fwd = np.array([1.0,0,0]) if args.auto_forward_axis=='x' else np.array([0,1.0,0])
        # Build current basis (columns)
        right = _norm(np.cross(up_vec, fwd))
        if np.linalg.norm(right) < 1e-6:
            right = np.array([1,0,0])
        basis_cur = np.stack([fwd, right, up_vec], axis=1)  # columns: fwd, right, up
        # Desired basis
        desired_right = _norm(np.cross(desired_up, desired_fwd))
        basis_des = np.stack([desired_fwd, desired_right, desired_up], axis=1)
        # Rotation matrix taking current -> desired: R * basis_cur = basis_des => R = basis_des * basis_cur^T
        Rcorr = basis_des @ basis_cur.T
        # Orthonormalize via SVD to avoid drift
        U,S,Vt = np.linalg.svd(Rcorr)
        Rcorr = U @ Vt
        qcorr = R.from_matrix(Rcorr).as_quat(scalar_first=True)
        ORIENT_FIX_QUAT = qcorr
        print(f"[cyan]Auto orientation: up={up_vec}, fwd={fwd} -> quat={ORIENT_FIX_QUAT}")
        hip_pivot = frames[0]['Hips'][0].copy() if 'Hips' in frames[0] else np.zeros(3)
        # Debug capture initial before rotation
        if args.debug_alignment:
            keys_dbg = ['Hips','LeftHand','RightHand','LeftFoot','RightFoot','LeftToeBase','RightToeBase','LeftFootMod','RightFootMod']
            print('[magenta]Alignment stage: initial positions (first frame)')
            for k in keys_dbg:
                if k in frames[0]:
                    p = frames[0][k][0]
                    print(f"  {k:12s} {p}")
        # Apply rotation about hip pivot
        for fr in frames:
            for k,(p,q) in fr.items():
                p2 = (p - hip_pivot) @ Rcorr.T + hip_pivot
                q2 = _lafan_utils.quat_mul(q, qcorr)  # qcorr ⊗ q
                fr[k] = (p2, q2)
        if args.debug_alignment:
            print('[magenta]Alignment stage: after auto orientation (first frame)')
            for k in keys_dbg:
                if k in frames[0]:
                    p = frames[0][k][0]
                    print(f"  {k:12s} {p}")

    # Root normalization now (after orientation), keeps feet on floor and hips at origin
    if args.normalize_root and 'Hips' in frames[0]:
        root0 = frames[0]['Hips'][0].copy()
        feet_z = []
        for k in ["LeftToeBase","RightToeBase","LeftFoot","RightFoot","LeftFootMod","RightFootMod"]:
            if k in frames[0]:
                feet_z.append(frames[0][k][0][2])
        floor_z = min(feet_z) if feet_z else root0[2]
        dz = root0.copy(); dz[2] = floor_z
        for fr in frames:
            for name,(p,q) in fr.items():
                fr[name] = (p - dz, q)
        if args.debug_alignment:
            print('[magenta]Alignment stage: after normalization (first frame)')
            for k in ['Hips','LeftHand','RightHand','LeftFoot','RightFoot']:
                if k in frames[0]:
                    print(f"  {k:10s} {frames[0][k][0]}")
        print(f"[cyan]Normalized root: shifted by {dz} (floor_z={floor_z:.3f})[/cyan]")

    if args.debug_stats:
        def _variance(a):
            if len(a)==0: return 0.0
            return float(np.var(a, axis=0).mean())
        key_joints = ["Hips","Spine1","LeftUpLeg","LeftLeg","LeftToeBase","LeftArm","LeftForeArm","LeftHand"]
        print("[cyan]Motion variance (mean positional variance) per joint:[/cyan]")
        for j in key_joints:
            if j in frames[0]:
                pos_series = np.array([f[j][0] for f in frames])
                print(f"  {j:12s}: var={_variance(pos_series):.6e}")
            else:
                print(f"  {j:12s}: MISSING")
        # Quick warning if almost static
        static = [j for j in key_joints if j in frames[0] and _variance(np.array([f[j][0] for f in frames])) < 1e-6]
        if static:
            print(f"[yellow]Warning: joints with near-zero positional variance: {static}. Animation may be missing or not baked.[/yellow]")
        # Root motion magnitude
        if "Hips" in frames[0]:
            hips_pos = np.array([f["Hips"][0] for f in frames])
            disp = np.linalg.norm(hips_pos[-1] - hips_pos[0])
            print(f"[cyan]Root displacement (|hips_last - hips_first|): {disp:.6f} m[/cyan]")
        # Bounding box size sanity check
        all_pts = np.array([f["Hips"][0] for f in frames]) if "Hips" in frames[0] else None
        if all_pts is not None:
            bbox = all_pts.max(axis=0) - all_pts.min(axis=0)
            print(f"[cyan]Approx hips bbox (x,y,z): {bbox}")
            if np.max(bbox) < 1e-3:
                print("[red]Detected almost zero root motion. Likely export did not bake animation. Re-run with --use_blender --overwrite after ensuring actions exist, or verify FBX has keyframes.[/red]")
    print(f"Loaded {len(frames)} frames (assumed FPS=30), human_height={human_height:.2f}m")

    # Optional pelvis Z offset tweak (diagnostic) applied AFTER normalization/orientation
    if args.pelvis_z_offset and frames:
        try:
            if args.pelvis_z_offset.strip().lower() == 'auto':
                # Lazy load robot model to get current pelvis height (qpos zero) for comparison
                try:
                    from general_motion_retargeting.params import ROBOT_XML_DICT
                    model_path = str(ROBOT_XML_DICT[args.robot])
                    import mujoco as mj
                    mdl = mj.MjModel.from_xml_path(model_path)
                    data = mj.MjData(mdl)
                    mj.mj_forward(mdl, data)
                    pelvis_body = mdl.body('pelvis') if hasattr(mdl, 'body') else None
                    if pelvis_body is not None:
                        pelvis_id = mdl.body('pelvis').id
                        robot_pelvis_z = float(data.xpos[pelvis_id][2])
                    else:
                        robot_pelvis_z = 0.0
                except Exception:
                    robot_pelvis_z = 0.0
                human_hips_z = float(frames[0]['Hips'][0][2]) if 'Hips' in frames[0] else 0.0
                offset_val = robot_pelvis_z - human_hips_z
                print(f"[cyan]Auto pelvis_z_offset: robot pelvis z={robot_pelvis_z:.3f} human hips z={human_hips_z:.3f} -> applying offset {offset_val:.3f} m[/cyan]")
            else:
                offset_val = float(args.pelvis_z_offset)
                print(f"[cyan]Manual pelvis_z_offset: applying {offset_val:.3f} m[/cyan]")
            if abs(offset_val) > 5:
                print(f"[yellow]pelvis_z_offset magnitude {offset_val} seems large; ignoring.[/yellow]")
            else:
                for fr in frames:
                    for k,(p,q) in fr.items():
                        fr[k] = (p + np.array([0,0,offset_val]), q)
        except ValueError:
            print(f"[red]Invalid --pelvis_z_offset value: {args.pelvis_z_offset} (expected float or 'auto')[/red]")

    # Optional horizontal root alignment (after vertical offset)
    if args.align_root_xy and frames:
        try:
            if args.align_root_xy.strip().lower() == 'auto':
                try:
                    from general_motion_retargeting.params import ROBOT_XML_DICT
                    import mujoco as mj
                    model_path = str(ROBOT_XML_DICT[args.robot])
                    mdl = mj.MjModel.from_xml_path(model_path)
                    data = mj.MjData(mdl)
                    mj.mj_forward(mdl, data)
                    pelvis_id = mdl.body('pelvis').id
                    target_xy = data.xpos[pelvis_id][:2].copy()
                except Exception:
                    target_xy = np.array([0.0,0.0])
                if 'Hips' in frames[0]:
                    current_xy = frames[0]['Hips'][0][:2]
                    delta_xy = target_xy - current_xy
                else:
                    delta_xy = np.array([0.0,0.0])
                print(f"[cyan]Auto align_root_xy: applying delta {delta_xy} m to all joints (XY only)")
            else:
                # Expect "x,y"
                parts = [p for p in args.align_root_xy.replace(';',',').split(',') if p.strip()]
                if len(parts) != 2:
                    raise ValueError("Expected two comma-separated numbers for --align_root_xy")
                target_xy = np.array([float(parts[0]), float(parts[1])])
                current_xy = frames[0]['Hips'][0][:2] if 'Hips' in frames[0] else target_xy
                delta_xy = target_xy - current_xy
                print(f"[cyan]Manual align_root_xy: moving XY by {delta_xy} m")
            if np.linalg.norm(delta_xy) > 20:
                print(f"[yellow]align_root_xy delta {delta_xy} seems huge; ignoring.[/yellow]")
            else:
                for fr in frames:
                    for k,(p,q) in fr.items():
                        fr[k] = (p + np.array([delta_xy[0], delta_xy[1], 0.0]), q)
        except Exception as e:
            print(f"[red]Invalid --align_root_xy value ({args.align_root_xy}): {e}[/red]")

    # Prepare error CSV if requested
    csv_writer = None
    if args.errors_csv:
        try:
            first_create = not pathlib.Path(args.errors_csv).exists()
            csv_fp = open(args.errors_csv, 'a')
            if first_create:
                csv_fp.write('frame,error1,error2,'
                              'pelvis_pos_err,left_hand_pos_err,right_hand_pos_err\n')
            csv_writer = csv_fp
            print(f"[cyan]Logging per-frame errors to {args.errors_csv}[/cyan]")
        except Exception as e:
            print(f"[red]Failed to open errors CSV: {e}[/red]")
            csv_writer = None
    # Run retarget with optional logging
    retargeter = GMR(
        src_human="fbx",
        tgt_robot=args.robot,
        actual_human_height=human_height,
        solver=args.solver,
        verbose=False,
    )
    if args.no_scale_human:
        # Override all scale factors to 1.0 (diagnostic mode)
        for k in retargeter.human_scale_table.keys():
            retargeter.human_scale_table[k] = 1.0
        if args.log_errors:
            print("[yellow]Human scaling disabled (--no_scale_human); using unscaled limb lengths.[/yellow]")
    # Override pelvis weights if requested
    if args.pelvis_pos_w1 is not None and hasattr(retargeter, 'human_body_to_task1'):
        tbl = retargeter.ik_match_table1
        if 'pelvis' in tbl:
            body_name, _, rot_w, pos_off, rot_off = tbl['pelvis']
            tbl['pelvis'][1] = args.pelvis_pos_w1
            if args.log_errors:
                print(f"[cyan]Override pelvis pos weight stage1 -> {args.pelvis_pos_w1}")
    if args.pelvis_pos_w2 is not None and hasattr(retargeter, 'ik_match_table2'):
        tbl = retargeter.ik_match_table2
        if 'pelvis' in tbl:
            body_name, _, rot_w, pos_off, rot_off = tbl['pelvis']
            tbl['pelvis'][1] = args.pelvis_pos_w2
            if args.log_errors:
                print(f"[cyan]Override pelvis pos weight stage2 -> {args.pelvis_pos_w2}")
    fps = 30
    viewer = RobotMotionViewer(robot_type=args.robot,
                               motion_fps=fps,
                               record_video=args.record_video,
                               video_path=args.video_path,
                               traj_csv_path=args.traj_csv)
    # Pre-fetch mapping for key bodies
    key_map = {
        'pelvis': 'Hips',
        'left_wrist_yaw_link': 'LeftHand',
        'right_wrist_yaw_link': 'RightHand'
    }
    # Helper to fetch robot body world pos
    def _robot_body_pos(name):
        try:
            bid = retargeter.model.body(name).id
            return retargeter.configuration.data.xpos[bid].copy()
        except Exception:
            return np.full(3, np.nan)
    for f_idx, frame in enumerate(frames):
        qpos = retargeter.retarget(frame)
        # Compute errors if requested
        if args.log_errors or csv_writer:
            err1 = retargeter.error1() if hasattr(retargeter, 'error1') else np.nan
            err2 = retargeter.error2() if hasattr(retargeter, 'error2') else np.nan
            pelvis_err = left_err = right_err = np.nan
            human_targets = getattr(retargeter, 'scaled_human_data', frame)
            if 'Hips' in human_targets:
                robot_pelvis = _robot_body_pos('pelvis')
                pelvis_err = np.linalg.norm(robot_pelvis - human_targets['Hips'][0]) if robot_pelvis.size==3 else np.nan
            if 'LeftHand' in human_targets:
                robot_lh = _robot_body_pos('left_wrist_yaw_link')
                left_err = np.linalg.norm(robot_lh - human_targets['LeftHand'][0]) if robot_lh.size==3 else np.nan
            if 'RightHand' in human_targets:
                robot_rh = _robot_body_pos('right_wrist_yaw_link')
                right_err = np.linalg.norm(robot_rh - human_targets['RightHand'][0]) if robot_rh.size==3 else np.nan
            if args.log_errors and f_idx % 30 == 0:
                print(f"[blue]Frame {f_idx:04d} err1={err1:.4f} err2={err2:.4f} | pelvis={pelvis_err:.3f}m LH={left_err:.3f}m RH={right_err:.3f}m[/blue]")
            if args.task_error_breakdown and f_idx % 60 == 0:
                try:
                    import numpy as _np
                    def _task_errs(tasks):
                        return {t.frame_name: float(_np.linalg.norm(t.compute_error(retargeter.configuration))) for t in tasks}
                    errs1 = _task_errs(getattr(retargeter, 'tasks1', []))
                    errs2 = _task_errs(getattr(retargeter, 'tasks2', []))
                    print('[magenta]Task error breakdown (stage1 / stage2):')
                    for name in sorted(set(errs1)|set(errs2)):
                        print(f"  {name:20s} {errs1.get(name, float('nan')):8.4f} | {errs2.get(name, float('nan')):8.4f}")
                except Exception as _e:
                    print(f"[red]Failed task error breakdown: {_e}")
            if csv_writer:
                csv_writer.write(f"{f_idx},{err1},{err2},{pelvis_err},{left_err},{right_err}\n")
        viewer.step(
            root_pos=qpos[:3],
            root_rot=qpos[3:7],
            dof_pos=qpos[7:],
            human_motion_data=retargeter.scaled_human_data,
            rate_limit=args.rate_limit,
            follow_camera=True,
        )
    viewer.close()
    if csv_writer:
        csv_writer.close()

if __name__ == "__main__":  # pragma: no cover
    main()
