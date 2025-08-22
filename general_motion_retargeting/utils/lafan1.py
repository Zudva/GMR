import numpy as np
from scipy.spatial.transform import Rotation as R

import general_motion_retargeting.utils.lafan_vendor.utils as utils
from general_motion_retargeting.utils.lafan_vendor.extract import read_bvh


def load_lafan1_file(bvh_file):
    """
    Must return a dictionary with the following structure:
    {
        "Hips": (position, orientation),
        "Spine": (position, orientation),
        ...
    }
    """
    data = read_bvh(bvh_file)
    global_data = utils.quat_fk(data.quats, data.pos, data.parents)

    rotation_matrix = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
    rotation_quat = R.from_matrix(rotation_matrix).as_quat(scalar_first=True)

    frames = []
    # Precompute which optional foot joints exist
    has_left = {k: k in data.bones for k in ["LeftFoot", "LeftToe"]}
    has_right = {k: k in data.bones for k in ["RightFoot", "RightToe"]}

    for frame in range(data.pos.shape[0]):
        result = {}
        for i, bone in enumerate(data.bones):
            orientation = utils.quat_mul(rotation_quat, global_data[0][frame, i])
            position = global_data[1][frame, i] @ rotation_matrix.T / 100  # cm to m
            result[bone] = (position, orientation)

        # Add modified foot pose only if source joints exist (LAFAN1 pattern)
        if has_left.get("LeftFoot") and has_left.get("LeftToe") and "LeftFoot" in result and "LeftToe" in result:
            result["LeftFootMod"] = (result["LeftFoot"][0], result["LeftToe"][1])
        if has_right.get("RightFoot") and has_right.get("RightToe") and "RightFoot" in result and "RightToe" in result:
            result["RightFootMod"] = (result["RightFoot"][0], result["RightToe"][1])
        frames.append(result)

    # Height estimation fallback:
    # 1) Prefer Head minus min of (Left/RightFootMod) if available.
    # 2) Else use Head minus global minimum z across all joints in last frame.
    if frames:
        last = frames[-1]
        if "Head" in last:
            head_z = last["Head"][0][2]
            foot_candidates = []
            for k in ["LeftFootMod", "RightFootMod", "LeftFoot", "RightFoot"]:
                if k in last:
                    foot_candidates.append(last[k][0][2])
            if foot_candidates:
                min_foot = min(foot_candidates)
                human_height = head_z - min_foot
            else:
                # global min z
                min_all = min(v[0][2] for v in last.values())
                human_height = head_z - min_all
        else:
            # Fallback: span of z across all joints
            z_vals = [v[0][2] for v in last.values()]
            human_height = max(z_vals) - min(z_vals) if z_vals else 1.75
    else:
        human_height = 1.75

    # If Head missing or improbable height, clamp to reasonable default
    if not np.isfinite(human_height) or human_height < 0.9 or human_height > 2.3:
        human_height = 1.75

    return frames, human_height


