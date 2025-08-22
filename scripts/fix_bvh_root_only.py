#!/usr/bin/env python3
"""
Canonicalize a BVH so only the root joint has translation channels, with optional global upright rotation
and floor alignment. All non-root joints receive a static OFFSET (median/mean/first) and only rotation channels.

Pipeline:
 1. Parse BVH via existing lafan vendor reader (local quats + local translations per joint).
 2. Optional global upright: FK -> apply preset quaternion to every global joint (positions + orientations) -> IK back.
    Presets: none, x±90, y±90, z180, auto (choose preset maximizing first-frame spine upward Z component).
 3. Optional floor align: FK again, compute min global Z over foot / toe joints, shift root local Z so feet rest at z=0.
 4. Compute static offsets for all non-root joints (median/mean/first over their local translations). Keep original root offset.
 5. Convert all local quaternions to Euler angles in requested order (default zyx) and serialize frames: root pos + root rot + other rots.
 6. Rewrite hierarchy CHANNELS lines accordingly (root 6 channels, others 3) and update OFFSET lines.

Usage examples:
  python scripts/fix_bvh_root_only.py --input data/optitrack/test.bvh --output out/test_canonical.bvh
  python scripts/fix_bvh_root_only.py --input in.bvh --output out_upright.bvh --upright auto --floor_align
  python scripts/fix_bvh_root_only.py --input in.bvh --output out_mean_xyz.bvh --offset_stat mean --out_order xyz
"""
import argparse, re, sys, numpy as np
from pathlib import Path
from general_motion_retargeting.utils.lafan_vendor.extract import read_bvh
from general_motion_retargeting.utils.lafan_vendor import utils as vutils

ROT_TOKEN_MAP = {'x':'Xrotation','y':'Yrotation','z':'Zrotation'}

ORIENT_PRESETS = {
    'none': np.array([1,0,0,0], dtype=np.float64),
    'x90': np.array([np.sqrt(0.5), np.sqrt(0.5),0,0], dtype=np.float64),
    'x-90': np.array([np.sqrt(0.5),-np.sqrt(0.5),0,0], dtype=np.float64),
    'y90': np.array([np.sqrt(0.5),0,np.sqrt(0.5),0], dtype=np.float64),
    'y-90': np.array([np.sqrt(0.5),0,-np.sqrt(0.5),0], dtype=np.float64),
    'z180': np.array([0,0,0,1], dtype=np.float64),
}

def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument('--input', required=True)
    ap.add_argument('--output', required=True)
    ap.add_argument('--out_order', default='zyx', help='Euler rotation order (e.g. zyx, xyz, yxz).')
    ap.add_argument('--frame_time', type=float, default=1/30)
    ap.add_argument('--offset_stat', choices=['first','median','mean'], default='median')
    ap.add_argument('--precision', type=int, default=6)
    ap.add_argument('--upright', default='none', choices=list(ORIENT_PRESETS.keys())+['auto'])
    ap.add_argument('--floor_align', action='store_true', help='Shift root so min foot/toe global z=0 after upright')
    ap.add_argument('--verbose', action='store_true')
    return ap.parse_args()

def _quat_mul(q1, q2):
    w1,x1,y1,z1 = q1[...,0],q1[...,1],q1[...,2],q1[...,3]
    w2,x2,y2,z2 = q2[...,0],q2[...,1],q2[...,2],q2[...,3]
    return np.stack([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ], axis=-1)

def _quat_to_matrix(q):
    w,x,y,z = q
    n = w*w+x*x+y*y+z*z
    if n == 0: return np.eye(3)
    s = 2.0/n
    wx, wy, wz = s*w*x, s*w*y, s*w*z
    xx, xy, xz = s*x*x, s*x*y, s*x*z
    yy, yz, zz = s*y*y, s*y*z, s*z*z
    return np.array([
        [1-(yy+zz), xy - wz,    xz + wy],
        [xy + wz,   1-(xx+zz),  yz - wx],
        [xz - wy,   yz + wx,    1-(xx+yy)]
    ], dtype=np.float64)

def _quats_to_eulers(quats, order):
    order = order.lower()
    if set(order) != set('xyz') or len(order)!=3:
        raise ValueError(f'Bad order {order}')
    q = quats
    w,x,y,z = q[...,0], q[...,1], q[...,2], q[...,3]
    m00 = 1 - 2*(y*y+z*z)
    m01 = 2*(x*y - z*w)
    m02 = 2*(x*z + y*w)
    m10 = 2*(x*y + z*w)
    m11 = 1 - 2*(x*x+z*z)
    m12 = 2*(y*z - x*w)
    m20 = 2*(x*z - y*w)
    m21 = 2*(y*z + x*w)
    m22 = 1 - 2*(x*x+y*y)
    def extract(o):
        if o=='xyz':
            sy = np.clip(np.sqrt(m00*m00 + m10*m10),1e-8,None)
            X = np.arctan2(m21,m22); Y = np.arctan2(-m20,sy); Z = np.arctan2(m10,m00)
            return np.stack([X,Y,Z],axis=-1)
        if o=='zyx':
            sy = np.clip(np.sqrt(m22*m22 + m21*m21),1e-8,None)
            Z = np.arctan2(m10,m00); Y = np.arctan2(-m20,sy); X = np.arctan2(m21,m22)
            return np.stack([X,Y,Z],axis=-1)
        if o=='yxz':
            sy = np.clip(np.sqrt(m11*m11 + m12*m12),1e-8,None)
            Y = np.arctan2(m02,m22); X = np.arctan2(-m12,sy); Z = np.arctan2(m10,m11)
            return np.stack([X,Y,Z],axis=-1)
        if o=='xzy':
            sy = np.clip(np.sqrt(m00*m00 + m01*m01),1e-8,None)
            X = np.arctan2(-m12,m11); Z = np.arctan2(-m01,sy); Y = np.arctan2(m02,m00)
            return np.stack([X,Y,Z],axis=-1)
        if o=='zxy':
            sy = np.clip(np.sqrt(m22*m22 + m20*m20),1e-8,None)
            Z = np.arctan2(-m01,m11); X = np.arctan2(m21,m22); Y = np.arctan2(-m20,sy)
            return np.stack([X,Y,Z],axis=-1)
        if o=='yzx':
            sy = np.clip(np.sqrt(m11*m11 + m10*m10),1e-8,None)
            Y = np.arctan2(-m02,m22); Z = np.arctan2(m10,sy); X = np.arctan2(m12,m11)
            return np.stack([X,Y,Z],axis=-1)
        raise ValueError(o)
    return extract(order)

def load_header_lines(path: Path):
    lines = path.read_text(encoding='utf-8', errors='ignore').splitlines()
    head=[]
    for ln in lines:
        if ln.strip().startswith('MOTION'): break
        head.append(ln)
    return head

def main():
    args = parse_args()
    src = Path(args.input)
    if not src.is_file():
        print(f'[error] input not found: {src}'); sys.exit(1)
    anim = read_bvh(str(src))
    F,J = anim.pos.shape[0], anim.pos.shape[1]
    lquats = anim.quats.copy()
    lpos = anim.pos.copy()

    # Global upright rotation
    if args.upright != 'none':
        gq,gp = vutils.quat_fk(lquats, lpos, anim.parents)
        bones = anim.bones
        def find(names):
            for n in names:
                if n in bones: return bones.index(n)
            return None
        hips = find(['Hips','CC_Base_Pelvis','CC_Base_Hip','CC_Base_BoneRoot'])
        spine = find(['Spine1','Spine','CC_Base_Spine01','CC_Base_Waist','Spine2','CC_Base_Spine02'])
        preset = args.upright
        if preset=='auto' and hips is not None and spine is not None:
            base_vec = gp[0,spine] - gp[0,hips]
            best='none'; best_z=-1e9
            for cand in ['none','x90','x-90','y90','y-90','z180']:
                R = _quat_to_matrix(ORIENT_PRESETS[cand])
                vz = (R @ base_vec)[2]
                if vz>best_z: best_z=vz; best=cand
            preset=best
            print(f'[info] upright auto -> {preset}')
        elif preset=='auto':
            print('[warn] upright auto skipped (missing hips/spine)'); preset='none'
        if preset!='none':
            q_fix=ORIENT_PRESETS[preset]
            R_fix=_quat_to_matrix(q_fix)
            gq = _quat_mul(gq, np.broadcast_to(q_fix, gq.shape))
            gp = (R_fix @ gp.reshape(-1,3).T).T.reshape(gp.shape)
            lquats,lpos = vutils.quat_ik(gq, gp, anim.parents)
            if args.verbose: print(f'[info] applied global preset {preset}')
            if hips is not None and spine is not None:
                gq2,gp2 = vutils.quat_fk(lquats, lpos, anim.parents)
                v = gp2[0,spine]-gp2[0,hips]
                print(f'[metric] spine_z={v[2]/(np.linalg.norm(v)+1e-8):.4f}')

    # Floor alignment
    if args.floor_align:
        gq_tmp,gp_tmp = vutils.quat_fk(lquats, lpos, anim.parents)
        foot_ids=[]
        groups=[['LeftFoot','CC_Base_L_Foot','LeftToe','LeftToeBase','CC_Base_L_ToeBase'],
                ['RightFoot','CC_Base_R_Foot','RightToe','RightToeBase','CC_Base_R_ToeBase']]
        for grp in groups:
            for n in grp:
                if n in anim.bones:
                    foot_ids.append(anim.bones.index(n)); break
        if foot_ids:
            min_z = float(np.min(gp_tmp[:,foot_ids,2]))
            lpos[:,0,2] -= min_z
            print(f'[info] floor align shift root z by {-min_z:.4f}')
        else:
            print('[warn] floor align skipped: no foot joints')

    # Static offsets
    if args.offset_stat=='median': offsets = np.median(lpos, axis=0)
    elif args.offset_stat=='mean': offsets = np.mean(lpos, axis=0)
    else: offsets = lpos[0].copy()
    offsets[0] = anim.offsets[0]

    # Euler angles
    order = args.out_order.lower()
    eulers = np.degrees(_quats_to_eulers(lquats, order))  # (F,J,3)

    # Header rewrite
    header_lines = load_header_lines(src)
    joint_idx=-1
    rot_tokens = ' '.join(ROT_TOKEN_MAP[c] for c in order)
    rewritten=[]
    for line in header_lines:
        if re.match(r'ROOT\s+\w+', line) or re.match(r'\s*JOINT\s+\w+', line):
            joint_idx+=1; rewritten.append(line); continue
        if 'CHANNELS' in line:
            if joint_idx==0:
                newline = re.sub(r'CHANNELS\s+\d+.*', f'CHANNELS 6 Xposition Yposition Zposition {rot_tokens}', line)
            else:
                newline = re.sub(r'CHANNELS\s+\d+.*', f'CHANNELS 3 {rot_tokens}', line)
            rewritten.append(newline); continue
        if 'OFFSET' in line and joint_idx>=0:
            off = offsets[joint_idx]
            newline = re.sub(r'OFFSET\s+.*', f'OFFSET {off[0]:.6f} {off[1]:.6f} {off[2]:.6f}', line)
            rewritten.append(newline); continue
        rewritten.append(line)

    prec = args.precision
    out_lines = rewritten + ['MOTION', f'Frames: {F}', f'Frame Time: {args.frame_time:.7f}']
    for f in range(F):
        row=[]
        root_pos = lpos[f,0]
        row.extend([f'{root_pos[0]:.{prec}f}', f'{root_pos[1]:.{prec}f}', f'{root_pos[2]:.{prec}f}'])
        row.extend([f'{eulers[f,0,0]:.{prec}f}', f'{eulers[f,0,1]:.{prec}f}', f'{eulers[f,0,2]:.{prec}f}'])
        for j in range(1,J):
            row.extend([f'{eulers[f,j,0]:.{prec}f}', f'{eulers[f,j,1]:.{prec}f}', f'{eulers[f,j,2]:.{prec}f}'])
        out_lines.append(' '.join(row))

    dst = Path(args.output)
    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_text('\n'.join(out_lines))
    print(f'[info] wrote canonical BVH {dst} (frames={F}, joints={J}, order={order})')

if __name__ == '__main__':
    main()
