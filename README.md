# GMR: General Motion Retargeting
# GMR: General Motion Retargeting


  <a href="https://arxiv.org/abs/2505.02833">
    <img src="https://img.shields.io/badge/paper-arXiv%3A2505.02833-b31b1b.svg" alt="arXiv Paper"/>
  </a>
  <a href="https://opensource.org/licenses/MIT">
    <img src="https://img.shields.io/badge/License-MIT-yellow.svg" alt="License: MIT"/>
  </a>
  <a href="https://github.com/YanjieZe/GMR/releases">
    <img src="https://img.shields.io/badge/version-0.2.0-blue.svg" alt="Version"/>
  </a>
  <a href="https://x.com/ZeYanjie/status/1952446745696469334">
    <img src="https://img.shields.io/badge/twitter-ZeYanjie-blue.svg" alt="Twitter"/>
  </a>
  <a href="https://yanjieze.github.io/humanoid-foundation/#GMR">
    <img src="https://img.shields.io/badge/blog-GMR-blue.svg" alt="Blog"/>
  </a>


![Banner for GMR](./assets/GMR.png)

Key features of GMR:
- Real-time high-quality retargeting, unlock the potential of real-time whole-body teleoperation, i.e., [TWIST](https://github.com/YanjieZe/TWIST).
- Carefully tuned for good performance of RL tracking policies.
- Support multiple humanoid robots and multiple human motion data formats (See our table below).

**NOTE: If you want this repo to support a new robot or a new human motion data format, send the robot files (`.xml` (must), `.urdf` (must), and meshes (must)) / human motion data to <a href="mailto:lastyanjieze@gmail.com">Yanjie Ze</a> or create an issue, we will support it as soon as possible.** Please make sure the robot files you sent can be open-sourced in this repo.

This repo is licensed under the [MIT License](LICENSE).

See [CHANGELOG](doc/CHANGELOG.md) for recent updates.

# News & Updates

- 2025-08-10: GMR now supports [Booster K1](https://www.boosterobotics.com/), the 9th robot in the repo.
- 2025-08-09: GMR now supports *Unitree G1 with Dex31 hands*.
- 2025-08-07: GMR now supports [Galexea R1 Pro](https://galaxea-dynamics.com/) (this is a wheeled humanoid robot!) and [KUAVO](https://www.kuavo.ai/), the 7th and 8th humanoid robots in the repo.
- 2025-08-06: GMR now supports [HighTorque Hi](https://www.hightorquerobotics.com/hi/), the 6th humanoid robot in the repo.
- 2025-08-04: Initial release of GMR. Check our [twitter post](https://x.com/ZeYanjie/status/1952446745696469334).


# Demo

Demo 1: Retargeting LAFAN1 dancing motion to 5 different robots (Unitree G1, Booster T1, Stanford ToddlerBot, Fourier N1, and ENGINEAI PM01):



https://github.com/user-attachments/assets/23566fa5-6335-46b9-957b-4b26aed11b9e

Demo 2: Galexea R1 Pro, a wheeled humanoid robot, doing human motion


https://github.com/user-attachments/assets/903ed0b0-0ac5-4226-8f82-5a88631e9b7c


https://github.com/user-attachments/assets/deea0e64-f1c6-41bc-8661-351682006d5d


Demo 3: Screen recording of my one command line usage. Switch robots with just changign an argument.


https://github.com/user-attachments/assets/03f10902-c541-40b1-8104-715a5759fd5e

Demo 4: HighTorque robot doing a twist dance



https://github.com/user-attachments/assets/1d3e663b-f29e-41b1-8e15-5c0deb6a4a5c

Demo 5: Kuavo robot picking up a box


https://github.com/user-attachments/assets/02fc8f41-c363-484b-a329-4f4e83ed5b80



# Supported Robots and Data Formats

| Robot/Data Format | Robot DoF | SMPLX ([AMASS](https://amass.is.tue.mpg.de/), [OMOMO](https://github.com/lijiaman/omomo_release)) | BVH ( [LAFAN1](https://github.com/ubisoft/ubisoft-laforge-animation-dataset)) | FBX ( [OptiTrack](https://www.optitrack.com/)) | More formats coming soon | 
| --- | --- | --- | --- | --- | --- |
| Unitree G1 `unitree_g1` | Leg (2\*6) + Waist (3) + Arm (2\*7) = 29 | ✅ | ✅ | ✅ |
| Unitree G1 with Hands `unitree_g1_with_hands` | Leg (2\*6) + Waist (3) + Arm (2\*7) + Hand (2\*7) = 43 | ✅ | ✅ | ✅ |
| Unitree H1 | TBD | TBD | TBD | TBD |
| AgiBot X1 | TBD | TBD | TBD | TBD |
| Booster T1 `booster_t1` | TBD | ✅ |  ✅  | TBD | 
| Booster K1 `booster_k1` | Neck (2) + Arm (2\*4) + Leg (2\*6) = 22 | ✅ | TBD | TBD |
| Stanford ToddlerBot `stanford_toddy` | TBD | ✅ | ✅ | TBD |
| Berkeley Humanoid Lite `berkeley_humanoid_lite` | TBD | TBD | TBD | TBD |
| Fourier N1 `fourier_n1` | TBD | ✅ | ✅ | TBD |
| ENGINEAI PM01 `engineai_pm01` | TBD | ✅ | ✅ | TBD |
| HighTorque Hi `hightorque_hi` | Head (2) + Arm (2\*5) + Waist (1) + Leg (2\*6) = 25 | ✅ | TBD | TBD |
| Galaxea R1 Pro `galaxea_r1pro` (this is a wheeled robot!) |  Base (6) + Torso (4) + Arm (2*7) = 24 | ✅ | TBD | TBD |
| Kuavo `kuavo_s45` |  Head (2) + Arm (2\*7) + Leg (2\*6) = 28 | ✅ | TBD | TBD |
| More robots coming soon | |







# Installation

The code is tested on Ubuntu 22.04/20.04.

```bash
# create conda env
conda create -n gmr python=3.10 -y
conda activate gmr

# install GMR
pip install -e .

# NOTE: after install SMPLX, change `ext` in `smplx/body_models.py` from `npz` to `pkl` if you are using SMPL-X pkl files.

# to resolve some possible rendering issues
conda install -c conda-forge libstdcxx-ng -y
```


# Data Preparation

[[SMPLX](https://github.com/vchoutas/smplx) body model] download SMPL-X body models to `assets/body_models` from [SMPL-X](https://smpl-x.is.tue.mpg.de/) and then structure as follows:
```bash
- assets/body_models/smplx/
-- SMPLX_NEUTRAL.pkl
-- SMPLX_FEMALE.pkl
-- SMPLX_MALE.pkl
```

[[AMASS](https://amass.is.tue.mpg.de/) motion data] download raw SMPL-X data to any folder you want from [AMASS](https://amass.is.tue.mpg.de/). NOTE: Do not download SMPL+H data.

[[OMOMO](https://github.com/lijiaman/omomo_release) motion data] download raw OMOMO data to any folder you want from [this google drive file](https://drive.google.com/file/d/1tZVqLB7II0whI-Qjz-z-AU3ponSEyAmm/view?usp=sharing). And process the data into the SMPL-X format using `scripts/convert_omomo_to_smplx.py`.

[[LAFAN1](https://github.com/ubisoft/ubisoft-laforge-animation-dataset) motion data] download raw LAFAN1 bvh files from [the official repo](https://github.com/ubisoft/ubisoft-laforge-animation-dataset), i.e., [lafan1.zip](https://github.com/ubisoft/ubisoft-laforge-animation-dataset/blob/master/lafan1/lafan1.zip).


# Human/Robot Motion Data Formulation

To better use this library, you can first have an understanding of the human motion data we use and the robot motion data we obtain.

Each frame of **human motion data** is formulated as a dict of (human_body_name, 3d global translation + global rotation).

Each frame of **robot motion data** can be understood as a tuple of (robot_base_translation, robot_base_rotation, robot_joint_positions).



# Usage

## Retargeting from SMPL-X (AMASS, OMOMO) to Robot

**NOTE: after install SMPL-X, change `ext` in `smplx/body_models.py` from `npz` to `pkl` if you are using SMPL-X pkl files.**


Retarget a single motion:
```bash
# single motion
python scripts/smplx_to_robot.py --smplx_file <path_to_smplx_data> --robot <path_to_robot_data> --save_path <path_to_save_robot_data.pkl> --rate_limit
```
By default you should see the visualization of the retargeted robot motion in a mujoco window.
If you want to record video, add `--record_video` and `--video_path <your_video_path,mp4>`.
- `--rate_limit` is used to limit the rate of the retargeted robot motion to keep the same as the human motion. If you want it as fast as possible, remove `--rate_limit`.


Retarget a folder of motions:
```bash
python scripts/smplx_to_robot_dataset.py --src_folder <path_to_dir_of_smplx_data> --tgt_folder <path_to_dir_to_save_robot_data> --robot <robot_name>
```
By default there is no visualization for batch retargeting.


## Retargeting from BVH (LAFAN1) to Robot

Retarget a single motion:
```bash
# single motion (Linux)
python scripts/bvh_to_robot.py --bvh_file <path_to_bvh_data> --robot <robot_name> --save_path <out.pkl> --rate_limit

# macOS (нужно окно MuJoCo) → запускать через mjpython
mjpython scripts/bvh_to_robot.py --bvh_file <path_to_bvh_data> --robot <robot_name> --save_path <out.pkl> --rate_limit
```
By default you should see the visualization of the retargeted robot motion in a mujoco window. 
- `--rate_limit` is used to limit the rate of the retargeted robot motion to keep the same as the human motion. If you want it as fast as possible, remove `--rate_limit`.

Additional useful BVH flags recently added:

| Flag | Purpose | Notes |
|------|---------|-------|
| `--orient_fix {none,x90,x-90,y90,y-90,z180,auto}` | Глобальная ориентация BVH; `auto` подбирает пресет чтобы «спина смотрела вверх». | Применяется до IK; влияет на все кости. |
| `--offset_to_ground` | Опускает человека так, чтобы минимальная точка ступней оказалась около z=0 (перед retarget). | Удобно убрать «парение» или «провал». |
| `--show_human_names` | Показать подписи костей человеческого скелета в viewer. | Может снижать FPS. |
| `--human_offset dx,dy,dz` | Сдвиг (метры) визуализации человека относительно робота. | Только визуализация. |
| `--show_root_diff` | Показ ∆ позиции корня робота и человеческих Hips. | В оверлее слева. |
| `--show_quat` | Добавить строку с корневым кватернионом (wxyz) в overlay и видео. | Для отладки ориентации. |
| `--print_bvh_bones` | Вывести список костей BVH и выйти. | Без ретаргета. |
| `--dump_first_frame_json path.json` | Сохранить первый кадр после нормализации имён / синтеза суставов. | Для проверки названий / ориентации. |
| `--no_viewer` | Запуск headless (без окна). | Полезно на macOS без mjpython. |

Внутренние шаги пайплайна BVH:
1. Чтение BVH → кадры словарей {joint: (pos, quat_wxyz)}
2. Нормализация имён `_fill_synonyms`
3. Синтез недостающих костей (например, FootMod / Spine2)
4. Ориентационный фикс `--orient_fix`
5. (Опционально) привязка к полу `--offset_to_ground`
6. Обновление целей IK → решение IK → `qpos`
7. Визуализация + (опц.) запись видео / сохранение pickle


Retarget a folder of motions:
```bash
python scripts/bvh_to_robot_dataset.py --src_folder <path_to_dir_of_bvh_data> --tgt_folder <path_to_dir_to_save_robot_data> --robot <robot_name>
```
By default there is no visualization for batch retargeting.


## Retargeting from FBX (OptiTrack) to Robot

We provide the script to use OptiTrack MoCap data for real-time streaming and retargeting.

Usually you will have two computers, one is the server that installed with Motive (Desktop APP for OptiTrack) and the other is the client that installed with GMR.

Find the server ip (the computer that installed with Motive) and client ip (your computer). Set the streaming as follows:

![OptiTrack Streaming](./assets/optitrack.png)

And then run:
```bash
python scripts/optitrack_to_robot.py --server_ip <server_ip> --client_ip <client_ip> --use_multicast False --robot unitree_g1
```
You should see the visualization of the retargeted robot motion in a mujoco window.

### Offline Processing of an FBX File (FBX -> BVH -> Robot)

We provide an extended pipeline with automatic conversion, orientation correction, normalization, diagnostics and logging via `scripts/fbx_to_robot.py`.

macOS NOTE: launch with `mjpython` (MuJoCo patched python) to open the viewer; running with plain `python` on macOS will raise an error (`launch_passive requires ... mjpython`).

Basic auto-orientation + normalization example:
```bash
mjpython scripts/fbx_to_robot.py \
  --fbx_file /path/to/motion.fbx \
  --robot unitree_g1 \
  --always_overwrite \
  --orient_fix auto --auto_forward_axis x \
  --normalize_root \
  --log_errors \
  --errors_csv errors.csv \
  --dump_bvh_header 60
```

Ubisoft axis style (skip extra orientation composition) variant:
```bash
mjpython scripts/fbx_to_robot.py \
  --fbx_file /path/to/motion.fbx \
  --robot unitree_g1 \
  --always_overwrite \
  --ubisoft_axes \
  --normalize_root \
  --log_errors --errors_csv errors.csv
```

Key FBX pipeline flags:
- `--always_overwrite` force reconversion (ignore existing BVH).
- `--orient_fix {x90,x-90,y90,y-90,z180,auto,none}` preset or auto-detected global rotation.
- `--auto_forward_axis {x|y}` desired forward when using `--orient_fix auto`.
- `--ubisoft_axes` apply only canonical Y-up→Z-up axis fix (skip additional quaternion composition).
- `--normalize_root` translate initial Hips to origin and move floor (estimated from feet) to z=0.
- `--dump_bvh_header N` print first N lines of the BVH for channel/joint inspection.
- `--force_generic_loader` bypass strict lafan loader; use generic parser.
- Error diagnostics: `--log_errors` (console every ~30 frames) + `--errors_csv path` (per-frame CSV: frame,error1,error2,pelvis_pos_err,left_hand_pos_err,right_hand_pos_err).

Viewer overlay (enabled by default) displays root XYZ, velocity, and yaw/pitch/roll; trajectory CSV logging is available programmatically (constructor parameter) and will be exposed via CLI in a future update.

If a BVH already exists and you only want to inspect its header and play it:
```bash
mjpython scripts/fbx_to_robot.py --bvh_file /path/to/file.bvh --robot unitree_g1 --dump_bvh_header 80 --log_errors
```

Troubleshooting tips:
- Static (T-pose) export: re-run with `--always_overwrite --use_blender` ensuring the FBX has keyframes.
- Lying / rotated avatar: try `--orient_fix auto` or a preset (e.g. `--orient_fix x90`).
- Parser reshape/channel errors: add `--force_generic_loader` (automatic loose fallback also triggers when strict parsing fails).
- Almost zero motion variance warning: verify animation baked correctly in Blender; ensure correct armature selected.

### Minimal BVH / FBX Quick Start (New Quick Orientation Scan)

If you already have a BVH (or converted FBX externally) and just want a quick visualization without complex auto-orientation logic:

```bash
mjpython scripts/fbx_to_robot.py \
  --bvh_file data/optitrack/test.bvh \
  --robot unitree_g1 \
  --orient_fix none \
  --quick_orient_scan
```

`--quick_orient_scan` tries a tiny set of global rotations (identity, x±90, y±90, z180) around the hips and picks the one making the spine point most upward (preventing the character from lying face‑down). Use it only when you pass `--orient_fix none`.

#### Flag Behavior Clarification

| Flag | What it Does | When to Use | Side Effects |
|------|---------------|-------------|--------------|
| `--normalize_root` | Shifts initial hips to origin and raises / lowers so feet sit on z=0 | For datasets with arbitrary world offsets / ground penetration | Can make model appear “sunk” if ground estimate off; combine with `--use_root_motion` to restore trajectory |
| `--use_root_motion` | Reapplies original pre-normalization hips XY(Z) trajectory to base after IK | When you want preserved global translation/heading | If normalization floor estimate was low, legs may clip until offsets tuned |
| `--pelvis_pos_w1/2` | Increases pelvis position weight in IK stage1/stage2 | If pelvis drifts or root motion isn’t followed | Too large: may force unnatural leg pose (stiff/feet sliding) |
| `--suggest_offsets <thr>` | Compares first-frame human vs robot target per body, suggests position offsets where |delta| > threshold (m) | Rapid coarse alignment when skeleton origins differ | Uses only first frame (heuristic); can overfit if starting pose atypical |
| `--apply_suggested_offsets` | Immediately applies those suggested offsets at runtime (does NOT write JSON) | Fast iteration while tuning | May distort limbs (e.g. crossed legs) if suggestions include lateral leg swaps |

If `--suggest_offsets` produced crossed legs, lower the threshold (e.g. 0.25→0.15→0.10) or skip applying and instead manually edit only safe torso/arm offsets in `general_motion_retargeting/ik_configs/fbx_to_g1.json`.

Recommended progressive workflow:
1. Start: quick orientation scan (command above).
2. If upright: add `--normalize_root --use_root_motion`. Check for sinking; if sunk only to waist, ground estimate may be low—consider later adding a custom pelvis Z offset flag (coming soon) or adjusting floor detection.
3. Increase pelvis weights only if root jitters or floats (`--pelvis_pos_w1 20 --pelvis_pos_w2 40`). If no visible improvement, keep defaults.
4. Run `--suggest_offsets 0.15` WITHOUT `--apply_suggested_offsets` first to inspect console deltas; selectively copy good torso/shoulder values into config.
5. Re-run without suggestions and verify symmetry (avoid leg crossing). Then optionally fine-tune smaller threshold.

Persisting offsets: once satisfied, edit the JSON file (e.g. `fbx_to_g1.json`) and place chosen vectors into `stage1.pos_offset` / `stage2.pos_offset` entries so you no longer need runtime suggestion flags.

### Quick Reference: Two Canonical Commands (BVH vs FBX)

Below are two "рабочие" (tested working) baseline commands you can copy-paste depending on whether you start from BVH or directly from an FBX file.

BVH (already have BVH, just orient & view):
```bash
mjpython scripts/fbx_to_robot.py \
  --bvh_file data/optitrack/test.bvh \
  --robot unitree_g1 \
  --orient_fix none \
  --quick_orient_scan
```
Add these if you also want root normalization + original translation restored:
```bash
  --normalize_root --use_root_motion
```

FBX (convert FBX -> BVH + full pipeline with auto orientation & root motion):
```bash
mjpython scripts/fbx_to_robot.py \
  --fbx_file /path/to/your_motion.fbx \
  --robot unitree_g1 \
  --always_overwrite \
  --orient_fix auto --auto_forward_axis x \
  --normalize_root --use_root_motion \
  --log_errors --errors_csv errors.csv
```
Optional tuning flags (use only if needed):
```bash
  --pelvis_pos_w1 30 --pelvis_pos_w2 50            # strengthen pelvis tracking
  --suggest_offsets 0.15 --apply_suggested_offsets  # heuristic first-frame offsets (inspect before applying)
  --camera_dist 5                                   # move camera back
```
Minimal difference summary:
- BVH path skips conversion; we rely on `--quick_orient_scan` to choose an upright preset because we disabled other orientation logic (`--orient_fix none`).
- FBX path performs Blender conversion (if needed), full auto orientation (`--orient_fix auto`), normalization, then restores raw root trajectory (`--use_root_motion`).
- Normalization may change vertical placement; if the avatar sinks, adjust offsets later instead of removing orientation correction.

### Extended Diagnostics & Mapping Debug (Added After Original Release)

New helper flags for deeper inspection of FBX/BVH alignment and IK quality:
```bash
--dump_first_frame_json first_frame_debug.json   # JSON: all joints (pos meters, quat wxyz) after orientation, before IK loop
--dump_targets_csv targets.csv                  # CSV: per-frame key target positions (hips, hands, toes)
--debug_initial_targets                         # Prints robot-vs-human deltas for pelvis/hands/toes before loop
--debug_pelvis_anchor                           # Every 30 frames: distances pelvis→(Hips, LeftUpLeg, RightUpLeg)
--suggest_offsets <thr> [--apply_suggested_offsets]
--auto_pelvis_offset xyz                        # Auto inject pelvis pos_offset along chosen axes
```

Example (stable orientation known = `x-90`) with full logging & dumps:
```bash
mjpython scripts/fbx_to_robot.py \
  --fbx_file /path/to/your_motion.fbx \
  --robot unitree_g1 \
  --always_overwrite \
  --orient_fix x-90 \
  --log_errors --errors_csv errors.csv \
  --dump_first_frame_json first_frame_debug.json \
  --dump_targets_csv targets.csv
```

### Differences vs. Original Video Demo Pipeline

The initial repository video showed a minimal retarget path. The current extended pipeline adds layers primarily for robustness & diagnosis. Key differences:

| Aspect | Original Demo (Video) | Extended Pipeline (Now) | Benefit |
|--------|-----------------------|-------------------------|---------|
| Orientation handling | Manual preset / implicit | `--orient_fix auto` + `--quick_orient_scan` | Automatic upright detection, fewer “lying” avatars |
| Axis fix | Fixed Y-up→Z-up | Selectable (`--ubisoft_axes`, `--no_axis_fix`) | Easier experimentation with unusual exports |
| Root normalization | None / implicit origin | `--normalize_root` + `--use_root_motion` | Clean floor contact + preserved global trajectory |
| Pelvis drift control | Default weights | `--pelvis_pos_w1/2`, `--auto_pelvis_offset` | Stabilized base alignment, remove constant bias |
| Morphology offsets | Static JSON only | `--suggest_offsets` + optional apply | Rapid coarse tuning without editing JSON each run |
| Orientation quick preset test | Manual trial & error | `--quick_orient_scan` | Single run finds best of {id, x±90, y±90, z180} |
| Per-frame diagnostics | Basic console output | `--log_errors`, `--errors_csv`, `--task_error_breakdown` | Quantitative tracking / regression checks |
| Joint mapping sanity | None | `--dump_first_frame_json`, `--debug_initial_targets` | Fast detection of mis-mapped limbs (e.g., left arm) |
| Trajectory export | Not exposed | `--traj_csv`, `--dump_targets_csv` | Offline plotting / analytics |
| Pelvis anchor insight | None | `--debug_pelvis_anchor` | Understand which human joint pelvis follows |
| Camera control | Default distance | `--camera_dist` | Consistent framing for comparisons |

Practical effect: you can start with a legacy quick view, then progressively enable only the diagnostics you need (JSON or CSV dumps, offset suggestions) without permanently altering the IK config. This reduces iteration time when resolving issues like a single limb misalignment (e.g., left arm) or constant pelvis bias.

### Legacy Comparison

For a detailed table comparing the early minimal commit (`1b9f50c`) to the current feature-rich pipeline (orientation scan, root normalization/restoration, diagnostics, FBX path, offset heuristics, dumps, viewer upgrades), see: [doc/LEGACY_DIFF.md](doc/LEGACY_DIFF.md).


## Visualize saved robot motion
We provide a lightweight viewer `scripts/vis_robot_motion.py` for already retargeted motions (`root_pos`, `root_rot`, `dof_pos`). It now supports both pickle (`.pkl`) and Torch (`.pt`) motion files.

Quaternion convention reminder:
- Internal pipeline (IK / viewer input) uses `wxyz`.
- Saved motion files (`.pkl` / `.pt`) store `root_rot` as `xyzw` (legacy decision to align with some datasets).
- The viewer automatically reorders to `wxyz` and renormalizes; you do NOT need to preprocess.

Quick playback examples:
```bash
# Linux (any .pkl or .pt)
python scripts/vis_robot_motion.py --motion out/aiming1_g1.pkl --robot unitree_g1
python scripts/vis_robot_motion.py --motion out/aiming1_g1.pt  --robot unitree_g1

# macOS (GUI requires mjpython)
mjpython scripts/vis_robot_motion.py --motion out/aiming1_g1.pkl --robot unitree_g1
mjpython scripts/vis_robot_motion.py --motion out/aiming1_g1.pt  --robot unitree_g1
```

Fast mode (disable real-time pacing):
```bash
mjpython scripts/vis_robot_motion.py --motion out/aiming1_g1.pkl --robot unitree_g1 --no_rate_limit
```

Headless validation (shape + quaternion norm check, no GUI):
```bash
python scripts/vis_robot_motion.py --motion out/aiming1_g1.pkl --robot unitree_g1 --headless --summary
```

Record video while playing (.pkl or .pt):
```bash
mjpython scripts/vis_robot_motion.py --motion out/aiming1_g1.pkl --robot unitree_g1 --video videos/replay.mp4
```

Viewer flags summary:
| Flag | Purpose |
|------|---------|
| `--motion PATH` | Input motion file (`.pkl` or `.pt`). |
| `--robot NAME` | Robot key (e.g. `unitree_g1`). |
| `--video path.mp4` | Record playback to MP4. |
| `--no_rate_limit` | Run frames as fast as possible (no sleep). |
| `--headless` | Skip GUI; validate data & print result. |
| `--summary` | Print tensor shapes / dtypes before playback. |

Ground alignment AFTER saving (если робот «парит»). Быстрая правка pickle:
```python
import pickle, numpy as np
m = pickle.load(open('out/aiming1_g1.pkl','rb'))
dz = m['root_pos'][:,2].min()   # или m['root_pos'][0,2]
m['root_pos'][:,2] -= dz
pickle.dump(m, open('out/aiming1_g1_floor.pkl','wb'))
```
или (предпочтительно) используйте `--offset_to_ground` при ретаргете BVH, чтобы сохранить корректную посадку сразу в сохранённом файле.

## Motion file conversion: PKL ↔ PT (Torch)

После ретаргета скрипты сохраняют движение в pickle (`.pkl`) со структурой:

```
{
  'fps': int,
  'root_pos': (T,3) float64,
  'root_rot': (T,4) float64  # ПОРЯДОК: xyzw (заметка: внутренний IK держит wxyz)
  'dof_pos': (T,Ndof) float64,
  'local_body_pos': None или массив,
  'link_body_list': None или список имён
}
```

Для прямой загрузки в PyTorch удобнее `.pt` (нет доп. копий numpy→tensor). Добавлен скрипт `scripts/convert_motion_pkl_to_pt.py`.

Базовые команды:
```bash
# PKL -> PT
python scripts/convert_motion_pkl_to_pt.py --input out/aiming1_g1.pkl --output out/aiming1_g1.pt

# PT -> PKL
python scripts/convert_motion_pkl_to_pt.py --input out/aiming1_g1.pt --output out/aiming1_g1_back.pkl

# Показать сводку (без сохранения итогового файла)
python scripts/convert_motion_pkl_to_pt.py --input out/aiming1_g1.pkl --output dummy.pt --summary

# Перезаписать существующий файл
python scripts/convert_motion_pkl_to_pt.py --input out/aiming1_g1.pkl --output out/aiming1_g1.pt --force
```

Особенности:
- Кватернион в сохранённом motion pickle: `xyzw`. Скрипт конвертации НЕ меняет порядок — он сохраняет как есть.
- Viewer (`vis_robot_motion.py`) сам переставляет в `wxyz` при воспроизведении.
- Скрипт не меняет dtype (по умолчанию float64) — для тренировки обычно выгодно float32.

### Опциональная оптимизация (float32 для экономии памяти)

```python
import pickle, torch, numpy as np
data = pickle.load(open('out/aiming1_g1.pkl','rb'))
for k in ['root_pos','root_rot','dof_pos']:
    data[k] = data[k].astype(np.float32)
torch.save({k: torch.from_numpy(v) if isinstance(v, np.ndarray) else v for k,v in data.items()},
           'out/aiming1_g1_f32.pt')
```

### Проверка нормализации кватернионов
```python
import pickle, numpy as np
q = pickle.load(open('out/aiming1_g1.pkl','rb'))['root_rot']  # xyzw
err = np.abs(np.linalg.norm(q, axis=1) - 1).max()
print('max |norm-1| =', err)
```
Если `err < 1e-6` — всё хорошо.

### Быстрая пакетная конверсия каталога PKL → PT
```bash
for f in out/*.pkl; do \
  python scripts/convert_motion_pkl_to_pt.py --input "$f" --output "${f%.pkl}.pt" --force; \
done
```

### Зачем переходить на .pt
| Плюс | Описание |
|------|----------|
| Быстрая загрузка | `torch.load` сразу создаёт тензоры на CPU. |
| Меньше копий | Нет промежуточного numpy→tensor в тренировочном коде. |
| Легко оптимизировать | Можно заранее перевести в float32 / half / quant. |
| Интеграция с DataLoader | Можно memory pinning / async transfer.

Если понадобится: можно добавить опцию `--float32` в сам скрипт — откройте issue / PR.


# Speed Benchmark

| CPU | Retargeting Speed |
| --- | --- |
| AMD Ryzen Threadripper 7960X 24-Cores | 60~70 FPS |
| 13th Gen Intel Core i9-13900K 24-Cores | 35~45 FPS |
| TBD | TBD |


# Citation

If you find our code useful, please consider citing our papers:

```bibtex
@article{ze2025twist,
title={TWIST: Teleoperated Whole-Body Imitation System},
author= {Yanjie Ze and Zixuan Chen and João Pedro Araújo and Zi-ang Cao and Xue Bin Peng and Jiajun Wu and C. Karen Liu},
year= {2025},
journal= {arXiv preprint arXiv:2505.02833}
}
```
and this github repo:
```bibtex
@software{ze2025gmr,
title={GMR: General Motion Retargeting},
author= {Yanjie Ze and João Pedro Araújo and Jiajun Wu and C. Karen Liu},
year= {2025},
url= {https://github.com/YanjieZe/GMR},
note= {GitHub repository}
}
```
# Known Issues

Designing a single config for all different humans is not trivial. We observe some motions might have bad retargeting results. If you observe some bad results, please let us know! We now have a collection of such motions in [TEST_MOTIONS.md](TEST_MOTIONS.md).



# Acknowledgement
Our IK solver is built upon [mink](https://github.com/kevinzakka/mink) and [mujoco](https://github.com/google-deepmind/mujoco). Our visualization is built upon [mujoco](https://github.com/google-deepmind/mujoco). The human motion data we try includes [AMASS](https://amass.is.tue.mpg.de/), [OMOMO](https://github.com/lijiaman/omomo_release), and [LAFAN1](https://github.com/ubisoft/ubisoft-laforge-animation-dataset).

The original robot models can be found at the following locations:

* Booster T1: [Official website](https://booster.feishu.cn/wiki/UvowwBes1iNvvUkoeeVc3p5wnUg) ([English](https://booster.feishu.cn/wiki/DtFgwVXYxiBT8BksUPjcOwG4n4f)).
* [EngineAI PM01](https://github.com/engineai-robotics/engineai_ros2_workspace): [Link to file](https://github.com/engineai-robotics/engineai_ros2_workspace/blob/community/src/simulation/mujoco/assets/resource) 
* [Fourier N1](https://github.com/FFTAI/Wiki-GRx-Gym): [Link to file](https://github.com/FFTAI/Wiki-GRx-Gym/tree/FourierN1/legged_gym/resources/robots/N1)
* [Toddlerbot](https://github.com/hshi74/toddlerbot): [Link to file](https://github.com/hshi74/toddlerbot/tree/main/toddlerbot/descriptions/toddlerbot_active)
* [Unitree G1](https://github.com/unitreerobotics/unitree_ros): [Link to file](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/g1_description)
* [HighToqure Hi](https://www.hightorquerobotics.com/hi/)
* [Galaxea R1 Pro](https://galaxea-dynamics.com/): MIT license
* [LEJU Kuavo S45](https://gitee.com/leju-robot/kuavo-ros-opensource/blob/master/LICENSE): MIT license
* [Berkley Humanoid Lite](https://github.com/HybridRobotics/Berkeley-Humanoid-Lite-Assets): CC-BY-SA-4.0 license
* [Booster K1](https://www.boosterobotics.com/)
