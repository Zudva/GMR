#!/usr/bin/env python
"""Convert saved robot motion .pkl (as produced by smplx_to_robot.py / bvh_to_robot.py) to PyTorch .pt
and optionally back.

Usage:
  Forward (pkl -> pt):
    python scripts/convert_motion_pkl_to_pt.py --input out/aiming1_g1.pkl --output out/aiming1_g1.pt
  Backward (pt -> pkl):
    python scripts/convert_motion_pkl_to_pt.py --input out/aiming1_g1.pt --output out/aiming1_g1_back.pkl

Notes:
  - The saved root_rot in pickle is stored in xyzw order (as produced by saving scripts). We keep it AS-IS.
  - No numerical changes are applied except tensor conversion.
  - Non-numpy fields are copied verbatim.
"""

import argparse
import pickle
import sys
from pathlib import Path
from typing import Any, Dict

import numpy as np
import torch


def load_any(path: Path) -> Dict[str, Any]:
    if path.suffix == ".pkl":
        with open(path, "rb") as f:
            return pickle.load(f)
    if path.suffix == ".pt":
        obj = torch.load(path, map_location="cpu")
        # ensure plain dict
        if not isinstance(obj, dict):
            raise TypeError("Expected dict in .pt file")
        return obj
    raise ValueError("Unsupported input extension (need .pkl or .pt)")


def to_torch_dict(d: Dict[str, Any]) -> Dict[str, Any]:
    out = {}
    for k, v in d.items():
        if isinstance(v, np.ndarray):
            out[k] = torch.from_numpy(v)
        else:
            out[k] = v
    return out


def to_pickle_dict(d: Dict[str, Any]) -> Dict[str, Any]:
    out = {}
    for k, v in d.items():
        if isinstance(v, torch.Tensor):
            out[k] = v.detach().cpu().numpy()
        else:
            out[k] = v
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True, help="Input motion file (.pkl or .pt)")
    ap.add_argument("--output", required=True, help="Output motion file (.pt or .pkl)")
    ap.add_argument("--force", action="store_true", help="Overwrite existing output")
    ap.add_argument("--summary", action="store_true", help="Print shape/type summary and exit")
    args = ap.parse_args()

    in_path = Path(args.input)
    out_path = Path(args.output)

    if out_path.exists() and not args.force:
        print(f"[error] Output {out_path} exists (use --force)")
        sys.exit(1)

    data = load_any(in_path)

    if args.summary:
        print("--- Motion Summary ---")
        for k, v in data.items():
            if isinstance(v, (np.ndarray, torch.Tensor)):
                print(f"{k}: shape={tuple(v.shape)} dtype={v.dtype}")
            else:
                print(f"{k}: type={type(v)}")
        return

    if out_path.suffix == ".pt":
        torch.save(to_torch_dict(data), out_path)
        print(f"Saved torch tensor dict to {out_path}")
    elif out_path.suffix == ".pkl":
        pickle.dump(to_pickle_dict(data), open(out_path, "wb"))
        print(f"Saved pickle motion to {out_path}")
    else:
        print("[error] Output extension must be .pt or .pkl")
        sys.exit(1)


if __name__ == "__main__":
    main()
