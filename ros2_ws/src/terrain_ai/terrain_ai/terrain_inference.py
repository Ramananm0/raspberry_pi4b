"""
Terrain Patch-Grid Inference Engine
- EfficientNet-B4 backbone (patch-based, matches training)
- Divides the camera frame into a grid, classifies each cell independently
- Returns per-cell terrain labels + confidence + overall safety level
- Runtime backend: ONNX Runtime (onnxruntime) for best ARM/Pi 4B performance;
  falls back to PyTorch if the .onnx file is absent.
"""

import json
import os

import numpy as np
from PIL import Image

MODEL_DIR = os.path.expanduser('~/terrain_dataset/model')

GRID_ROWS    = 3
GRID_COLS    = 3
GROUND_START = 0.40   # skip top 40% of frame (sky/background)
CONF_THRESH  = 0.45

LEVEL_COLOUR = {
    'SAFE':    (76,  175, 80),
    'CAUTION': (255, 193, 7),
    'UNSAFE':  (244, 67,  54),
    'STOP':    (156, 39,  176),
}

# ── Image pre-processing (pure numpy/PIL, no PyTorch dependency) ──────────────
_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
_STD  = np.array([0.229, 0.224, 0.225], dtype=np.float32)

def _preprocess(pil_img: Image.Image, size: int = 224) -> np.ndarray:
    img = pil_img.resize((size, size), Image.BILINEAR).convert("RGB")
    arr = np.asarray(img, dtype=np.float32) / 255.0          # (H,W,3)
    arr = (arr - _MEAN) / _STD                                # normalise
    return arr.transpose(2, 0, 1)                             # (3,H,W)


# ── Backends ──────────────────────────────────────────────────────────────────
class _OrtBackend:
    """ONNX Runtime backend — preferred on Pi 4B ARM (uses NEON SIMD)."""

    def __init__(self, model_dir: str, patch_size: int):
        import onnxruntime as ort
        onnx_path = os.path.join(model_dir, "best_model.onnx")
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = 4          # use all Pi 4B cores
        opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        self._sess     = ort.InferenceSession(
            onnx_path, opts, providers=["CPUExecutionProvider"]
        )
        self._inp_name = self._sess.get_inputs()[0].name
        self._patch    = patch_size
        print(f"[TerrainInference] ONNX-RT backend  →  {onnx_path}")

    def run(self, crops: list) -> np.ndarray:
        batch = np.stack([_preprocess(c, self._patch) for c in crops])  # (N,3,H,W)
        return self._sess.run(None, {self._inp_name: batch})[0]          # (N, n_cls)


class _TorchBackend:
    """PyTorch fallback (slower on Pi, kept for compatibility)."""

    def __init__(self, model_dir: str, patch_size: int, classes: list):
        import torch
        import torch.nn as nn
        from torchvision import models, transforms

        n     = len(classes)
        model = models.efficientnet_b4(weights=None)
        in_f  = model.classifier[1].in_features
        model.classifier = nn.Sequential(
            nn.Dropout(p=0.5, inplace=True),
            nn.Linear(in_f, n),
        )
        w = torch.load(
            os.path.join(model_dir, "best_model.pth"),
            map_location="cpu",
            weights_only=True,
        )
        model.load_state_dict(w)
        self._model = model.eval()
        self._patch = patch_size
        self._tf = transforms.Compose([
            transforms.Resize((patch_size, patch_size)),
            transforms.ToTensor(),
            transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
        ])
        print(f"[TerrainInference] PyTorch fallback backend  →  "
              f"{os.path.join(model_dir, 'best_model.pth')}")

    def run(self, crops: list) -> np.ndarray:
        import torch
        import torch.nn.functional as F
        t = torch.stack([self._tf(c) for c in crops])
        with torch.no_grad():
            return self._model(t).numpy()   # (N, n_cls)


# ── Main engine ───────────────────────────────────────────────────────────────
class TerrainGridEngine:

    def __init__(self, model_dir: str = MODEL_DIR):
        meta_path = os.path.join(model_dir, "model_meta.json")
        with open(meta_path) as f:
            self.meta = json.load(f)

        self.classes    = self.meta["classes"]
        self.safety_map = self.meta["safety"]
        self.threshold  = self.meta.get("threshold", CONF_THRESH)
        self.patch_size = self.meta.get("patch_size", 224)

        # Choose backend: ONNX-RT if .onnx exists, else PyTorch
        onnx_path = os.path.join(model_dir, "best_model.onnx")
        try:
            self._backend = _OrtBackend(model_dir, self.patch_size)
        except Exception as e:
            print(f"[TerrainInference] ORT unavailable ({e}), falling back to PyTorch")
            self._backend = _TorchBackend(model_dir, self.patch_size, self.classes)

        self.level_priority = {"STOP": 4, "UNSAFE": 3, "CAUTION": 2, "SAFE": 1}

    # ── Internal helpers ──────────────────────────────────────────────────────
    def _make_grid(self, img: Image.Image):
        W, H   = img.size
        y_top  = int(H * GROUND_START)
        gH     = H - y_top
        cW, cH = W // GRID_COLS, gH // GRID_ROWS
        cells  = []
        for r in range(GRID_ROWS):
            for c in range(GRID_COLS):
                x0, y0 = c * cW, y_top + r * cH
                cells.append({
                    "crop": img.crop((x0, y0, x0+cW, y0+cH)),
                    "row": r, "col": c,
                    "bbox": (x0, y0, x0+cW, y0+cH),
                })
        return cells

    def _classify_batch(self, crops):
        logits = self._backend.run(crops)                      # (N, n_cls)
        exp    = np.exp(logits - logits.max(axis=1, keepdims=True))
        probs  = exp / exp.sum(axis=1, keepdims=True)          # softmax

        results = []
        for row in probs:
            top     = int(np.argmax(row))
            top_cls = self.classes[top]
            top_c   = float(row[top])
            dets = {top_cls: top_c}
            for i, c in enumerate(row):
                if i != top and float(c) >= self.threshold:
                    dets[self.classes[i]] = float(c)
            results.append(dets)
        return results

    def _worst_safety(self, terrain_dict):
        level, speed = "SAFE", 1.0
        for cls in terrain_dict:
            info = self.safety_map.get(cls, {"level": "CAUTION", "speed": 0.5})
            p = self.level_priority.get(info["level"], 0)
            if p > self.level_priority.get(level, 0):
                level, speed = info["level"], info["speed"]
            elif p == self.level_priority.get(level, 0):
                speed = min(speed, info["speed"])
        return level, speed

    # ── Public API ────────────────────────────────────────────────────────────
    def analyse(self, img: Image.Image) -> dict:
        cells  = self._make_grid(img)
        scores = self._classify_batch([c["crop"] for c in cells])

        all_terrains = {}
        for cell, dets in zip(cells, scores):
            lvl, spd      = self._worst_safety(dets)
            cell["terrains"] = dets
            cell["safety"]   = lvl
            cell["speed"]    = spd
            for cls, conf in dets.items():
                if cls not in all_terrains or conf > all_terrains[cls]:
                    all_terrains[cls] = conf

        overall_level, overall_speed = self._worst_safety(all_terrains)
        flag = overall_level in ("UNSAFE", "STOP")

        col_labels  = {0: "Left", 1: "Centre", 2: "Right"} if GRID_COLS == 3 else {}
        col_summary = {}
        for cell in cells:
            top_t = max(cell["terrains"], key=cell["terrains"].get)
            col_summary.setdefault(cell["col"], set()).add(top_t)

        spatial = " | ".join(
            f"{col_labels.get(c, f'Col{c}')}: {', '.join(sorted(ts))}"
            for c, ts in sorted(col_summary.items())
        )
        level_msg = {
            "SAFE":    "proceed normally",
            "CAUTION": "slow down",
            "UNSAFE":  "WARNING — moving slow",
            "STOP":    "STOP — obstacle/impassable",
        }
        message = (
            f"{spatial} → {overall_level} "
            f"({level_msg[overall_level]}, speed={overall_speed:.2f})"
        )

        return {
            "grid":         cells,
            "all_terrains": all_terrains,
            "safety_level": overall_level,
            "speed_factor": overall_speed,
            "flag":         flag,
            "message":      message,
            "grid_rows":    GRID_ROWS,
            "grid_cols":    GRID_COLS,
        }
