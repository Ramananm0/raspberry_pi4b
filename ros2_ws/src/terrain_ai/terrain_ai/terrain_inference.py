"""
Terrain Patch-Grid Inference Engine
- EfficientNet-B4 backbone (patch-based, matches training)
- Divides the camera frame into a grid, classifies each cell independently
- Returns per-cell terrain labels + confidence + overall safety level
"""

import json
import os
import torch
import torch.nn as nn
import numpy as np
from torchvision import transforms, models
from PIL import Image

MODEL_DIR = os.path.expanduser('~/terrain_dataset/model')

# Grid config
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


class TerrainGridEngine:
    def __init__(self, model_dir=MODEL_DIR):
        meta_path = os.path.join(model_dir, 'model_meta.json')
        with open(meta_path) as f:
            self.meta = json.load(f)

        self.classes    = self.meta['classes']
        self.safety_map = self.meta['safety']
        self.threshold  = self.meta.get('threshold', CONF_THRESH)
        self.patch_size = self.meta.get('patch_size', 224)
        self.device     = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model      = self._load_model(model_dir)
        self.transform  = transforms.Compose([
            transforms.Resize((self.patch_size, self.patch_size)),
            transforms.ToTensor(),
            transforms.Normalize([0.485, 0.456, 0.406],
                                  [0.229, 0.224, 0.225]),
        ])
        self.level_priority = {'STOP': 4, 'UNSAFE': 3, 'CAUTION': 2, 'SAFE': 1}

    def _load_model(self, model_dir):
        n     = len(self.classes)
        model = models.efficientnet_b4(weights=None)
        in_f  = model.classifier[1].in_features   # 1792
        model.classifier = nn.Sequential(
            nn.Dropout(p=0.5, inplace=True),
            nn.Linear(in_f, n),
        )
        weights = torch.load(
            os.path.join(model_dir, 'best_model.pth'),
            map_location=self.device,
            weights_only=True,
        )
        model.load_state_dict(weights)
        return model.to(self.device).eval()

    def _make_grid(self, img: Image.Image):
        W, H   = img.size
        y_top  = int(H * GROUND_START)
        gH     = H - y_top
        cW, cH = W // GRID_COLS, gH // GRID_ROWS

        cells = []
        for r in range(GRID_ROWS):
            for c in range(GRID_COLS):
                x0, y0 = c * cW, y_top + r * cH
                x1, y1 = x0 + cW, y0 + cH
                cells.append({
                    'crop': img.crop((x0, y0, x1, y1)),
                    'row': r, 'col': c,
                    'bbox': (x0, y0, x1, y1),
                })
        return cells

    @torch.no_grad()
    def _classify_batch(self, crops):
        import torch.nn.functional as F
        tensors = torch.stack([self.transform(c) for c in crops]).to(self.device)
        logits  = self.model(tensors)
        probs   = F.softmax(logits, dim=1).cpu().numpy()
        results = []
        for row in probs:
            top      = int(np.argmax(row))
            top_cls  = self.classes[top]
            top_conf = float(row[top])
            dets = {top_cls: top_conf}
            for i, conf in enumerate(row):
                if i != top and conf >= self.threshold:
                    dets[self.classes[i]] = float(conf)
            results.append(dets)
        return results

    def _worst_safety(self, terrain_dict):
        level, speed = 'SAFE', 1.0
        for cls in terrain_dict:
            info = self.safety_map.get(cls, {'level': 'CAUTION', 'speed': 0.5})
            if self.level_priority.get(info['level'], 0) > self.level_priority.get(level, 0):
                level, speed = info['level'], info['speed']
            elif self.level_priority.get(info['level'], 0) == self.level_priority.get(level, 0):
                speed = min(speed, info['speed'])
        return level, speed

    def analyse(self, img: Image.Image):
        cells  = self._make_grid(img)
        crops  = [c['crop'] for c in cells]
        scores = self._classify_batch(crops)

        all_terrains = {}
        for cell, dets in zip(cells, scores):
            lvl, spd = self._worst_safety(dets)
            cell['terrains'] = dets
            cell['safety']   = lvl
            cell['speed']    = spd
            for cls, conf in dets.items():
                if cls not in all_terrains or conf > all_terrains[cls]:
                    all_terrains[cls] = conf

        overall_level, overall_speed = self._worst_safety(all_terrains)
        flag = overall_level in ('UNSAFE', 'STOP')

        col_labels = {0: 'Left', 1: 'Centre', 2: 'Right'} if GRID_COLS == 3 else {}
        col_summary = {}
        for cell in cells:
            c = cell['col']
            top_terrain = max(cell['terrains'], key=cell['terrains'].get)
            col_summary.setdefault(c, set()).add(top_terrain)

        spatial = ' | '.join(
            f"{col_labels.get(c, f'Col{c}')}: {', '.join(sorted(ts))}"
            for c, ts in sorted(col_summary.items())
        )
        level_msg = {
            'SAFE':   'proceed normally',
            'CAUTION': 'slow down',
            'UNSAFE': 'WARNING — moving slow',
            'STOP':   'STOP — obstacle/impassable',
        }
        message = f"{spatial} → {overall_level} ({level_msg[overall_level]}, speed={overall_speed:.2f})"

        return {
            'grid':         cells,
            'all_terrains': all_terrains,
            'safety_level': overall_level,
            'speed_factor': overall_speed,
            'flag':         flag,
            'message':      message,
            'grid_rows':    GRID_ROWS,
            'grid_cols':    GRID_COLS,
        }
