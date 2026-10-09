#!/usr/bin/env python3
"""Pasa todas las RGB de bump_dataset por SAM2 y por YOLO26-seg.

Salida:
  segmented_objects/output/sam2/<subcarpeta>/<nombre>...
  segmented_objects/output/yolo26sem/<subcarpeta>/<nombre>...

Por cada imagen se guarda el overlay (_overlay.jpg) y la mascara combinada (_mask.png).
"""
import argparse
from pathlib import Path

import cv2
import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SRC = ROOT / "segmented_objects" / "bump_dataset"
DEFAULT_OUT = ROOT / "segmented_objects" / "output"

SAM2_MODEL = ROOT / "sam2.1_l.pt"
YOLO26SEM_CANDIDATES = [    
    "yolo26m-sem.pt"
]

DEVICE = "cuda" if torch.cuda.is_available() else "cpu"


def _list_images(src: Path) -> list[Path]:
    imgs = sorted(src.rglob("*_rgb.jpg"))
    if not imgs:
        imgs = sorted(p for p in src.rglob("*") if p.suffix.lower() in (".jpg", ".jpeg", ".png"))
    return imgs


def _combined_mask(result) -> np.ndarray | None:
    if result.masks is None or result.masks.data is None or len(result.masks.data) == 0:
        return None
    m = result.masks.data.cpu().numpy()  # (n, H, W) en [0,1]
    return (m.max(axis=0) > 0.5).astype(np.uint8) * 255


def _resolve_yolo_model() -> Path:
    for c in YOLO26SEM_CANDIDATES:
        return c

    raise FileNotFoundError(f"No se encontro yolo26m-sem.pt en: {YOLO26SEM_CANDIDATES}")


def run_sam2(images: list[Path], src: Path, out_dir: Path) -> None:
    from ultralytics import SAM

    model = SAM(str(SAM2_MODEL))
    print(f"[SAM2] modelo {SAM2_MODEL.name} | device {DEVICE} | {len(images)} imagenes")
    for i, f in enumerate(images, 1):
        rel = f.relative_to(src)
        dst_base = out_dir / rel
        dst_base.parent.mkdir(parents=True, exist_ok=True)
        img = cv2.imread(str(f), cv2.IMREAD_COLOR)
        res = model(img, device=DEVICE, verbose=False)[0]  # modo "everything"
        cv2.imwrite(str(dst_base.with_name(dst_base.stem + "_overlay.jpg")), res.plot())
        mask = _combined_mask(res)
        if mask is not None:
            cv2.imwrite(str(dst_base.with_name(dst_base.stem + "_mask.png")), mask)
        print(f"  [{i}/{len(images)}] {rel}")


def run_yolo26sem(images: list[Path], src: Path, out_dir: Path) -> None:
    from ultralytics import YOLO

    # model_path = _resolve_yolo_model()
    model = YOLO("yolo26m-sem.pt")
    # print(f"[YOLO26-sem] modelo {model_path.name} | device {DEVICE} | {len(images)} imagenes")
    for i, f in enumerate(images, 1):
        rel = f.relative_to(src)
        dst_base = out_dir / rel
        dst_base.parent.mkdir(parents=True, exist_ok=True)
        img = cv2.imread(str(f), cv2.IMREAD_COLOR)
        res = model.predict(img, device=DEVICE, verbose=False)[0]
        cv2.imwrite(str(dst_base.with_name(dst_base.stem + "_overlay.jpg")), res.plot())
        mask = _combined_mask(res)
        if mask is not None:
            cv2.imwrite(str(dst_base.with_name(dst_base.stem + "_mask.png")), mask)
        n = 0 if res.boxes is None else len(res.boxes)
        print(f"  [{i}/{len(images)}] {rel} -> {n} detecciones")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--src", type=Path, default=DEFAULT_SRC, help="dataset de entrada")
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT, help="carpeta de salida")
    ap.add_argument("--only", choices=("sam2", "yolo26sem"), help="ejecutar solo un modelo")
    args = ap.parse_args()

    src = args.src.resolve()
    images = _list_images(src)
    if not images:
        raise SystemExit(f"No hay imagenes en {src}")

    if args.only in (None, "sam2"):
        run_sam2(images, src, (args.out / "sam2").resolve())
    if args.only in (None, "yolo26sem"):
        run_yolo26sem(images, src, (args.out / "yolo26sem").resolve())


if __name__ == "__main__":
    main()
