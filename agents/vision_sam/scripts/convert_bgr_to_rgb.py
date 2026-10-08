#!/usr/bin/env python3
"""Copia bump_dataset a un dataset nuevo invirtiendo el orden de canales de las RGB (BGR<->RGB).

- Mantiene la estructura de carpetas (with_bump/ without_bump/).
- Los *_rgb.jpg se guardan con los canales invertidos.
- Los *_depth.npy se copian sin tocar.
"""
import argparse
import shutil
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SRC = ROOT / "segmented_objects" / "bump_dataset"
DEFAULT_DST = ROOT / "segmented_objects" / "bump_dataset_rgb"


def convert_dataset(src: Path, dst: Path) -> None:
    rgb_files = sorted(src.rglob("*_rgb.jpg"))
    depth_files = sorted(src.rglob("*_depth.npy"))
    print(f"Origen : {src}")
    print(f"Destino: {dst}")
    print(f"{len(rgb_files)} imagenes RGB | {len(depth_files)} mapas de profundidad")

    for f in rgb_files:
        rel = f.relative_to(src)
        out = dst / rel
        out.parent.mkdir(parents=True, exist_ok=True)
        img = cv2.imread(str(f), cv2.IMREAD_COLOR)  # canales tal cual estan en el fichero
        if img is None:
            print(f"  [SKIP] no se pudo leer {f}")
            continue
        swapped = np.ascontiguousarray(img[:, :, ::-1])  # invierte orden de canales
        cv2.imwrite(str(out), swapped)

    for f in depth_files:
        rel = f.relative_to(src)
        out = dst / rel
        out.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(f, out)  # profundidad intacta

    print("Hecho.")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--src", type=Path, default=DEFAULT_SRC, help="dataset de entrada")
    ap.add_argument("--dst", type=Path, default=DEFAULT_DST, help="dataset de salida")
    args = ap.parse_args()
    convert_dataset(args.src.resolve(), args.dst.resolve())


if __name__ == "__main__":
    main()
