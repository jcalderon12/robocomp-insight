#!/usr/bin/env python3
"""Pasa una imagen propia por un backbone DINOv2 guardado (por dinov2_probe.py) y,
si hay clasificadores guardados para ese modelo, predice con ellos.

Uso:
  python3 dinov2_infer.py foto.jpg
  python3 dinov2_infer.py foto.jpg --model dinov2_vits14 --classifier RandomForest
"""
import argparse
import json
from pathlib import Path

import joblib
import numpy as np
import torch
from PIL import Image

from dinov2_probe import MODELS_DIR, preprocess, DEVICE

LABELS = {0: "without_bump", 1: "with_bump"}


def load_backbone(name: str):
    meta_path = MODELS_DIR / f"{name}.json"
    weights_path = MODELS_DIR / f"{name}.pt"
    if not weights_path.exists():
        raise FileNotFoundError(f"No hay backbone guardado en {weights_path}. "
                                 f"Lanza antes dinov2_probe.py --model {name}.")
    meta = json.loads(meta_path.read_text()) if meta_path.exists() else {}

    model = torch.hub.load("facebookresearch/dinov2", name, pretrained=False)
    model.load_state_dict(torch.load(weights_path, map_location=DEVICE))
    model.eval().to(DEVICE)
    print(f"Backbone {name} cargado desde {weights_path} (dim={meta.get('embed_dim', model.embed_dim)})")
    return model


def extract_cls(model, image_path: Path) -> np.ndarray:
    img = Image.open(image_path).convert("RGB")
    x = preprocess(img).unsqueeze(0).to(DEVICE)
    with torch.no_grad():
        cls = model(x)
    return cls.squeeze(0).cpu().numpy()


def find_classifiers(model_name: str, only: str | None) -> list[Path]:
    prefix = f"{model_name}_"
    paths = sorted(p for p in MODELS_DIR.glob(f"{prefix}*.joblib"))
    if only:
        paths = [p for p in paths if p.stem.removeprefix(prefix) == only]
    return paths


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("image", type=Path, help="imagen propia a analizar")
    ap.add_argument("--model", default="dinov2_vits14",
                    choices=["dinov2_vits14", "dinov2_vitb14", "dinov2_vitl14", "dinov2_vitg14"])
    ap.add_argument("--classifier", default=None,
                    help="nombre del clasificador a usar (LogisticRegression, RandomForest, kNN). "
                         "Por defecto prueba todos los guardados para ese modelo.")
    args = ap.parse_args()

    if not args.image.is_file():
        raise SystemExit(f"No existe la imagen: {args.image}")

    model = load_backbone(args.model)
    embedding = extract_cls(model, args.image)
    print(f"Embedding: shape={embedding.shape} norma={np.linalg.norm(embedding):.4f}")

    clf_paths = find_classifiers(args.model, args.classifier)
    if not clf_paths:
        print(f"No hay clasificadores guardados para {args.model}"
              + (f" con nombre '{args.classifier}'" if args.classifier else "")
              + ". Lanza antes dinov2_probe.py para generarlos.")
        return

    print()
    x = embedding.reshape(1, -1)
    prefix = f"{args.model}_"
    for clf_path in clf_paths:
        clf_name = clf_path.stem.removeprefix(prefix)
        clf = joblib.load(clf_path)
        pred = int(clf.predict(x)[0])
        line = f"{clf_name:<20} -> {LABELS[pred]}"
        if hasattr(clf, "predict_proba"):
            proba = clf.predict_proba(x)[0]
            line += f"  (p(without)={proba[0]:.3f}, p(with)={proba[1]:.3f})"
        print(line)


if __name__ == "__main__":
    main()
