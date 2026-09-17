#!/usr/bin/env python3
"""Prueba rapida: descriptor global DINOv2 (token CLS) sobre bump_dataset_rgb (frame
completo, sin segmentar, SIN redimensionar) + clasificador sklearn para ver cuanto discrimina.

Pipeline: imagen -> recorte al multiplo de 14 mas cercano (resolucion nativa, sin resize)
+ normalizacion -> patch embed + ViT (DINOv2) -> token CLS (vector D-dim) -> uno por imagen
-> varios clasificadores con validacion cruzada (dataset pequeno, no separamos train/test
a mano).

Sin resize hay muchos mas parches por imagen (720x1280 nativo -> ~4641 parches, frente a
los 256 de la version a 224x224), asi que tarda bastante mas por imagen.

Uso:
  python3 dinov2_probe.py                       # dinov2_vits14 (rapido, 384-dim)
  python3 dinov2_probe.py --model dinov2_vitb14  # mas grande (768-dim)
"""
import argparse
import json
from pathlib import Path

import joblib
import numpy as np
import torch
from PIL import Image
from torchvision import transforms
from sklearn.ensemble import RandomForestClassifier
from sklearn.linear_model import LogisticRegression
from sklearn.neighbors import KNeighborsClassifier
from sklearn.model_selection import StratifiedKFold, cross_val_predict
from sklearn.metrics import accuracy_score, classification_report, confusion_matrix

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SRC = ROOT / "segmented_objects" / "bump_dataset_rgb"
MODELS_DIR = Path(__file__).resolve().parent / "models"  # backbone + classifiers, for dinov2_infer.py

DEVICE = "cuda" if torch.cuda.is_available() else "cpu"

# DINOv2 exige que alto y ancho sean multiplos del patch size (14). Sin resize, en vez de
# forzar un tamano fijo, recortamos al multiplo de 14 mas cercano por abajo (perdida de
# como mucho 13px por lado, imperceptible) para conservar la resolucion nativa.
PATCH_SIZE = 14
NORMALIZE = transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])


def _crop_to_patch_multiple(img: Image.Image, patch: int = PATCH_SIZE) -> Image.Image:
    w, h = img.size
    new_w, new_h = (w // patch) * patch, (h // patch) * patch
    left, top = (w - new_w) // 2, (h - new_h) // 2
    return img.crop((left, top, left + new_w, top + new_h))


def preprocess(img: Image.Image) -> torch.Tensor:
    img = _crop_to_patch_multiple(img)
    x = transforms.functional.to_tensor(img)
    return NORMALIZE(x)


def load_model(name: str):
    model = torch.hub.load("facebookresearch/dinov2", name)
    model.eval().to(DEVICE)
    return model


def save_backbone(model, name: str) -> Path:
    """Save the backbone's weights standalone (state_dict), so dinov2_infer.py can
    reload it later offline without hitting torch.hub again."""
    MODELS_DIR.mkdir(parents=True, exist_ok=True)
    path = MODELS_DIR / f"{name}.pt"
    torch.save(model.state_dict(), path)
    meta_path = MODELS_DIR / f"{name}.json"
    meta_path.write_text(json.dumps({"model": name, "embed_dim": int(model.embed_dim)}, indent=2))
    return path


def extract_cls(model, img_path: Path) -> np.ndarray:
    img = Image.open(img_path).convert("RGB")
    x = preprocess(img).unsqueeze(0).to(DEVICE)
    with torch.no_grad():
        cls = model(x)  # forward por defecto de los hub models = token CLS, shape (1, D)
    return cls.squeeze(0).cpu().numpy()


def load_split(src: Path, model) -> tuple[np.ndarray, np.ndarray, list]:
    X, y, paths = [], [], []
    for label, sub in ((1, "with_bump"), (0, "without_bump")):
        files = sorted(src.glob(f"{sub}/*_rgb.jpg"))
        print(f"[{sub}] {len(files)} imagenes")
        for f in files:
            X.append(extract_cls(model, f))
            y.append(label)
            paths.append(f)
    return np.array(X), np.array(y), paths


def _offdiag_mean(m: np.ndarray) -> float:
    n = m.shape[0]
    return float((m.sum() - np.trace(m)) / max(n * n - n, 1))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--src", type=Path, default=DEFAULT_SRC, help="dataset con with_bump/ without_bump/")
    ap.add_argument("--model", default="dinov2_vits14",
                    choices=["dinov2_vits14", "dinov2_vitb14", "dinov2_vitl14", "dinov2_vitg14"])
    ap.add_argument("--folds", type=int, default=5)
    args = ap.parse_args()

    print(f"Cargando {args.model} en {DEVICE}...")
    model = load_model(args.model)
    print(f"Dim del descriptor (CLS): {model.embed_dim}")
    backbone_path = save_backbone(model, args.model)
    print(f"Backbone guardado en {backbone_path}")

    src = args.src.resolve()
    X, y, paths = load_split(src, model)
    print(f"X: {X.shape}  y: {y.shape}  (1=with_bump, 0=without_bump)")

    out = src / "dinov2_embeddings.npz"
    np.savez(out, X=X, y=y, paths=np.array([str(p) for p in paths]))
    print(f"Embeddings guardados en {out}")

    # Sanity check barato, sin entrenar nada: similitud coseno media dentro/entre clases.
    Xn = X / np.linalg.norm(X, axis=1, keepdims=True)
    sim = Xn @ Xn.T
    sim_with = _offdiag_mean(sim[np.ix_(y == 1, y == 1)])
    sim_without = _offdiag_mean(sim[np.ix_(y == 0, y == 0)])
    sim_cross = float(sim[np.ix_(y == 1, y == 0)].mean())
    print(f"\nSimilitud coseno media: with-with={sim_with:.4f}  without-without={sim_without:.4f}  "
          f"with-without={sim_cross:.4f}")
    print("(si with-without no es claramente menor que las dos anteriores, el descriptor global\n"
          " no separa bien las clases -> esperado con frame completo, ver recomendaciones)")

    # Varios clasificadores, misma validacion cruzada, para comparar (70 imagenes -> no
    # separamos train/test a mano). kNN con distancia coseno es el protocolo de evaluacion
    # "de fabrica" que usa el propio paper de DINOv2 sobre sus embeddings.
    classifiers = {
        "LogisticRegression": LogisticRegression(max_iter=2000, C=1.0),
        "RandomForest": RandomForestClassifier(n_estimators=300, random_state=0),
        "kNN (k=5, coseno)": KNeighborsClassifier(n_neighbors=5, metric="cosine"),
    }
    cv = StratifiedKFold(n_splits=args.folds, shuffle=True, random_state=0)

    summary = []
    for name, clf in classifiers.items():
        y_pred = cross_val_predict(clf, X, y, cv=cv)
        acc = accuracy_score(y, y_pred)
        summary.append((name, acc))
        print(f"\n=== {name}, validacion cruzada ({args.folds} folds) ===")
        print(classification_report(y, y_pred, target_names=["without_bump", "with_bump"]))
        print("Matriz de confusion (filas=real, cols=predicho):\n", confusion_matrix(y, y_pred))

        # Reentrena con TODO el dataset (la validacion cruzada de arriba es solo para medir;
        # este es el modelo que se guarda para usar luego en dinov2_infer.py).
        clf.fit(X, y)
        clf_path = MODELS_DIR / f"{args.model}_{name.split(' ')[0]}.joblib"
        joblib.dump(clf, clf_path)
        print(f"Clasificador guardado en {clf_path}")

    print("\n=== Resumen ===")
    for name, acc in sorted(summary, key=lambda kv: -kv[1]):
        print(f"  {name:<22} accuracy={acc * 100:.1f}%")


if __name__ == "__main__":
    main()
