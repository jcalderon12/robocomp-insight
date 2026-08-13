import glob
import os
from uuid import uuid4

import cv2


class YoloDataset:
    """Builds a YOLO-format detection dataset on disk under
    <base_path>/<dataset_name>/{images,labels}/{train,val}, plus the data.yaml
    manifest Ultralytics reads to train.

    add_image/add_negative_image only write to disk, with no in-memory
    bookkeeping: the dataset-generation code that calls them runs across
    forked worker processes (multiprocessing), whose in-memory state never
    returns to the parent process, so counting has to happen by reading the
    filesystem instead (see get_dataset_size).
    """

    def __init__(self, base_path: str, dataset_name: str, class_name: str):
        self.root = os.path.join(base_path, dataset_name)
        self.class_name = class_name
        for split in ("train", "val"):
            os.makedirs(os.path.join(self.root, "images", split), exist_ok=True)
            os.makedirs(os.path.join(self.root, "labels", split), exist_ok=True)

    def _split_dir(self, is_train: bool) -> str:
        return "train" if is_train else "val"

    def add_image(self, img_bgr, label: str, is_train: bool) -> None:
        """Save an image with its YOLO-format label ("<class> <cx> <cy> <w> <h>").
            Parameters:
                - img_bgr: image array (OpenCV BGR), saved as-is, unmasked.
                - label (str): YOLO label line(s) for this image.
                - is_train (bool): whether this image belongs to the train split.
        """
        split = self._split_dir(is_train)
        name = uuid4().hex
        cv2.imwrite(os.path.join(self.root, "images", split, f"{name}.jpg"), img_bgr)
        with open(os.path.join(self.root, "labels", split, f"{name}.txt"), "w") as f:
            f.write(label.strip() + "\n")

    def add_negative_image(self, img_bgr, is_train: bool) -> None:
        """Save an image with no object present: image plus an empty label file.
            Parameters:
                - img_bgr: image array (OpenCV BGR).
                - is_train (bool): whether this image belongs to the train split.
        """
        split = self._split_dir(is_train)
        name = uuid4().hex
        cv2.imwrite(os.path.join(self.root, "images", split, f"{name}.jpg"), img_bgr)
        open(os.path.join(self.root, "labels", split, f"{name}.txt"), "w").close()

    def get_dataset_size(self) -> int:
        """Total number of images currently on disk (train + val)."""
        return sum(
            len(glob.glob(os.path.join(self.root, "images", split, "*.jpg")))
            for split in ("train", "val")
        )

    def create_yaml(self) -> None:
        """Write the data.yaml manifest Ultralytics reads to locate the dataset."""
        with open(self.get_data_yaml_path(), "w") as f:
            f.write(f"path: {self.root}\n")
            f.write("train: images/train\n")
            f.write("val: images/val\n")
            f.write("nc: 1\n")
            f.write(f"names: ['{self.class_name}']\n")

    def get_data_yaml_path(self) -> str:
        return os.path.join(self.root, "data.yaml")
