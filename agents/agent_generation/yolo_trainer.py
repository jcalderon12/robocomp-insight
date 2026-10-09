import os


class YoloTrainer:
    """Thin wrapper around ultralytics YOLO training.

    get_trained_model_path's layout (base_path/runs/detect/<output_dir>/weights/best.pt)
    is Ultralytics' own default save convention when training with
    project=base_path/runs/detect and name=output_dir; it must match exactly,
    since the generated concept_X agent checks that same path at startup to
    decide whether training is needed at all.
    """

    def __init__(self, base_path: str, dataset_path: str, model_name: str, epochs: int, output_dir: str):
        self.base_path = base_path
        self.dataset_path = dataset_path
        self.model_name = model_name
        self.epochs = epochs
        self.output_dir = output_dir
        self.model = None

    def init_training(self) -> None:
        from ultralytics import YOLO  # imported here for lazy loading
        self.model = YOLO(self.model_name)
        self.model.train(
            data=self.dataset_path,
            epochs=self.epochs,
            project=os.path.join(self.base_path, "runs", "detect"),
            name=self.output_dir,
            exist_ok=True,
        )

    def get_trained_model_path(self) -> str:
        return os.path.join(self.base_path, "runs", "detect", self.output_dir, "weights", "best.pt")
