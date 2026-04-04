import os
from pathlib import Path

# Get the project roots
PROJECT_ROOT = Path(__file__).parent.parent.parent.parent.absolute()
PYTHON_DIR = PROJECT_ROOT / "python"
DATASET_DIR = PYTHON_DIR / "dataset"

# Subdirectories
RAW_IMAGES_DIR = DATASET_DIR / "images"
ANNOTATIONS_DIR = DATASET_DIR / "annotations"
YOLO_DATASET_DIR = DATASET_DIR / "yolo_format"

# Create directories that may not exist
RAW_IMAGES_DIR.mkdir(parents=True, exist_ok=True)
ANNOTATIONS_DIR.mkdir(parents=True, exist_ok=True)
YOLO_DATASET_DIR.mkdir(parents=True, exist_ok=True)

# Object Classes for detection
OBJECT_CLASSES = {
    0: "pencil",
    1: "mug",
    2: "sticky_note",
    3: "mouse",
    4: "pen"
}

# Reverse mapping for convenience
CLASS_TO_ID = {v: k for k, v in OBJECT_CLASSES.items()}

# Camera settigs
CAMERA_INDEX = 0
CAMERA_FRAME_WIDTH = 1280
CAMERA_FRAME_HEIGHT = 720
CAMERA_FPS = 30

# Image capture settings
CAPTURE_QUALITY = 95