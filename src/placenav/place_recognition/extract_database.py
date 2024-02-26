'''
Script to extract a database of global descriptors from images
with a place recognition model loaded uting the Hloc interface. 

Borrowed from
Hloc (https://github.com/cvg/Hierarchical-Localization/)
by Paul-Edouard Sarlin et al
'''

import collections.abc as collections
import glob
import pprint
from pathlib import Path
from types import SimpleNamespace
from typing import Dict, List, Optional, Union

import h5py
import numpy as np
import torch
import torchvision
from tqdm import tqdm

from placenav import logger
from placenav.models import pr_models as extractors
from placenav.models.pr_models.base_model import dynamic_load
from placenav.utils import read_image

"""
A set of standard configurations that can be directly selected from the command
line using their name. Each is a dictionary with the following entries:
    - output: the name of the feature file that will be generated.
    - model: the model configuration, as passed to a feature extractor.
    - preprocessing: how to preprocess the images read from disk.
"""

def parse_image_list(path, with_intrinsics=False):
    images = []
    with open(path, "r") as f:
        for line in f:
            line = line.strip("\n")
            if len(line) == 0 or line[0] == "#":
                continue
            name, *data = line.split()
            if with_intrinsics:
                model, width, height, *params = data
                params = np.array(params, float)
                cam = pycolmap.Camera(
                    model=model, width=int(width), height=int(height), params=params
                )
                images.append((name, cam))
            else:
                images.append(name)

    assert len(images) > 0
    logger.info(f"Imported {len(images)} images from {path.name}")
    return images


def parse_image_lists(paths, with_intrinsics=False):
    images = []
    files = list(Path(paths.parent).glob(paths.name))
    assert len(files) > 0
    for lfile in files:
        images += parse_image_list(lfile, with_intrinsics=with_intrinsics)
    return images

def list_h5_names(path):
    names = []
    with h5py.File(str(path), "r", libver="latest") as fd:

        def visit_fn(_, obj):
            if isinstance(obj, h5py.Dataset):
                names.append(obj.parent.name.strip("/"))

        fd.visititems(visit_fn)
    return list(set(names))

class ImageDataset(torch.utils.data.Dataset):
    default_conf = {
        "globs": ["*.jpg", "*.png", "*.jpeg", "*.JPG", "*.PNG"],
        "grayscale": False,
        "resize_max": None,
        "resize_force": False,
        "interpolation": "cv2_area",  # pil_linear is more accurate but slower
    }

    def __init__(self, root, conf, paths=None, image_transform=None):
        self.conf = conf = SimpleNamespace(**{**self.default_conf, **conf})
        self.root = root

        if paths is None:
            paths = []
            for g in conf.globs:
                paths += glob.glob((Path(root) / "**" / g).as_posix(), recursive=True)
            if len(paths) == 0:
                raise ValueError(f"Could not find any image in root: {root}.")
            paths = sorted(set(paths))
            self.names = [Path(p).relative_to(root).as_posix() for p in paths]
            logger.info(f"Found {len(self.names)} images in root {root}.")
        else:
            if isinstance(paths, (Path, str)):
                self.names = parse_image_lists(paths)
            elif isinstance(paths, collections.Iterable):
                self.names = [p.as_posix() if isinstance(p, Path) else p for p in paths]
            else:
                raise ValueError(f"Unknown format for path argument {paths}.")

            for name in self.names:
                if not (root / name).exists():
                    raise ValueError(f"Image {name} does not exists in root: {root}.")

        self._image_transform = image_transform

    def __getitem__(self, idx):
        name = self.names[idx]
        image = read_image(self.root / name)
        size = image.shape[:2][::-1]

        transf_img = self._image_transform(image.astype(np.uint8))

        data = {
            "image": transf_img,
            "original_size": np.array(size),
        }
        return data

    def __len__(self):
        return len(self.names)


@torch.no_grad()
def main(
    conf: Dict,
    image_dir: Path,
    image_transform: torchvision.transforms.Compose,
    export_dir: Optional[Path] = None,
    as_half: bool = True,
    image_list: Optional[Union[Path, List[str]]] = None,
    feature_path: Optional[Path] = None,
    overwrite: bool = False,
) -> Path:
    logger.info(
        "Extracting descriptors with configuration:" f"\n{pprint.pformat(conf)}"
    )
    dataset = ImageDataset(image_dir, {}, paths=image_list, image_transform=image_transform)
    if feature_path is None:
        feature_path = Path(export_dir, conf["output"] + ".h5")
    feature_path.parent.mkdir(exist_ok=True, parents=True)
    skip_names = set(
        list_h5_names(feature_path) if feature_path.exists() and not overwrite else ()
    )
    dataset.names = [n for n in dataset.names if n not in skip_names]
    if len(dataset.names) == 0:
        logger.info("Skipping the extraction.")
        return feature_path

    device = "cuda" if torch.cuda.is_available() else "cpu"
    Model = dynamic_load(extractors, conf["model"]["name"])
    model = Model(conf["model"]).eval().to(device)

    loader = torch.utils.data.DataLoader(
        dataset, num_workers=1, shuffle=False, pin_memory=True
    )
    for idx, data in enumerate(tqdm(loader)):
        name = dataset.names[idx]
        pred = model({"image": data["image"].to(device, non_blocking=True)})
        pred = {k: v[0].cpu().numpy() for k, v in pred.items()}

        pred["image_size"] = data["original_size"][0].numpy()

        if as_half:
            for k in pred:
                dt = pred[k].dtype
                if (dt == np.float32) and (dt != np.float16):
                    pred[k] = pred[k].astype(np.float16)

        with h5py.File(str(feature_path), "a", libver="latest") as fd:
            try:
                if name in fd:
                    del fd[name]
                grp = fd.create_group(name)
                for k, v in pred.items():
                    grp.create_dataset(k, data=v)

            except OSError as error:
                if "No space left on device" in error.args[0]:
                    logger.error(
                        "Out of disk space: storing features on disk can take "
                        "significant space, did you enable the as_half flag?"
                    )
                    del grp, fd[name]
                raise error

        del pred

    logger.info("Finished exporting features.")
    return feature_path