import h5py
import numpy as np
from pathlib import Path

class PlaceRecognitionDBHandler:

    def __init__(self, db_path, img_dir_path):

        # Load the gallery db
        # Expects that the topological map consists of
        # a non-cyclical non-branching graph of images,
        # where each image's filename is an integer
        # that specifies the order in which the images were taken.
        
        self.gallery_db = h5py.File(db_path, 'r')
        self.gallery_len = len(self.gallery_db)

        img_paths = []
        descriptors = []

        img_dir_path = Path(img_dir_path)
        img_names = self.gallery_db.keys()
        for img_name in img_names:
            img_attrs = self.gallery_db[img_name]
            img_descriptor = img_attrs['global_descriptor']

            img_paths.append(img_dir_path / img_name)
            descriptors.append(img_descriptor)

        # Get the image filenames without the extension and check that they are integers
        try:
            img_idxs = [int(img_path.stem) for img_path in img_paths]
        except ValueError:
            raise ValueError("Image filenames must be integers, got {}".format(img_paths))

        # Sort the image paths and descriptors according to the image filename
        self.img_paths = np.array([x for _, x in sorted(zip(img_idxs, img_paths), key=lambda pair: pair[0])])
        self.descriptors = np.array([x for _, x in sorted(zip(img_idxs, descriptors), key=lambda pair: pair[0])])

    def get_descriptors(self):
        return self.descriptors

    def get_filepaths(self):
        return self.img_paths

    def get_by_filename(self, filename):
        idx = np.argwhere(self.img_paths == filename).flatten()[0]
        return self.descriptors[idx], self.odometries[idx]

    def get_by_idx(self, idx):
        return self.img_paths[idx], self.descriptors[idx], self.odometries[idx]

