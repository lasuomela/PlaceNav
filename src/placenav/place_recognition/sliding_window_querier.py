import numpy as np

from .gallery_db import PlaceRecognitionDBHandler

class PlaceRecognitionSlidingWindowFilter:
    '''
    '''
    def __init__(
        self, extractor, gallery_db_path, img_dir_path
    ):
        # Load map data
        self.extractor = extractor
        self.db_handler = PlaceRecognitionDBHandler(gallery_db_path, img_dir_path)
        self.descriptors = self.db_handler.get_descriptors()

    def match(self, img, window_lower, window_upper):
        query_desc = self.extractor(img)['global_descriptor'].numpy().squeeze()

        diff = self.descriptors - query_desc
        dists = np.linalg.norm(diff, axis=1)

        # Get the index of the most similar descriptor
        # within the specified window
        sg_idx = np.argmin(dists[window_lower:window_upper])
        sg_idx += window_lower
        return sg_idx