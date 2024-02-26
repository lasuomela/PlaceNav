import torch
from typing import Dict

from placenav import logger
from placenav.models import pr_models as models
from placenav.models.pr_models.base_model import dynamic_load

class FeatureExtractor:

    def __init__(self, conf, device = 'cuda'):

        self.device = device
        logger.info('Feature extractor: Using device {}'.format(self.device))
        Model = dynamic_load(models, conf['model']['name'])
        self.model = Model(conf['model'])
        if hasattr(self.model, 'eval'):
            self.model.eval()
        self.model = self.model.to(self.device)

    @torch.no_grad()
    def __call__(self, img: torch.Tensor) -> Dict:
        '''
        Extracts the global descriptor from the input image

        Args:
            img (torch.Tensor): Preprocessed input image

        Returns:
            pred (Dict): Dictionary containing the global descriptor
        '''
        size = img.shape[:2][::-1]
        data_dict = {'image': img, 'original_size': size}
        
        pred = self.model(data_dict)
        pred['image_size'] = data_dict['original_size']

        if 'global_descriptor' in pred:
            if pred['global_descriptor'].is_cuda:
                pred['global_descriptor'] = pred['global_descriptor'].cpu()

        return pred