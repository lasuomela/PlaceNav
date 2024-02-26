'''
Hloc-style interface for loading the CosPlace model.

Borrowed from Hloc (https://github.com/cvg/Hierarchical-Localization/)
by Paul-Edouard Sarlin et al
'''

import torch
import torchvision.transforms as tvf

from .base_model import BaseModel
from .cosplace_model.cosplace_network import GeoLocalizationNet


class CosPlace(BaseModel):
    default_conf = {
        'backbone': 'efficientnet_b0',
        'fc_output_dim' : 512,
    }
    required_inputs = ['image']
    def _init(self, conf):

        self.net = GeoLocalizationNet(conf['backbone'], conf['fc_output_dim'])
        model_state_dict = torch.load( conf['checkpoint_path'])
        self.net.load_state_dict(model_state_dict)
        self.net = self.net.eval()

    def _forward(self, data):
        image = data['image']
        desc = self.net(image)
        return {
            'global_descriptor': desc,
        }