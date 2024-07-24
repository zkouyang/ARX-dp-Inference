import h5py
import matplotlib.pyplot as plt
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import torchvision.transforms as transforms
import torchvision.models as models
import os

class StageObsPolicy(nn.Module):
    def __init__(self, num_classes):
        super(StageObsPolicy, self).__init__()
        self.resnet = models.resnet18(pretrained=True)
        self.resnet.fc = nn.Identity() 
        self.gru = nn.GRU(512, 256, batch_first=True) 
        self.fc = nn.Linear(256, num_classes)  

    def forward(self, images, hidden=None):
        batch_size, seq_len, C, H, W = images.size()
        images = images.view(batch_size * seq_len, C, H, W)
        resnet_features = self.resnet(images)
        resnet_features = resnet_features.view(batch_size, seq_len, -1)
        gru_out, hidden = self.gru(resnet_features, hidden)
        outputs = self.fc(gru_out)
        return outputs, hidden
