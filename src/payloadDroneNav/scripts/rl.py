#######################################################
# custom MLP feature extractor  with stable baselines #
#######################################################
import gym
import torch as th
import torch.nn as nn
import stable_baselines3 as sb
from   stable_baselines3 import PPO
from   stable_baselines3.common.torch_layers import BaseFeatureExtractor


#################################################################################################
# Inherit baseFeature extractor to create a custom CNN
# Use inputs -> Linear1(size equivalent to number of inputs) -> Relu -> Linear(any size) -> Relu
#################################################################################################
class CustomFeatureExtractor(BaseFeatureExtractor):
    def __init__(self, observation_space: gym.spaces.Box, features_dim: int = 256)
        super(CustomFeatureExtractor, self).__init__(observation_space, features_dim)
        
        self.privInfoSize  = 1
        self.stateInfoSize = 9
        self.input_size    = self.privInfoSize + self.stateInfoSize
        self.hidden1_size  = 20
        self.output_size   = features_dim

        self.fc1           = nn.Linear(self.input_size, self.hidden1_size)
        self.fc2           = nn.Linear(self.hidden1_size, self.output_size)

        self.relu          = nn.ReLU()
        
    def forward(self, x):
        return nn.Flatten(self.relu(self.fc2(self.relu(self.fc1(x)))))








