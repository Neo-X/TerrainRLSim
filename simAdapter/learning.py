import os
import argparse
import math

import terrainRLSim
import numpy as np
import matplotlib.pyplot as plt
import json
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.distributions import Categorical

from model import Actor, Critic
from utils.utils import get_action, save_checkpoint
from collections import deque
from utils.running_state import ZFilter
from hparams import HyperParams as hp
from tensorboardX import SummaryWriter


def log_density(x, mu, std, logstd):
    var = std.pow(2)
    log_density = -(x - mu).pow(2) / (2 * var) \
                  - 0.5 * math.log(2 * math.pi) - logstd
    return log_density.sum(1, keepdim=True)



def get_action(mu, std):
    action = torch.normal(mu, std)
    action = action.data.numpy()
    return action

def get_loss(policy1, returns, states, actions):
    mu, std, logstd = policy1(torch.Tensor(states))
    log_policy = log_density(torch.Tensor(actions), mu, std, logstd)
#    returns = returns.unsqueeze(1)

    objective = returns * log_policy
    objective = objective.mean()
    return - objective

learning_rate = 0.0002
gamma         = 0.98

class Policy(nn.Module):
    def __init__(self):
        super(Policy, self).__init__()
        self.data = []
        
        self.fc1 = nn.Linear(329, 128)
        self.fc2 = nn.Linear(128, 128)
        self.fc3 = nn.Linear(128, 20)
        self.optimizer = optim.Adam(self.parameters(), lr=learning_rate)
        
    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = self.fc2(x)
        mu = self.fc3(x)
        logstd = torch.zeros_like(mu)
        std = torch.exp(logstd)
        #x = F.softmax(self.fc2(x), dim=0)
        return mu, std, logstd
      
    def put_data(self, item):
        self.data.append(item)
        
    def train_net(self):
        R = 0
        for policy1, returns, states ,actions in self.data[::-1]:
            loss = get_loss(policy1, returns, states ,actions)
            self.optimizer.zero_grad()
            loss.backward()
            self.optimizer.step()
        self.data = []




if __name__ == '__main__':
    envs_list = terrainRLSim.getEnvsList()
#    print("# of envs ", len(envs_list))
#    print("Envs:\n ", json.dumps(envs_list, sort_keys=True, indent=4))
    env = terrainRLSim.getEnv(env_name="PD_Humanoid_GRF_3D_Punch_Viz3D_WithCamVel_128x128_1Sub_Imitate_30FPS_DualState_v0", render=True)
    pi = Policy()

    env.reset()
    actionSpace = env.getActionSpace()
    env.setRandomSeed(1234)

    actions=[]
    action = ((actionSpace.getMaximum() - actionSpace.getMinimum()) * np.random.uniform(size=actionSpace.getMinimum().shape[0])  ) + actionSpace.getMinimum()
    #action = actionSpace.getMaximum()
    print("action", action)
    actions.append(action)

    print("Actions: ", actions)    
    print("observation_space: ", env.observation_space.getMaximum())    
    print("Actions space max: ", len(env.action_space.getMaximum()))
    print("Actions space min: ", env.action_space.getMinimum())
    print("Actions space max: ", env.action_space.getMaximum())
    score = 0.0
    print_interval = 20

    for e in range(100):
        s =env.reset()

        for t in range(1000):
#            prob = pi(torch.from_numpy(s).float())
#            print(torch.Tensor(s).shape)
#            print(torch.Tensor(s).unsqueeze(0).shape)
#            mu,std, _ = pi(torch.Tensor(s).unsqueeze(0))
#            action = get_action(mu, std)[0]
            observation, reward, done, info = env.step(action)
#            print(action)
            s = observation
            score += reward[0][0]
            pi.put_data((pi, reward[0][0], s, action))
            #action = torch.normal(mu, std)

            #action = ((actionSpace.getMaximum() - actionSpace.getMinimum()) * np.random.uniform(size=actionSpace.getMinimum().shape[0])  ) + actionSpace.getMinimum()
            #print("action", observation)
            # actions.append(action)
            
            #print("Done: ", done)
            if(done):
                break
        pi.train_net()
        if e%print_interval==0 and e!=0:    
            print("# of episode :{}, avg score : {}".format(e, score/print_interval))
            score=0.0
            #print("Agent State: ", len(observation))



env.finish()
print(env)