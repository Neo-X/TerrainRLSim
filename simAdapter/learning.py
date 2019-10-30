import terrainRLSim
import numpy as np
import matplotlib.pyplot as plt
import json
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.distributions import Categorical


learning_rate = 0.0002
gamma         = 0.98

class Policy(nn.Module):
    def __init__(self):
        super(Policy, self).__init__()
        self.data = []
        
        self.fc1 = nn.Linear(329, 128)
        self.fc2 = nn.Linear(128, 5)
        self.optimizer = optim.Adam(self.parameters(), lr=learning_rate)
        
    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.softmax(self.fc2(x), dim=0)
        return x
      
    def put_data(self, item):
        self.data.append(item)
        
    def train_net(self):
        R = 0
        for r, log_prob in self.data[::-1]:
            R = r + gamma * R
            loss = -log_prob * R
            self.optimizer.zero_grad()
            loss.backward()
            self.optimizer.step()
        self.data = []


if __name__ == '__main__':
    envs_list = terrainRLSim.getEnvsList()
    print("# of envs ", len(envs_list))
    print("Envs:\n ", json.dumps(envs_list, sort_keys=True, indent=4))
    env = terrainRLSim.getEnv(env_name="PD_Dog2D_Steps_Terrain-v0", render=True)
    pi = Policy()

    env.reset()
    actionSpace = env.getActionSpace()
    env.setRandomSeed(1234)

    actions=[]
    action = ((actionSpace.getMaximum() - actionSpace.getMinimum()) * np.random.uniform(size=actionSpace.getMinimum().shape[0])  ) + actionSpace.getMinimum()
    print("action", action)
    actions.append(action)

    print("Actions: ", actions)    
    print("observation_space: ", env.observation_space.getMaximum())    
    print("Actions space max: ", len(env.action_space.getMaximum()))
    print("Actions space min: ", env.action_space.getMinimum())
    print("Actions space max: ", env.action_space.getMaximum())


    for e in range(1):
        s =env.reset()

        for t in range(1):
            prob = pi(torch.from_numpy(s).float())
            print(prob)
            observation, reward, done, info = env.step(actions)
            action = ((actionSpace.getMaximum() - actionSpace.getMinimum()) * np.random.uniform(size=actionSpace.getMinimum().shape[0])  ) + actionSpace.getMinimum()
            print("action", action)
            # actions.append(action)

            #print("Done: ", done)
            if(done):
                break


            #print("Agent State: ", len(observation))



env.finish()
print(env)