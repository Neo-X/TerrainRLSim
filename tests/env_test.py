import pytest
from numpy.testing import assert_allclose
import numpy as np

import warnings

import numpy as np
import matplotlib.pyplot as plt
import json
import os
import sys

class TestEnvs(object):
            
    def test_load_env(self):
        
        # terrainRL_PATH = os.environ['TERRAINRL_PATH']
        # sys.path.append(terrainRL_PATH+'/lib')
        from simAdapter import terrainRLSim
        envs_list = terrainRLSim.getEnvsList()
        # print ("# of envs: ", len(envs_list))
        # print ("Envs:\n", json.dumps(envs_list, sort_keys=True, indent=4))
        env = terrainRLSim.getEnv(env_name="PD_Biped3D_FULL_Imitate-Steps-v0", render=False)
        
        env.reset()
        actionSpace = env.getActionSpace()
        env.setRandomSeed(1234)
        env.finish()
        
    def test_multichar_terrain(self):
        
        # terrainRL_PATH = os.environ['TERRAINRL_PATH']
        # sys.path.append(terrainRL_PATH+'/lib')
        from simAdapter import terrainRLSim
        envs_list = terrainRLSim.getEnvsList()
        # print ("# of envs: ", len(envs_list))
        # print ("Envs:\n", json.dumps(envs_list, sort_keys=True, indent=4))
        env = terrainRLSim.getEnv(env_name="PD_Biped3D_MutliChar_WithVel_LargeBlocks-v0", render=False)
        
        env.reset()
        actionSpace = env.getActionSpace()
        env.setRandomSeed(1234)
        actions = []
        for i in range(11):
            action = ((actionSpace.getMaximum() - actionSpace.getMinimum()) * np.random.uniform(size=actionSpace.getMinimum().shape[0])  ) + actionSpace.getMinimum()
            actions.append(action)
            
        observation, reward,  done, info = env.step(actions)
        
        states = np.array(observation)
        img_data_size=1024
        agent_num = 1
        data_ = []
        for i in range(10):
            data_.append(states[i + 1][img_data_size*2:img_data_size*3])

        ### There is some non-zero data
        assert np.std(data_) > 0.01
        plt.show()
        env.finish()
    
    def test_multichar_velocityfield_x(self):
        
        # terrainRL_PATH = os.environ['TERRAINRL_PATH']
        # sys.path.append(terrainRL_PATH+'/lib')
        from simAdapter import terrainRLSim
        envs_list = terrainRLSim.getEnvsList()
        # print ("# of envs: ", len(envs_list))
        # print ("Envs:\n", json.dumps(envs_list, sort_keys=True, indent=4))
        env = terrainRLSim.getEnv(env_name="PD_Biped3D_MutliChar_WithVel_LargeBlocks-v0", render=False)
        
        env.reset()
        actionSpace = env.getActionSpace()
        env.setRandomSeed(1234)
        actions = []
        for i in range(11):
            action = ((actionSpace.getMaximum() - actionSpace.getMinimum()) * np.random.uniform(size=actionSpace.getMinimum().shape[0])  ) + actionSpace.getMinimum()
            actions.append(action)
            
        observation, reward,  done, info = env.step(actions)
        
        states = np.array(observation)
        img_data_size=1024
        agent_num = 1
        data_ = []
        for i in range(10):
            data_.append(states[i + 1][0:img_data_size])

        ### There is some non-zero data
        assert np.std(data_) > 0.01
        plt.show()
        env.finish()
        
    def test_multichar_velocityfield_y(self):
        
        # terrainRL_PATH = os.environ['TERRAINRL_PATH']
        # sys.path.append(terrainRL_PATH+'/lib')
        from simAdapter import terrainRLSim
        envs_list = terrainRLSim.getEnvsList()
        # print ("# of envs: ", len(envs_list))
        # print ("Envs:\n", json.dumps(envs_list, sort_keys=True, indent=4))
        env = terrainRLSim.getEnv(env_name="PD_Biped3D_MutliChar_WithVel_LargeBlocks-v0", render=False)
        
        env.reset()
        actionSpace = env.getActionSpace()
        env.setRandomSeed(1234)
        actions = []
        for i in range(11):
            action = ((actionSpace.getMaximum() - actionSpace.getMinimum()) * np.random.uniform(size=actionSpace.getMinimum().shape[0])  ) + actionSpace.getMinimum()
            actions.append(action)
            
        observation, reward,  done, info = env.step(actions)
        print("observation, observation")
        states = np.array(observation)
        img_data_size=1024
        agent_num = 1
        data_ = []
        for i in range(10):
            data_.append(states[i + 1][img_data_size:img_data_size*2])

        ### There is some non-zero data
        assert np.std(data_) > 0.01
        plt.show()
        env.finish()
    
       
    def test_HLC_velocityfield_x(self):
        print ("Testing velocity field for HLC")
        # terrainRL_PATH = os.environ['TERRAINRL_PATH']
        # sys.path.append(terrainRL_PATH+'/lib')
        from simAdapter import terrainRLSim
        envs_list = terrainRLSim.getEnvsList()
        # print ("# of envs: ", len(envs_list))
        # print ("Envs:\n", json.dumps(envs_list, sort_keys=True, indent=4))
        env = terrainRLSim.getEnv(env_name="PD_Biped3D_HLC_DynamicsObstacles-v0", render=False)
        
        env.reset()
        actionSpace = env.getActionSpace()
        env.setRandomSeed(1234)
        actions = []
        num_agents = 1
        for i in range(num_agents):
            action = ((actionSpace.getMaximum() - actionSpace.getMinimum()) * np.random.uniform(size=actionSpace.getMinimum().shape[0])  ) + actionSpace.getMinimum()
            actions.append(action)
            
        observation, reward,  done, info = env.step(actions)
        
        states = np.array(observation)
        img_data_size=1024
        data_ = []
        for i in range(10):
            data_.append(states[0][0:img_data_size])

        ### There is some non-zero data
        assert np.std(data_) > 0.01
        plt.show()
        env.finish()
    
    def test_HLC_velocityfield_y(self):
        
        # terrainRL_PATH = os.environ['TERRAINRL_PATH']
        # sys.path.append(terrainRL_PATH+'/lib')
        from simAdapter import terrainRLSim
        envs_list = terrainRLSim.getEnvsList()
        # print ("# of envs: ", len(envs_list))
        # print ("Envs:\n", json.dumps(envs_list, sort_keys=True, indent=4))
        env = terrainRLSim.getEnv(env_name="PD_Biped3D_HLC_DynamicsObstacles-v0", render=False)
        
        env.reset()
        actionSpace = env.getActionSpace()
        env.setRandomSeed(1234)
        actions = []
        num_agents = 1
        for i in range(num_agents):
            action = ((actionSpace.getMaximum() - actionSpace.getMinimum()) * np.random.uniform(size=actionSpace.getMinimum().shape[0])  ) + actionSpace.getMinimum()
            actions.append(action)
            
        observation, reward,  done, info = env.step(actions)
        
        states = np.array(observation)
        img_data_size=1024
        data_ = []
        for i in range(10):
            data_.append(states[0][img_data_size:img_data_size*2])

        ### There is some non-zero data
        assert np.std(data_) > 0.01
        plt.show()
        env.finish()
    
        
if __name__ == '__main__':
    pytest.main([__file__])