

import terrainRLSim
import numpy as np
import json

if __name__ == '__main__':

    # env = getEnv(env_name="FSM_Biped2D_Terrain_Walls-v0", render=True)
    # env = terrainRLSim.getEnv(env_name="PD_Biped3D_HLC_DynamicsObstacles-v0", render=False)
    # env = terrainRLSim.getEnv(env_name="PD_Biped3D_MutliChar_Crowd_WithVel_LargeBlocks-v0", render=True)
    # env = terrainRLSim.getEnv(env_name="PD_Biped3D_MutliChar_DynamicObstacles-v0", render=True)
    # env = terrainRLSim.getEnv(env_name="PD_Biped3D_Imitate_NoPhase_AxisAngle_v0", render=True)
    # env = terrainRLSim.getEnv(env_name="PD_Biped3D_HLC_DynamicsObstacles-v1", render=True)
    # env = terrainRLSim.getEnv(env_name="PD_Biped3D_Imitate-v0", render=True)
    # env = terrainRLSim.getEnv(env_name="PD_Humanoid_2D_Imitate_60FPS_Torque_v0", render=True)
    # env = terrainRLSim.getEnv(env_name="PD_Humanoid_2D_Viz3D_FixedStart_64x64_1Sub_Imitate_30FPS_MultiModal_DualState_v0", render=True)
    # env = terrainRLSim.getEnv(env_name="PD_Humanoid1_3D_Run_Phase_v0", render=True)
    env = terrainRLSim.getEnv(env_name="PD_Biped3D_MutliChar_Humanoid_ScenarioSpace_Soccer_4_WithObstacles_AndOtherAgentPos_OnlyVel_SimpleReward_v1", render=True)

    envs_list = terrainRLSim.getEnvsList()
    print ("# of envs: ", len(envs_list))
    # print ("Envs:\n", json.dumps(envs_list, sort_keys=True, indent=4))
    # env = terrainRLSim.getEnv(env_name="PD_Biped3D_FULL_Imitate-Steps-v0", render=True)
    
    env.reset()
    actionSpace = env.getActionSpace()
    env.setRandomSeed(1234)
    
    actions = []
    for i in range(11):
        action = actionSpace.sample()
        actions.append(action * 0)
    # actions = np.array(actions) * 100            
    print("Actions: ", actions)
    
    print("observation_space: ", env.observation_space.high)
    
    print("Actions space max: ", len(env.action_space.high))
    print("Actions space min: ", env.action_space.low)
    print("Actions space max: ", env.action_space.high)
    
    for e in range(100):
        env.reset()
        for t in range(256):
            observation, reward,  done, info = env.step(actions)
            # env.getImitationState()
            print ("Done: ", done)
            print("Reward: ", reward)
            states = np.array(observation)
            print("states shape ", np.array(states[0]).shape)
            if ( done ):
                break
            """
            states = []
            for i in range(sim.getNumAgents()):
                ### get all states and check that they are different
                state = np.array(sim.getStateForAgent(i))
                print ("Agent: ", i, " state: ", state.shape)
                states.append(state)
                
                sim.updateActionForAgent(i, actions[i])
                """
            # print("Observation: ", observation)
            # print("action: ", actions)
                
            # print ("state: ", states)
            # print ("std length: ", len(np.std(states, axis=0)) )
            # print ("std for states: ", np.std(states))
            #### LLC states. If there is an LLC
            # llc_state = env.getLLCState()
            # print ("LLC state:", llc_state.shape)
            
            ## Get and vis terrain data
            if (False):
                import matplotlib.pyplot as plt
                img_data_size=1024
                agent_num = 0
                img_ = np.reshape(states[agent_num][0:img_data_size], (32,32))
                print("img_ shape", img_.shape)
                if (True):
                    plt.imshow(img_)
                    plt.title("velocity field x: min, " +  str(np.min(img_))  + " max, " +  str(np.max(img_)) )
                    plt.show()
                ## get and vis velocity data
                ## x
                if (True):
                    img_ = np.reshape(states[agent_num][img_data_size:img_data_size*2], (32,32))
                    plt.imshow(img_)
                    plt.title("velocity field y: min, " +  str(np.min(img_))  + " max, " +  str(np.max(img_)))
                    plt.show()
                ## y
                if (False):
                    img_ = np.reshape(states[agent_num][img_data_size*2:img_data_size*3], (32,32))
                    plt.imshow(img_)
                    plt.title("height field: min, " +  str(np.min(img_))  + " max, " +  str(np.max(img_)))
                    plt.show()
            
            # print ("Agent state: ", state)
            # if (done):
            #     env.reset()
        
    env.finish()
    print (env)
    
    env = terrainRLSim.getEnv(env_name="PD_Biped3D_FULL_Imitate-Steps-v0", render=False)
    env.finish()
    
    print ("Done")