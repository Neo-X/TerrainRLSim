
import terrainRLSim
import numpy as np
import json
import gc
import time

if __name__ == '__main__':

    envs_list = terrainRLSim.getEnvsList()
    print ("# of envs: ", len(envs_list))
    print ("Envs:\n", json.dumps(envs_list, sort_keys=True, indent=4))
    # env = terrainRLSim.getEnv(env_name="PD_Biped3D_FULL_Imitate-Steps-v0", render=True)
    
    print ("envs_list.keys(): ", envs_list.keys())
    for environment in envs_list.keys():
        print("Trying environemnt: ", environment)
        if not isinstance(envs_list[environment], dict):
            continue
            
        print("creating environment: ", environment)
        
        env = terrainRLSim.getEnv(env_name=environment, render=True)
        env.reset()
        actionSpace = env.getActionSpace()
        env.setRandomSeed(1234)
        
        actions = []
        for i in range(11):
            action = ((actionSpace.getMaximum() - actionSpace.getMinimum()) * np.random.uniform(size=actionSpace.getMinimum().shape[0])  ) + actionSpace.getMinimum()
            actions.append(action)
            
        for e in range(1):
            env.reset()
            
            for t in range(100):
                observation, reward,  done, info = env.step(actions)
                # print ("Done: ", done)
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
                # print("Reward: ", reward)
                # print("action: ", actions)
                    
                states = np.array(observation)
                print("states shape ", states[0].shape)
                # print ("std length: ", len(np.std(states, axis=0)) )
                # print ("std for states: ", np.std(states, axis=0))
                #### LLC states. If there is an LLC
                # llc_state = env.getLLCState()
                # print ("LLC state:", llc_state.shape)
                
                # print ("Agent state: ", state)
                # if (done):
                #     env.reset()
            
        env.finish()
        print (env)
        gc.collect()
        ## Sleep for a second
        time.sleep(1)
    
    print ("Done")