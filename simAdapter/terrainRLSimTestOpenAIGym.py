

import terrainRLSim
import numpy as np
import json
import gym
import time

if __name__ == '__main__':

    env = gym.make("PD_Biped2D_MultiTask_Terrain-v0")

    envs_list = terrainRLSim.getEnvsList()
    print ("# of envs: ", len(envs_list))
    # print ("Envs:\n", json.dumps(envs_list, sort_keys=True, indent=4))
    # env = terrainRLSim.getEnv(env_name="PD_Biped3D_FULL_Imitate-Steps-v0", render=True)
    
    env.reset()
    actionSpace = env.action_space
    env.seed(1234)
    
    actions = []
    for i in range(1):
        action = actionSpace.sample()
        actions.append(action)
    # actions = np.array(actions) * 100            
    print("Actions: ", actions)
    
    print("observation_space: ", env.observation_space.high)
    
    print("Actions space max: ", len(env.action_space.high))
    print("Actions space min: ", env.action_space.low)
    print("Actions space max: ", env.action_space.high)
    
    
    for e in range(50):
        numTasks = env.getNumTasks()
        print ("Number of tasks: ", numTasks)
        task_id = np.random.randint(0,numTasks)
        env.set_task(0)
        print ("task_id: ", task_id)
        env.reset()
        for t in range(100):
            action = env.action_space.sample()
            observation, reward,  done, info = env.step(action)
            # env.getImitationState()
#             time.sleep(1)
            if (False):
                    img_ = env.render()
                    print ("img_: ", np.array(img_).shape)
                    import matplotlib
                    matplotlib.use('Agg')
                    import matplotlib.pyplot as plt
                    # img_ = viewData
#                     img_ = np.reshape(img_, (16,16))
#                     img_ = img_ + noise
#                     print("img_ shape", img_.shape, " sum: ", np.sum(viewData))
                    fig1 = plt.figure(1)
                    plt.imshow(img_, origin='lower')
                    plt.title("visual Data: ")
                    fig1.savefig("render_view_"+str(e)+"_"+str(t)+".png")
            print("Reward: ", reward)
#             states = np.array(observation)
#             print("states shape ", np.array(states).shape)
            print ("info: ", info)
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