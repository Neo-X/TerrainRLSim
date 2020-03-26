
import terrainRLSim
import numpy as np
import json

if __name__ == '__main__':

    # env = terrainRLSim.getEnv(env_name="PD_Humanoid_2D_Viz_Imitate_30FPS_v0", render=True)
    # env = terrainRLSim.getEnv(env_name="PD_Humanoid_2D_Viz_Imitate_30FPS_2_v0", render=True)
    env = terrainRLSim.getEnv(env_name="PD_Humanoid_2D_GRF_Viz3D_16x16_1Sub_Imitate_30FPS_DualState_v1", render=True)
    
    # env.reset()
    actionSpace = env.getActionSpace()
    env.setRandomSeed(1234)
    
    actions = []
    for i in range(1):
        action = ((env.observation_space.high - env.observation_space.low) * np.random.uniform(size=env.observation_space.high.shape[0])  ) + env.observation_space.low
        actions.append(action)            
    
    for e in range(5):
        
        for t in range(10):
            vizData = env.getVisualState()
            vizImitateData = env.getImitationVisualState()
            for vd in range(len(vizData)):
                # print("viewData: ", viewData)
                viewData = vizData[vd]
                viewImitateData = vizImitateData[vd]
                ## Get and vis terrain data
                if (True):
                    import matplotlib
                    matplotlib.use('Agg')
                    import matplotlib.pyplot as plt
                    # img_ = viewData
                    img_ = np.reshape(viewData, (16,16))
                    noise = np.random.normal(loc=0, scale=0.02, size=img_.shape)
                    img_ = img_ + noise
                    print("img_ shape", img_.shape, " sum: ", np.sum(viewData))
                    fig1 = plt.figure(1)
                    plt.imshow(img_, origin='lower')
                    plt.title("visual Data: " +  str(vd))
                    fig1.savefig("char_viz_state_"+str(e)+"_"+str(t)+".svg")

                    if (True):                    
                        img_ = viewImitateData
                        img_ = np.reshape(viewImitateData, (16, 16))
                        fig2 = plt.figure(2)
                        plt.imshow(img_, origin='lower')
                        plt.title("visual Data: " +  str(vd))
                        fig2.savefig("char_viz_imitation_state_"+str(e)+"_"+str(t)+".svg")
                    plt.show()
            observation, reward,  done, info = env.step(actions)
            imitationState = env.getImitationState()
            # print ("observation.shape: ", observation.shape)
            print ("imitationState.shape", np.array(imitationState).shape)
            print ("Done: ", done)
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
            print("Reward: ", reward)
            # print("action: ", actions)
                
            # states = np.array(observation)
            # print("states shape ", states[0].shape)
            # print ("std length: ", len(np.std(states, axis=0)) )
            # print ("std for states: ", np.std(states, axis=0))
            #### LLC states. If there is an LLC
            # llc_state = env.getLLCState()
            # print ("LLC state:", llc_state.shape)
            
            
            # print ("Agent state: ", state)
            # if (done):
            #     env.reset()
        env.reset()
        
    env.finish()
    print (env)
    
    print ("Done")