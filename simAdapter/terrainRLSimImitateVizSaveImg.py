
import terrainRLSim
import numpy as np
import json

if __name__ == '__main__':

    env = terrainRLSim.getEnv(env_name="PD_Humanoid_Morph_2D_GRF_Viz3D_TR_48x48_1Sub_Imitate_30FPS_DualState_v1", render=False)
#     env = terrainRLSim.getEnv(env_name="PD_Humanoid_2D_Viz_Imitate_30FPS_2_v0", render=False)
#     env = terrainRLSim.getEnv(env_name="PD_Humanoid_Morph_2D_GRF_Viz3D_48x48_1Sub_Imitate_30FPS_DualState_v1", render=True)
    
    # env.reset()
    actionSpace = env.getActionSpace()
    env.setRandomSeed(1234)
    
    actions = []
    for i in range(1):
        action = ((env.observation_space.high - env.observation_space.low) * np.random.uniform(size=env.observation_space.high.shape[0])  ) + env.observation_space.low
        actions.append(action)            
    
    for e in range(15):
        
        for t in range(2):
            vizData = env.getVisualState()
            vizImitateData = env.getImitationVisualState()
            for vd in range(len(vizData)):
#                 print("viewData: ", vizData)
                viewData = vizData[vd][:-3] ## remove cam velocity
                viewImitateData = vizImitateData[vd][:-3]
                ## Get and vis terrain data
            if (True):
                import matplotlib
                matplotlib.use('Agg')
                import matplotlib.pyplot as plt
                # img_ = viewData
#                     viewData = viewData - viewImitateData
                if (True):
                    if env._config["convert_to_greyscale"]:
                        img_ = np.reshape(viewData, env._config["resize_image"][-2:])
                    else:
                        img_ = np.reshape(viewData, env._config["resize_image"][-2:] + (3,))
#                     img_ = env.render(mode="rgb_array")
#                     noise = np.random.normal(loc=0, scale=0.02, size=img_.shape)
#                     img_ = img_ + noise
                    print("img_ shape", img_.shape, " sum: ", np.sum(viewData))
#                     fig1 = plt.figure(1)
#                     plt.imshow(img_, origin='lower')
#                     plt.title("visual Data: " +  str(vd))
#                     fig1.savefig("char_viz_state_"+str(e)+"_"+str(t)+".svg")

                if (True):                    
                    img__ = viewImitateData
                    if env._config["convert_to_greyscale"]:
                        img__ = np.reshape(viewImitateData, env._config["resize_image"][-2:])
                    else:
                        img__ = np.reshape(viewImitateData, env._config["resize_image"][-2:] + (3,))
                    fig2 = plt.figure(2)
                    img__ = np.concatenate((img_, img__), axis=1)
                    plt.imshow(img__, origin='lower')
                    plt.title("visual Data: " +  str(vd))
                    fig2.savefig("char_viz_imitation_state_"+str(e)+"_"+str(t)+".svg")
                
                if (True):
                    fig3 = plt.figure(2)
                    image = env.render(mode="rgb_array")
                    image = np.reshape(image, (128,128,3))
                    plt.imshow(image, origin='lower')
                    plt.title("visual Data: " +  str(image))
                    fig3.savefig("char_render2_"+str(e)+"_"+str(t)+".svg")
                plt.show()
            observation, reward,  done, info = env.step(actions)
#             imitationState = env.getImitationState()
            # print ("observation.shape: ", observation.shape)
#             print ("imitationState.shape", np.array(imitationState).shape)
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