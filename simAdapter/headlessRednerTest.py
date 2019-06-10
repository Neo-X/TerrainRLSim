
import terrainRLSim
import numpy as np
import json

if __name__ == '__main__':

    import matplotlib.pyplot as plt

    # env = terrainRLSim.getEnv(env_name="PD_Humanoid_2D_Viz_Imitate_30FPS_v0", render=True)
    # env = terrainRLSim.getEnv(env_name="PD_Humanoid_2D_Viz_Imitate_30FPS_2_v0", render=True)
    env = terrainRLSim.getEnv(env_name="PD_Humanoid_2D_Viz3D_Imitate_30FPS_v0", render=True)

    plt.ion()
    _fig, (_bellman_error_ax) = plt.subplots(1, 1, sharey=False, sharex=True)
    img_ = env.getEnv().getPixels(0, 0, 800, 450)
    img_ = np.reshape(img_, (450, 800, 3))
    _bellman_error_ax.imshow(img_, origin='lower')
    _bellman_error_ax.set_title("visual Data: ")
    # self._bellman_error_ax = plt.imshow(img_, origin='lower')
    # plt.title("visual Data: ")
    plt.grid(b=True, which='major', color='black', linestyle='--')
    plt.grid(b=True, which='minor', color='g', linestyle='--')
    _fig.set_size_inches(8.0, 4.5, forward=True)
    plt.show()
    
    # env.reset()
    actionSpace = env.getActionSpace()
    env.setRandomSeed(1234)
    
    actions = []
    for i in range(1):
        action = ((actionSpace.getMaximum() - actionSpace.getMinimum()) * np.random.uniform(size=actionSpace.getMinimum().shape[0])  ) + actionSpace.getMinimum()
        actions.append(action)            
    
    for e in range(10):
        
        for t in range(10):
            
            img_ = env.getEnv().getPixels(0, 0, 800, 450)
            img_ = np.reshape(img_, (450, 800, 3))
            ax_img = _bellman_error_ax.images[-1]
            ax_img.set_data(img_)
            # ax_img = self._bellman_error_ax.set_data(img_)
            # self._bellman_error_ax.canvas.draw()
            _fig.canvas.draw()
            vizData = env.getVisualState()
            
            vizImitateData = env.getImitationVisualState()
            for vd in range(len(vizData)):
                # print("viewData: ", viewData)
                viewData = vizData[vd]
                viewImitateData = vizImitateData[vd]
                ## Get and vis terrain data
                if (False):
                    import matplotlib.pyplot as plt
                    # img_ = np.reshape(viewData, (150,158,3))
                    img_ = viewData
                    noise = np.random.normal(loc=0, scale=0.02, size=img_.shape)
                    img_ = img_ + noise
                    print("img_ shape", img_.shape, " sum: ", np.sum(viewData))
                    plt.figure(1)
                    plt.imshow(img_, origin='lower')
                    plt.title("visual Data: " +  str(vd))

                    if (False):                    
                        img_ = viewImitateData
                        plt.figure(2)
                        plt.imshow(img_, origin='lower')
                        plt.title("visual Data: " +  str(vd))
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
    
    env = terrainRLSim.getEnv(env_name="PD_Biped3D_FULL_Imitate-Steps-v0", render=False)
    env.finish()
    
    print ("Done")