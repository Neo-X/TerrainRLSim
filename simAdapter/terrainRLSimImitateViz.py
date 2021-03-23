
import terrainRLSim
import numpy as np
import json
import cProfile, pstats, io

if __name__ == '__main__':
    
    profileCode = True
    
    if profileCode:
        pr = cProfile.Profile()
        pr.enable()

    # env = terrainRLSim.getEnv(env_name="PD_Humanoid_2D_Viz3D_Imitate_30FPS_v0", render=True)
    # env = terrainRLSim.getEnv(env_name="PD_Humanoid_2D_Viz_Imitate_30FPS_2_v0", render=True)
    # env = terrainRLSim.getEnv(env_name="PD_Humanoid_2D_Viz3D_128x128_Imitate_30FPS_DualState_v0", render=True)
    # env = terrainRLSim.getEnv(env_name="PD_Humanoid_3D_Viz3D_64x64_Imitate_30FPS_v0", render=True)
    # env = terrainRLSim.getEnv(env_name="PD_Humanoid_3D_Viz3D_64x64_Imitate_30FPS_DualState_v0", render=True)
    # env = terrainRLSim.getEnv(env_name="PD_Humanoid_GRF_3D_Walk_Viz3D_WithCamVel_64x64_1Sub_Imitate_30FPS_DualState_v0", render=True)
    env = terrainRLSim.getEnv(env_name="PD_Humanoid_Morph_2D_GRF_Viz3D_48x48_1Sub_Imitate_30FPS_DualState_v1", render=True)
    
    # env.reset()
    actionSpace = env.getActionSpace()
    env.setRandomSeed(1234)
    
    actions = []
    for i in range(1):
        action = ((actionSpace.high - actionSpace.low) * np.random.uniform(size=actionSpace.low.shape[0])  ) + actionSpace.low
        actions.append(action)            
    
    for e in range(10):
        
        for t in range(10):
            vizData = env.getVisualState()
            vizImitateData = env.getImitationVisualState()
            for vd in range(len(vizData)):
                # print("viewData: ", viewData)
                viewData = vizData[vd]
                viewImitateData = vizImitateData[vd]
                ## Get and vis terrain data
                if (False):
                    import matplotlib
                    # matplotlib.use('Agg')
                    import matplotlib.pyplot as plt
                    img_ = viewData[3:]
                    img_ = np.reshape(img_, (64,64,3))
                    noise = np.random.normal(loc=0, scale=0.02, size=img_.shape)
                    # img_ = img_ + noise
                    print("img_ shape", img_.shape, " sum: ", np.sum(img_))
                    fig1 = plt.figure(1)
                    plt.imshow(img_, origin='lower')
                    plt.title("visual Data: " +  str(vd))
                    fig1.savefig("viz_state_"+str(i)+".svg")

                    if (False):                    
                        img_ = viewImitateData
                        fig2 = plt.figure(2)
                        plt.imshow(img_, origin='lower')
                        plt.title("visual Data: " +  str(vd))
                        fig2.savefig("viz_imitation_state_"+str(i)+".svg")
                    plt.show()
            observation, reward,  done, info = env.step(actions)
            imitationState = env.getImitationState()
            print ("observation.shape: ", np.array(observation[0][0]).shape)
            print ("imitationState.shape", np.array(imitationState).shape)
            print ("Done: ", done)
            print ("motion id: ", env.getTaskID())
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
    if profileCode:
        pr.disable()
        f = open('x.prof', 'a')
        pstats.Stats(pr, stream=f).sort_stats('time').print_stats()
        f.close()
        
    print ("Done")