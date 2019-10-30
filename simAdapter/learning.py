
import terrainRLSim
import numpy as np
import matplotlib.pyplot as plt
import json


if __name__ == '__main__':
    envs_list = terrainRLSim.getEnvsList()
    print("# of envs ", len(envs_list))
    print("Envs:\n ", json.dumps(envs_list, sort_keys=True, indent=4))
    env = terrainRLSim.getEnv(env_name="PD_Dog2D_LLC_Imitate_Flat-v0", render=True)

    env.reset()
    actionSpace = env.getActionSpace()
    env.setRandomSeed(1234)

    actions=[]
    action = ((actionSpace.getMaximum() - actionSpace.getMinimum()) * np.random.uniform(size=actionSpace.getMinimum().shape[0])  ) + actionSpace.getMinimum()
    actions.append(action)


    print("observation_space: ", env.observation_space.getMaximum())


    for e in range(10):
        env.reset()

        for t in range(100):
            observation, reward, done, info = env.step(actions)
            #print("Done: ", done)
            if(done):
                break


            #print("Agent State: ", len(observation))



env.finish()
print(env)