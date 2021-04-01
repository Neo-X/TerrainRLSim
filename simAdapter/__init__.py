
import gym
import numpy as np

class Box(gym.spaces.Box):
    """
    A box in R^n.
    I.e., each coordinate is bounded.
    """

    def __init__(self, low, high, shape=None):
        """
        Two kinds of valid input:
            Box(-1.0, 1.0, (3,4)) # low and high are scalars, and shape is provided
            Box(np.array([-1.0,-2.0]), np.array([2.0,4.0])) # low and high are arrays of the same shape
        """
        if shape is None:
            assert low.shape == high.shape
            self.low = low
            self.high = high
        else:
            assert np.isscalar(low) and np.isscalar(high)
            self.low = low + np.zeros(shape)
            self.high = high + np.zeros(shape)

    def sample(self):
        return np.random.uniform(low=self.low, high=self.high, size=self.low.shape)

    def contains(self, x):
        return x.shape == self.shape and (x >= self.low).all() and (x <= self.high).all()

    @property
    def shape(self):
        return self.low.shape

    @property
    def flat_dim(self):
        return np.prod(self.low.shape)

    @property
    def bounds(self):
        return self.low, self.high

    def flatten(self, x):
        return np.asarray(x).flatten()

    def unflatten(self, x):
        return np.asarray(x).reshape(self.shape)

    def flatten_n(self, xs):
        xs = np.asarray(xs)
        return xs.reshape((xs.shape[0], -1))

    def unflatten_n(self, xs):
        xs = np.asarray(xs)
        return xs.reshape((xs.shape[0],) + self.shape)

    def __repr__(self):
        return "Box" + str(self.shape)

    def __eq__(self, other):
        return isinstance(other, Box) and np.allclose(self.low, other.low) and \
               np.allclose(self.high, other.high)

    def __hash__(self):
        return hash((self.low, self.high))

    def new_tensor_variable(self, name, extra_dims):
        return ext.new_tensor(
            name=name,
            ndim=extra_dims+1,
            dtype=theano.config.floatX
        )

def getEnvsList():
    import os, sys, json
    
    terrainRL_PATH = os.environ['TERRAINRL_PATH']
    print ("terrainRL_PATH: ", terrainRL_PATH)
    sys.path.append(terrainRL_PATH+'/lib')
    from simAdapter import terrainRLAdapter
    
    file = open(terrainRL_PATH+"/args/envs.json")
    env_data = json.load(file)
    file.close()
    
    return env_data

from gym.envs.registration import register as gym_register
# Use the gym_register because it allows us to set the max_episode_steps.
env_data = getEnvsList()

gym_register(
    id='PD_Biped2D_Flat-v0',
    entry_point='simAdapter.terrainRLSim:TerrainRLSimWrapper',
    reward_threshold=0.95,
    max_episode_steps=512,
    kwargs={'config': env_data["PD_Biped2D_Flat-v0"],
            "render": True}
)

gym_register(
    id='PD_Biped2D_Gaps_Terrain-v0',
    entry_point='simAdapter.terrainRLSim:TerrainRLSimWrapper',
    reward_threshold=0.95,
    max_episode_steps=512,
    kwargs={'config': env_data["PD_Biped2D_Gaps_Terrain-v0"],
            "render": False, 
            "flatten_observation": True,
            "fall_check": False}
)

gym_register(
    id='PD_Biped2D_Walk-v0',
    entry_point='simAdapter.terrainRLSim:TerrainRLSimWrapper',
    reward_threshold=0.95,
    max_episode_steps=512,
    kwargs={'config': env_data["PD_Biped2D_Flat-v0"],
            "render": False, 
            "flatten_observation": True,
            "fall_check": False}
)

gym_register(
    id='PD_Biped2D_Flat_Stand-v0',
    entry_point='simAdapter.terrainRLSim:TerrainRLSimWrapper',
    reward_threshold=0.95,
    max_episode_steps=512,
    kwargs={'config': env_data["PD_Biped2D_Flat_Stand-v0"],
            "render": False, 
            "flatten_observation": True,
            "fall_check": False}
)

gym_register(
    id='PD-Biped3D-HLC-Soccer-v1',
    entry_point='terrainRLSim:TerrainRLSimWrapper',
    reward_threshold=0.95,
    max_episode_steps=512,
    kwargs={'config': env_data["PD-Biped3D-HLC-Soccer-v1"]}
)

gym_register(
    id='PD-Biped3D-HLC-Soccer-Render-v1',
    entry_point='terrainRLSim:TerrainRLSimWrapper',
    reward_threshold=0.95,
    max_episode_steps=512,
    kwargs={'config': env_data["PD-Biped3D-HLC-Soccer-v1"],
            "render": True}
)

gym_register(
    id='PD_Biped2D_Flat_Walk_MultiTask-v0',
    entry_point='simAdapter.terrainRLSim:TerrainRLSimWrapper',
    reward_threshold=0.95,
    max_episode_steps=512,
    kwargs={'config': env_data["PD_Biped2D_Flat_Walk_MultiTask-v0"],
            "render": False, 
            "flatten_observation": True,
            "fall_check": False}
)
gym_register(
    id='PD_Biped2D_Flat_Walk_MultiTask_Render-v0',
    entry_point='simAdapter.terrainRLSim:TerrainRLSimWrapper',
    reward_threshold=0.95,
    max_episode_steps=512,
    kwargs={'config': env_data["PD_Biped2D_Flat_Walk_MultiTask-v0"],
            "render": True, 
            "flatten_observation": True,
            "fall_check": False}
)
gym_register(
    id='PD_Biped2D_Flat_Walk_MultiTask-v1',
    entry_point='simAdapter.terrainRLSim:TerrainRLSimWrapper',
    reward_threshold=0.95,
    max_episode_steps=512,
    kwargs={'config': env_data["PD_Biped2D_Flat_Walk_MultiTask-v1"],
            "render": False, 
            "flatten_observation": True,
            "fall_check": False}
)

gym_register(
    id='PD_Biped2D_Flat_Walk_MultiTask_Render-v1',
    entry_point='simAdapter.terrainRLSim:TerrainRLSimWrapper',
    reward_threshold=0.95,
    max_episode_steps=512,
    kwargs={'config': env_data["PD_Biped2D_Flat_Walk_MultiTask-v1"],
            "render": True, 
            "flatten_observation": True,
            "fall_check": False}
)

gym_register(
    id='PD-Biped3D-HLC-Obstacles-v2',
    entry_point='terrainRLSim:TerrainRLSimWrapper',
    reward_threshold=0.95,
    max_episode_steps=512,
    kwargs={'config': env_data["PD-Biped3D-HLC-Obstacles-v2"],
            "render": False,
            "flatten_observation": True}
)
gym_register(
    id='PD-Biped3D-HLC-Obstacles-render-v2',
    entry_point='simAdapter.terrainRLSim:TerrainRLSimWrapper',
    reward_threshold=0.95,
    max_episode_steps=512,
    kwargs={'config': env_data["PD-Biped3D-HLC-Obstacles-v2"],
            "render": True,
            "headless_render": True,
            "flatten_observation": True}
)

gym_register(
    id="PD_Biped2D_MultiTask_TerrainVel-v0",
    entry_point='simAdapter.terrainRLSim:TerrainRLSimWrapper',
    reward_threshold=0.95,
    max_episode_steps=512,
    kwargs={'config': env_data["PD_Biped2D_MultiTask_TerrainVel-v0"],
            "render": False, 
            "flatten_observation": True,
            "fall_check": False}
)

gym_register(
    id="PD_Biped2D_MultiTask_TerrainVel_Render-v0",
    entry_point='simAdapter.terrainRLSim:TerrainRLSimWrapper',
    reward_threshold=0.95,
    max_episode_steps=512,
    kwargs={'config': env_data["PD_Biped2D_MultiTask_TerrainVel-v0"],
            "render": True, 
            "flatten_observation": True,
            "fall_check": False}
)

gym_register(
    id="PD_Biped2D_MultiTask_Terrain-v0",
    entry_point='simAdapter.terrainRLSim:TerrainRLSimWrapper',
    reward_threshold=0.95,
    max_episode_steps=512,
    kwargs={'config': env_data["PD_Biped2D_MultiTask_Terrain-v1"],
            "render": False, 
            "flatten_observation": True,
            "fall_check": False}
)

gym_register(
    id="PD_Biped2D_MultiTask_Terrain_Render-v0",
    entry_point='simAdapter.terrainRLSim:TerrainRLSimWrapper',
    reward_threshold=0.95,
    max_episode_steps=512,
    kwargs={'config': env_data["PD_Biped2D_MultiTask_Terrain-v1"],
            "render": True, 
            "flatten_observation": True,
            "fall_check": False}
)
