

import numpy as np
# from OpenGL.GL.images import glReadPixels
# from OpenGL.GL import *
# from OpenGL.GLU import *
# from OpenGL.GLUT import *
from time import sleep
# import cv2
import time
from builtins import property

def checkDataIsValid(data, verbose=False, scale=1.0, identifier="Data"):
    """
        Checks to make sure the data going into the exp buffer is not garbage...
        Returns True if the data is valid
    """
    import numpy as np
    # print(identifier, " data: ", data)
    
    valid = True
    if (isinstance(data, list)):
        # print ("data is list", data)
        for dat  in data:
            valid = valid and checkDataIsValid(dat, verbose=verbose, scale=scale, identifier=identifier)
        return valid
    """
    if (isinstance(data[0][0], list) or (isinstance(bounds[0][0], np.ndarray))):
        ### Multi Agent or multi state simulation
        valid = True
        for data__ in data:
            valid = valid and checkDataIsValid(data__, verbose=verbose, scale=scale, identifier=identifier)
    """
    bad_value_boundary=100000
    data = np.array(data)
    if (not np.all(np.isfinite(data))):
        if ( verbose ):
            less_ = np.isfinite(data)
            bad_indecies = np.where(less_ == False)
            print (identifier + " not finite: ", less_ )
            print ("Bad Value indx: ", bad_indecies)
            bad_values_ = data[bad_indecies]
            print ("Bad Values: ", bad_values_)
        return False

    if (np.any(np.less(data, -bad_value_boundary*scale))):
        if ( verbose ):
            less_ = np.less(data, -1000.0*scale)
            bad_indecies = np.where(less_ == True)
            print (identifier + " too negative: ", less_ )
            print ("Bad Value indx: ", bad_indecies)
            bad_values_ = data[bad_indecies]
            print ("Bad Values: ", bad_values_)
        return False
    
    if (np.any(np.greater(data, bad_value_boundary*scale))):
        if ( verbose ):
            less_ = np.greater(data, bad_value_boundary*scale)
            bad_indecies = np.where(less_ == True)
            bad_values_ = data[bad_indecies]
            print (identifier + " too positive: ", less_ )
            print ("Bad Value indx: ", bad_indecies)
            bad_values_ = data[bad_indecies]
            print ("Bad Values: ", bad_values_)
        return False
    
    return True

class ActionSpace(object):
    """
        Wrapper for the action space of an env
    """
    
    def __init__(self, action_space):
        self._minimum = np.array(action_space[0])
        self._maximum = np.array(action_space[1])
        self._shape = np.array(action_space[1]).shape
        
    def getMinimum(self):
        return self._minimum

    def getMaximum(self):
        return self._maximum
    
    @property
    def low(self):
        return self._minimum
    
    @property
    def high(self):
        return self._maximum
    
    @property
    def shape(self):
        return self._shape

class TerrainRLSimWrapper(object):
    """
        Wrapper for the TerrainRLSim env to make function calls more simple
    """
    def __init__(self, sim, render=False, config=None):
        
        self._sim = sim
        self._render = render
        self._done = None
        self._done_multiAgent = None
        self._steps = 0
        
        self._config = config
        if ("process_visual_data" in self._config
        and (self._config["process_visual_data"] == True)):
            self._visual_state = [0] * self._config["timestep_subsampling"]
            self._imitation_visual_state = [0] * self._config["timestep_subsampling"]
        print ("TerrainRLSim Config: ", self._config)

        if ("headless_render" in self._config 
                and (self._config["headless_render"] == True)
                and (self._render == "yes")):
            import matplotlib.pyplot as plt

            plt.ion()
            self._fig, (self._bellman_error_ax) = plt.subplots(1, 1, sharey=False, sharex=True)
            img_ = self.getFullViewData()
            self._bellman_error_ax.imshow(img_, origin='lower')
            self._bellman_error_ax.set_title("visual Data: ")
            # self._bellman_error_ax = plt.imshow(img_, origin='lower')
            # plt.title("visual Data: ")
            # plt.grid(b=True, which='major', color='black', linestyle='--')
            # plt.grid(b=True, which='minor', color='g', linestyle='--')
            self._fig.set_size_inches(8.0, 4.5, forward=True)
            plt.show()

        if ( config != {}):
            ### Render first frame
            self.render()
            # time.sleep(2)
            self.reset()
        
            act_low = [-1] * self.getEnv().getActionSpaceSize()
            act_high = [1] * self.getEnv().getActionSpaceSize() 
            action_space = [act_low, act_high]
            action_space = self.getEnv().getActionSpaceBounds()
            self._action_space = ActionSpace(action_space)
            if ("process_visual_data" in self._config
                and (self._config["process_visual_data"] == True)):
                ob_low = (np.prod(self._visual_state[0].shape) * len(self._visual_state)) * [0]
                ob_high = (np.prod(self._visual_state[0].shape) * len(self._visual_state)) * [1]
                observation_space = [ob_low, ob_high]
                self._observation_space = ActionSpace(observation_space)
            else:
                ob_low = [-1] * self.getEnv().getObservationSpaceSize()
                ob_high = [1] * self.getEnv().getObservationSpaceSize() 
                observation_space = [ob_low, ob_high]
                self._observation_space = ActionSpace(observation_space)
        
    def render(self, headless_step=False):
        if (self._render):
            self._sim.display()
            if ("headless_render" in self._config 
                and (self._config["headless_render"] == True)
                and headless_step
                and (self._render == "yes")):
                print("Headless rendering:")
                img_ = self.getFullViewData()
                ax_img = self._bellman_error_ax.images[-1]
                ax_img.set_data(img_)
                # ax_img = self._bellman_error_ax.set_data(img_)
                # self._bellman_error_ax.canvas.draw()
                self._fig.canvas.draw()                      # View in default viewer
        
    def updateAction(self, action):
        # print ("step action: ", action)
        if (self._sim.getNumAgents() > 1): ### Multi Character simulation
            for i in range(self._sim.getNumAgents()):
                # print("action[i]: ", action[i])
                self._sim.updateActionForAgent(i, action[i])
        else:
            if ("flatten_observation" in self._config
            and (self._config["flatten_observation"])):
                self._sim.updateAction(action)
            else:
                self._sim.updateAction(action[0])
            
        self._sim.handleUpdatedAction()
        
    def getLLCState(self):
        ob = []
        if (self._sim.getNumAgents() > 0): ### Multi Character simulation
            for i in range(self._sim.getNumAgents()):
                state = self._sim.getLLCStateForAgent(i)
                len_ = len(state)
                # print("i: ", i, " len_", len_)
                state = np.array(state)
                ob.append(state)
            # ob = np.array(ob)
            ob = np.reshape(ob, (self._sim.getNumAgents(), len_))
        else:
            ob = self._sim.getLLCState()
            ob = np.reshape(np.array(ob), (-1, len(ob)))
        return self.checkState(ob)
    
    def updateLLCAction(self, action):
        if (self._sim.getNumAgents() > 0): ### Multi Character simulation
            for i in range(self._sim.getNumAgents()):
                self._sim.updateLLCActionForAgent(i, action[i])
        else:
            self._sim.updateLLCAction(action[0])
            
    def update(self):
        self.simUpdate()
        if (self._sim.getNumAgents() > 0): ### Multi Character simulation
            ### End of epoch when first agent falls
            """
            for a in range(self._sim.getNumAgents()):
                if (self._sim.endOfEpochForAgent(a)):
                    self._done = True
                    return
            self._done = False
            """
            ### End of epoch when last agent falls
            """
            fall_ = True
            for a in range(self._sim.getNumAgents()):
                fall_ = fall_ and self._sim.endOfEpochForAgent(a)
            self._done = fall_
            """
            ### Update fall stats
            self._done_multiAgent = [ self._done_multiAgent[i] or self._sim.endOfEpochForAgent(i) 
                                      for i in range(self._sim.getNumAgents())]
            # print (self._done_multiAgent)
            ### End epoch when half of agents have fallen
            fall_s = 0
            for a in range(self._sim.getNumAgents()):
                if ( self._sim.endOfEpochForAgent(a) ):
                    fall_s = fall_s + 1
            if ( fall_s > ((self._sim.getNumAgents()/2.0) + 0.002)):
                self._done = True
                return
            else:
                self._done = False
        else:
            self._done = self._done or self.endOfEpochForAgent(0)
        # self.render()
        # print("Trying to render...")
        
    def agentHasFallenMultiAgent(self):
        # print ("Done_MA: ", self._done_multiAgent)
        return self._done_multiAgent
        
    def agentHasFallen(self):
        if (self._sim.getNumAgents() > 0): ### Multi Character simulation
            ### End of epoch when first agent falls
            """
            for a in range(self._sim.getNumAgents()):
                if (self._sim.endOfEpochForAgent(a)):
                    self._done = True
                    return
            self._done = False
            """
            ### End of epoch when last agent falls
            """
            fall_ = True
            for a in range(self._sim.getNumAgents()):
                fall_ = fall_ and self._sim.endOfEpochForAgent(a)
            self._done = fall_
            """
            ### End epoch when more than half of agents have fallen
            fall_s = 0
            for a in range(self._sim.getNumAgents()):
                if ( self._done_multiAgent[a] ):
                    fall_s = fall_s + 1
            if ( fall_s > ((self._sim.getNumAgents()/2.0)+0.002)):
                return True
        else:
            return self._sim.agentHasFallen()
    
    def getCharacaterState(self):
        return self._sim.getState()
    
    def getObservation(self):
        ob = []
        if (self._sim.getNumAgents() > 1): ### Multi Character simulation
            for i in range(self._sim.getNumAgents()):
                ### get all states and check that they are different
                state = np.array(self._sim.getStateForAgent(i))
                # print ("Agent: ", i, " state: ", state.shape)
                ob.append(state)
            ob_ = np.array(ob)
            # print ("ob_: ", ob_[0].shape)
            # for h in range(ob_.shape[0]):
            #     print ("ob_ ", h, ": ", ob_[h].shape)
            
            # print ("ob_: ", repr(ob_))
            # ob = np.reshape(ob_, (self._sim.getNumAgents(), -2))
            ob = np.reshape(ob_, (self._sim.getNumAgents(), len(ob[0])))
        else:
            if ( ("use_dual_pose_state_representations" in self._config)
                     and (self._config["use_dual_pose_state_representations"] == True)):
                    ob_ = []
                    ob = np.array(self._sim.getState())
                    ob = ob.flatten()
                    # ob = np.reshape(np.array(ob), (-1, len(ob)))
                    ob_.append(ob)

                    # This is just supposed to be mltiple modes of the agent state
                    # ob = np.array(self.getVisualState()) 
                    ob = np.array(self._sim.getState())
                    ob = ob.flatten()
                    ob_.append(ob)
                    # print ("viz state shape:", np.array(ob_).shape)
                    return self.checkState([ob_])
                
            if ("process_visual_data" in self._config
                and (self._config["process_visual_data"] == True)
                and ("use_dual_state_representations" in self._config
                     and (self._config["use_dual_state_representations"] == True))
                and
                    ("use_multimodal_state_representations" in self._config
                     and (self._config["use_multimodal_state_representations"] == True))):
                    ob_ = []
                    ob = self._sim.getState()
                    # ob = np.reshape(np.array(ob), (-1, len(ob)))
                    ob_.append(ob)
                    
                    ob = np.array(self.getVisualState())
                    ob = ob.flatten()
                    ### Add pose state after pixel state, also me being lazy and duplicating pose data
                    ob = np.concatenate((ob, self._sim.getState()), axis=0)
                    # print ("vis ob shape: ", ob.shape)
                    ob_.append(ob)
                    return self.checkState([ob_])
                
            if ("process_visual_data" in self._config
                and (self._config["process_visual_data"] == True)
                and ("use_dual_state_representations" in self._config
                     and (self._config["use_dual_state_representations"] == True))):
                    ob_ = []
                    ob = self._sim.getState()
                    # ob = np.reshape(np.array(ob), (-1, len(ob)))
                    ob_.append(ob)
                    ob = np.array(self.getVisualState())
                    ob = ob.flatten()
                    # print ("vis ob shape: ", ob.shape)
                    ob_.append(ob)
                    return self.checkState([ob_])
            if ("process_visual_data" in self._config
                and (self._config["process_visual_data"] == True)
                and ("use_dual_viz_state_representations" in self._config
                     and (self._config["use_dual_viz_state_representations"] == True))):
                    ob_ = []
                    ob = np.array(self.getVisualState())
                    ob = ob.flatten()
                    # ob = np.reshape(np.array(ob), (-1, len(ob)))
                    ob_.append(ob)

                    ob = np.array(self.getImitationVisualState())
                    ob = ob.flatten()
                    ob_.append(ob)
                    # print ("viz state shape:", np.array(ob_).shape)
                    return self.checkState([ob_])
                
            elif ("process_visual_data" in self._config
                    and (self._config["process_visual_data"] == True)):
                # print("Getting visual state")
                ob = np.array(self.getVisualState())
                ob = np.reshape(np.array(ob), (-1, 
                       (np.prod(ob.shape))))
            else:
                # print("Getting char state")
                ob = self._sim.getState()
                ob = np.reshape(np.array(ob), (-1, len(ob)))
            # ob = np.array(ob)
        if ("flatten_observation" in self._config
            and (self._config["flatten_observation"])):
            ob = ob.flatten()
        return self.checkState(ob)
    
    def simUpdate(self):
        
        if ( "timestep_subsampling" in self._config ):
            ### The first render will be the same as the last from the previous step
            start=0
            if (self._config["timestep_subsampling"] > 1):
                if ("process_visual_data" in self._config
                        and (self._config["process_visual_data"] == True)):
                    self._visual_state[0] = self._visual_state[self._config["timestep_subsampling"]-1]
                    start=1
                    if (self._config["also_imitation_visual_data"]):
                        self._imitation_visual_state[0] = self._imitation_visual_state[self._config["timestep_subsampling"]-1]
                # self._sim.update()
                # self.render()
            for i in range(start, self._config["timestep_subsampling"]):
                self._sim.update()
                self.render()
                # print ("grabbing frame: ", i)
                if ("process_visual_data" in self._config
                    and (self._config["process_visual_data"] == True)):
                    self._visual_state[i] = self._getVisualState()
                    if (self._config["also_imitation_visual_data"]):
                        self._imitation_visual_state[i] = self._getImitationVisualState()
        else:
            self._sim.update()
            
    def step(self, action):
        """
            Adding multi character support
        """
        # action = action[0]
        action = np.array(action, dtype="float64")
        # print ("step action: ", action)
        self.updateAction(action)
        
        # for i in range(15):
        reward = np.zeros((self.getNumAgents(),1))
        if ( "control_return" in self._config and (self._config["control_return"] == True) ):
            i=0
            while ( (not self._sim.needUpdatedAction()) and (i < 50 )):
                # print ("Controlling return")
                self.update()
                self.render(headless_step=True)
                i=i+1
                reward = reward + np.array(self.calcRewards())
            reward = reward / i    
        else:
            self.update()
            self.render(headless_step=True)
            reward = self.calcRewards()
            
            
        # if ( self._render == True ):
        #    self._sim.display()
        # print("Num Agents: ", self._sim.getNumAgents())
        
        ob = self.getObservation()
        self._steps = self._steps + 1    
            
        # self._done = self._sim.agentHasFallen() or self._done or (self._steps >= self._config["time_limit"])
        # observation, reward, done, info
        # ob = np.array(ob)
        # print ("ob shape: ", ob.shape)
        if ("flatten_observation" in self._config
            and (self._config["flatten_observation"])):
            reward = reward[0][0]
        
        # ob[0,0] = np.nan
        
        return self.checkState(ob), self.checkState(reward), self._done, self._config
        
    def checkState(self, state):
        if (not checkDataIsValid(state)):
            state = np.zeros_like(state)
            self._done = True
            self._sim.reload()
            print ("Found obs nan")
        return state
        
    def calcRewardForAgent(self, a):
        return self._sim.calcRewardForAgent(a)
    
    def getNumAgents(self):
        return self._sim.getNumAgents()
    
    def endOfEpochForAgent(self, a):
        return self._sim.endOfEpochForAgent(a)
    
    def calcRewards(self):
        reward = []
        if (self._sim.getNumAgents() > 0): ### Multi Character simulation
            for i in range(self._sim.getNumAgents()):
                ### get all states and check that they are different
                # reward.append([self._sim.calcRewardForAgent(i)])
                ### This might not work so well for multichar simulations. Need to make agentHasFallen for each agent
                # reward.append([self._sim.calcRewardForAgent(i) * int(not self._done_multiAgent[i])])
                if ("use_forward_vel_reward" in self._config
                    and (self._config["use_forward_vel_reward"] == True)):
                    dist = self._sim.calcVelocity(i)-1
                    reward__ = np.exp((dist*dist)*-1.5)
                    reward.append([reward__])
                else:
                    reward.append([self._sim.calcRewardForAgent(i)])
        else:
            if ("use_forward_vel_reward" in self._config
                    and (self._config["use_forward_vel_reward"] == True)):
                dist = self._sim.calcVelocity()-1
                reward__ = np.exp((dist*dist)*-1.5)
                reward = reward__
            else:
                reward = self._sim.calcReward()
            
        return reward
        
    def reset(self):
        self._sim.initEpoch()
        self._done = False
        self._done_multiAgent = [False for i in range(self._sim.getNumAgents())]
        if ( "timestep_subsampling" in self._config ):
            for i in range(self._config["timestep_subsampling"]):
                if ("process_visual_data" in self._config
                    and (self._config["process_visual_data"] == True)):
                    self._visual_state[i] = self._getVisualState()
                    if (self._config["also_imitation_visual_data"]):
                        self._imitation_visual_state[i] = self._getImitationVisualState()
        ob = self.getObservation()
        self._steps = 0
        return ob
    
    def initEpoch(self):
        self.reset()
        
    def finish(self):
        """
            Unload simulation, free memory.
        """
        self._sim.finish()
        
    def getActionSpace(self):
        return self._action_space
    
    @property
    def action_space(self):
        return self._action_space
    
    @property
    def observation_space(self):
        return self._observation_space
    
    def endOfEpoch(self):
        return self._done
        
    def init(self):
        # self._sim.init()
        if ("rendering_key_presses" in self._config):
            for key_ in self._config["rendering_key_presses"]:
                print ("Pressing key: ", key_)
                # self.onKeyEvent(key_.decode("utf-8"), 0, 0)
                if (type(key_) == int):
                    self.onKeyEvent(key_, 0, 0)
                else:
                    self.onKeyEvent(ord(key_), 0, 0)
        
    def getEnv(self):
        return self._sim
    
    def onKeyEvent(self, c, x, y):
        self.getEnv().onKeyEvent(c, x, y)
        
    def setRandomSeed(self, seed):
        """
            Set the random seed for the simulator
            This is helpful if you are running many simulations in parallel you don't
            want them to be producing the same results if they all init their random number 
            generator the same.
        """
        # print ( "Setting random seed: ", seed )
        self.getEnv().setRandomSeed(seed)
        
    def getImitationState(self):
        im_state = self.getEnv().getImitationState()
        im_state = np.reshape(np.array(im_state), (-1, len(im_state)))
        return im_state
    
    def getImitationStateAtTime(self, animTime):
        return self.getEnv().getImitationStateAtTime(animTime)
    
    def getFullViewData(self):
        from skimage.measure import block_reduce
        ### Get pixel data from view
        img = self.getEnv().getPixels(0,
                           0, 
                           800, 
                           450)
        # assert(np.sum(img) > 0.0)
        ### reshape into image, colour last
        img = np.reshape(img, (450, 
                           800, 3))
        ### downsample image
        ### convert to greyscale
        # assert(np.sum(img) > 0.0)
        return img
        
    def getViewData(self):
        from skimage.measure import block_reduce
        ### Get pixel data from view
        img = np.array(self.getEnv().getPixels(self._config["image_clipping_area"][0],
                           self._config["image_clipping_area"][1], 
                           self._config["image_clipping_area"][2], 
                           self._config["image_clipping_area"][3]))
        # assert(np.sum(img) > 0.0)
        ### reshape into image, colour last
        if ("skip_reshape" in self._config and
            (self._config["skip_reshape"] == True)):
            pass
        else:
            # print ("img shape:", img.shape)
            img = np.reshape(img, (self._config["image_clipping_area"][3], 
                               self._config["image_clipping_area"][2], 3)) / 255.0
            ### downsample image
            img = block_reduce(img, block_size=(self._config["downsample_image"][0], 
                                                self._config["downsample_image"][1], 
                                                self._config["downsample_image"][2]), func=np.mean)
        ### convert to greyscale
        if (self._config["convert_to_greyscale"]):
            img = np.mean(img, axis=2)
        # assert(np.sum(img) > 0.0)
        return img
    
    def getAgentVelocity(self):
        # add velocity
        state_ = []
        vel = self._sim.calcVelocity3D()
        state_.extend(vel)
        return state_
    
    def getImitationAgentVelocity(self):
        # add velocity
        state_ = []
        vel = self._sim.getImitationVelocity3D()
        state_.extend(vel)
        return state_
    
    def _getVisualState(self):
        ### toggle things that we don't want in the rendered image
        # k for kin char
        # j for char info in top left
        # '0' for disable ground rendering
        # '9' for don't draw character
        # '8' for camera track kin char instead
        # '7' to disable drawing background grid
        # 'v' to render the simchar like the kin char (colour and shape)
        self.onKeyEvent(ord('k'), 0, 0)
        self.onKeyEvent(ord('j'), 0, 0)
        self.onKeyEvent(ord('0'), 0, 0)
        self.onKeyEvent(ord('7'), 0, 0)
        self.onKeyEvent(ord('v'), 0, 0)
        self.render()
        img = self.getViewData()
        self.onKeyEvent(ord('k'), 0, 0)
        self.onKeyEvent(ord('j'), 0, 0)
        self.onKeyEvent(ord('0'), 0, 0)
        self.onKeyEvent(ord('7'), 0, 0)
        self.onKeyEvent(ord('v'), 0, 0)
        
        if ("append_camera_velocity_state" in self._config
            and (self._config["append_camera_velocity_state"] == True)):
            ### Add velocity to state
            vel_ = self.getAgentVelocity()
            # print ("img shape: ", img.shape)
            img = np.concatenate((img.flatten(), vel_), axis=0)
        # self.render()
        return img
    
    def getVisualState(self):
        if ("transpose_image_data" in self._config
            and (self._config["transpose_image_data"] == True)):
            return np.transpose(self._visual_state)
        else:
            return self._visual_state
    
    def _getImitationVisualState(self):
        self.onKeyEvent(ord('7'), 0, 0)
        self.onKeyEvent(ord('8'), 0, 0)
        self.onKeyEvent(ord('9'), 0, 0)
        self.onKeyEvent(ord('j'), 0, 0)
        self.onKeyEvent(ord('0'), 0, 0)
        # self._sim.update()
        self.render()
        img = self.getViewData()
        self.onKeyEvent(ord('0'), 0, 0)
        self.onKeyEvent(ord('j'), 0, 0)
        self.onKeyEvent(ord('9'), 0, 0)
        self.onKeyEvent(ord('8'), 0, 0)
        self.onKeyEvent(ord('7'), 0, 0)
        
        if ("append_camera_velocity_state" in self._config
            and (self._config["append_camera_velocity_state"] == True)):
            ### Add velocity to state
            vel_ = self.getImitationAgentVelocity()
            img = np.concatenate((img.flatten(), vel_), axis=0)
        # self.render()
        return img
    
    def getImitationVisualState(self):
        if ("transpose_image_data" in self._config
            and (self._config["transpose_image_data"] == True)):
            return np.transpose(self._imitation_visual_state)
        else:
            return self._imitation_visual_state
        
    def getMultiModalRewardState(self):
        state_ = np.array(self._sim.getState())
        viz_state_ = np.array(self.getImitationVisualState())
        # print ("state_ shape: ", state_.shape)
        # print ("viz_state_ shape: ", viz_state_.shape, " mean: ", np.mean(viz_state_), " std: ", np.std(viz_state_))
        multi_state_ = np.reshape(np.concatenate((viz_state_.flatten(), state_), axis=0), 
                                  newshape=(1, state_.size + viz_state_.size))
        return multi_state_
 
    def getMultiModalImitationState(self):
        state_ = np.array(self.getImitationState()).flatten()
        viz_state_ = np.array(self.getImitationVisualState()).flatten()
        
        # print ("state_ shape: ", state_.shape)
        # print ("viz_state_ shape: ", viz_state_.shape, " state_: ", state_.shape)
        multi_state_ = np.reshape(np.concatenate((viz_state_, state_), axis=0), 
                                  newshape=(1, state_.size + viz_state_.size))
        return multi_state_       
    
    def computeReward(self, state, imitationState):
        """
            Proxy verson of true reward
        """
        state = np.array(state).flatten()
        imitationState = self.getImitationStateAtTime(imitationState)
        # imitationState = self.getImitationState()
        ### pose difference
        # print ("state: ", state[0][3:])
        # print ("imitationState: ", imitationState[3:])
        weights = self.getEnv().getJointWeights()
        diff = np.array(state - imitationState)
        
        full_weights = np.zeros_like(diff)
        if ("sim_type" in self._config
            and (self._config["sim_type"] == "3D")): ### 2D
            full_weights[0] = weights[0] ## root height
            # print ("Computing 3D reward")
            ## it goes [ all poses ... all _roations]
            ### [pos_x, pos_y, pos_z, q_s, q_x, q_y, q_z, ...., 
            ###  vel_x, vel_y, vel_z, ang_vel_x, ang_vel_y, ang_vel_z ]
            # full_weights.append(weights[0])
            mid_point = (len(weights) * 7) + 1 ### How many relative distance pose values are there
            for i in range(1,len(weights)):
                full_weights[((i-1)* 7) + 1] = weights[i] ### link rel x pos
                full_weights[((i-1) * 7) + 2] = weights[i] ### link rel y pos
                full_weights[((i-1) * 7) + 3] = weights[i] ### link rel z pos
                
                full_weights[((i-1)* 7) + 4] = 0 ### link q.s pos
                full_weights[((i-1) * 7) + 5] = 0 ### link q.x pos
                full_weights[((i-1) * 7) + 6] = 0 ### link q.y pos
                full_weights[((i-1)* 7) + 7] = 0 ### link q.z pos
                
                full_weights[((i-1) * 6) + mid_point] = weights[i] ### link rel x vel
                full_weights[((i-1) * 6) + mid_point + 1] = weights[i] ### link rel y vel
                full_weights[((i-1) * 6) + mid_point + 2] = weights[i] ### link rel z vel
                
                full_weights[((i-1) * 6) + mid_point + 3] = 0 ### link rel x ang vel
                full_weights[((i-1) * 6) + mid_point + 4] = 0 ### link rel y ang vel
                full_weights[((i-1) * 6) + mid_point + 5] = 0 ### link rel z ang vel
                
            
        else:
            full_weights[0] = weights[0] ## root height
            ## it goes [ all poses ... all _roations]
            for i in range(1,len(weights)):
                full_weights[((i-1)* 2) + 1] = weights[i] ### link rel x pos
                full_weights[((i-1) * 2) + 2] = weights[i] ### link rel y pos
                full_weights[((i-1) * 2) + int(len(full_weights)/2)] = weights[i] ### link rel x vel
                full_weights[((i-1) * 2) + (int(len(full_weights)/2) + 1)] = weights[i] ### link rel y vel
        
        dist = np.sum(np.square(diff) * full_weights)
        dist = dist / float(len(weights))
        
        ### vel 
        # print("reward dist: ", dist)
        reward_ = np.exp((dist*dist)*-1.5)
        return reward_
    
    def computeImitationReward(self, reward_func):
        """
            Uses a learned imitation based reward function to
            compute the reward in the simulation 
        """
        # print("self.getImitationState(): ", self.getVisualState())
        # print("self.getImitationVisualState(): ", self.getImitationVisualState())
        if ("use_multimodal_state_representations" in self._config
            and (self._config["use_multimodal_state_representations"] == True)):
            multi_state_ = self.getMultiModalRewardState()
            # dist = reward_func(np.reshape(self._sim.getState() ,newshape=(1, state_.size)),
            #                 np.reshape(viz_state_, newshape=(1, viz_state_.size)))
            # print ("multi_state_ shape: ", multi_state_.shape)
            # tmp_dist = reward_func(self.getMultiModalImitationState())
            # print ("tmp_dist for pure imitation data: ", tmp_dist)
            # pose_diff = self.getImitationState() - self._sim.getState()
            # print ("pose_diff: ", pose_diff)
            dist = reward_func(multi_state_)
        elif ("process_visual_data" in self._config
            and (self._config["process_visual_data"] == True)):
            state_ = np.array(self.getVisualState())
            dist = reward_func(np.reshape(self.getVisualState() ,newshape=(1, state_.size)),
                            np.reshape(self.getImitationVisualState(), newshape=(1, state_.size)))
        else:
            state_ = np.array(self.getImitationState())
            # print ("state_ shape: ", np.array(state_).shape)
            dist = reward_func(np.reshape(self.getCharacaterState() ,newshape=(1, state_.size)),
                                np.reshape(self.getImitationState(), newshape=(1, state_.size)))
            # print (reward_func, dist)
            # return dist
        # print("reward dist: ", len(dist), dist)
        return -dist[0]
    
    def getStateFromSimState(self, simState):
        # print ("simState: ", simState)
        simState_ = self.getSimState()
        self.setSimState(simState)
        state_ = self.getObservation()
        self.setSimState(simState_)
        return state_
    
    def setSimState(self, state_):
        # print ("state shape: ", np.array(state_).shape )
        # print ("state shape: ", repr(state_) )
        self.getEnv().setSimState(state_)
    
    def getSimState(self):
        return self.getEnv().getSimState()
    
    def getAnimationTime(self):
        return self.getEnv().getAnimationTime()
    
    def getTaskID(self):
        return self.getEnv().getTaskID()
    
    def set_task(self, id):
        # print ("task id: ", id[0], type(id[0]))
        self.getEnv().setTaskID(id)
        
        
    def sample_tasks(self, tasks):
        import random
        tasks = random.sample(range(self.getEnv().GetNumMotions()), tasks)
        return tasks 
    
    def log_diagnostics(self, paths, prefix):
        pass
        
    
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

def paramsToKeyValues(paramsDict):
    
    out = []
    for key in paramsDict:
        if (key == "comment__"):
            continue
        print ("key: ", key, " value: ", paramsDict[key])
        out.append("-" + key + "=")
        out.append(paramsDict[key])
    return out

def getEnv(env_name, render=False, GPU_device=None):
    import os, sys, json
    print ("****** TerrianRL render: ", render)
    terrainRL_PATH = os.environ['TERRAINRL_PATH']
    print ("terrainRL_PATH: ", terrainRL_PATH)
    sys.path.append(terrainRL_PATH+'/lib')
    from simAdapter import terrainRLAdapter
    
    env_data = getEnvsList()
    # print("Envs: ", json.dumps(env_data, indent=2))

    if (env_name in env_data):
        config_file = env_data[env_name]['config_file']
    elif (env_name == None):
        sim_ = TerrainRLSimWrapper(None, render=render, config={})
        return sim_
    else:
        print("Env: ", env_name, " not found. Check that you have the correct env name.")
        return None
    
    ## place holder  
    more_args = []
    if ("TerrainRL_Parameters" in env_data[env_name]):
        more_args = paramsToKeyValues(env_data[env_name]["TerrainRL_Parameters"])
    args_ = ['train', '-arg_file=', terrainRL_PATH+'/'+config_file, 
                                        '-relative_file_path=', terrainRL_PATH+'/'] + more_args
    print ("TerrainRL args: ", args_)
    sim = terrainRLAdapter.cSimAdapter(args_)
    if (render == True or (render == "yes")):
        sim.setRender(True)
    else:
        sim.setRender(False)
    if ("headless_render" in env_data[env_name] 
        and (env_data[env_name]["headless_render"] == True)):
        print("Enable headless rendering")
        sim.setHeadlessRender(True)
        
    if (GPU_device is not None):
        sim.setRenderingGPUDevicveIndex(GPU_device)
    sim.init()
    
    if ("action_fps" in env_data[env_name]):
        sim.changeAnimTimestep(1.0/env_data[env_name]["action_fps"])
        simTimeStep = sim.getAnimTimestep()
    
    if ("process_visual_data" in env_data[env_name]
        and (env_data[env_name]["process_visual_data"] == True)):
        sim.changeAnimTimestep(1.0/env_data[env_name]["action_fps"])
        simTimeStep = sim.getAnimTimestep()
        print("Old sime timestep: ", simTimeStep)
        if ("timestep_subsampling" in env_data[env_name]
            and (env_data[env_name]["timestep_subsampling"] > 1.0)):
            sim.changeAnimTimestep(simTimeStep/(env_data[env_name]["timestep_subsampling"]-1))
    
        
    conf__ = env_data[env_name]
    
    sim_ = TerrainRLSimWrapper(sim, render=render, config=conf__)
    
    sim_.init()
    return sim_

if __name__ == '__main__':

    env = getEnv(env_name="PD_Biped2D_Gaps_Terrain-v0", render=True)
    
    env.reset()
    actionSpace = env.getActionSpace()
    
    for i in range(100):
        action = ((actionSpace.getMaximum() - actionSpace.getMinimum()) * np.random.uniform()) + actionSpace.getMinimum()  
        observation, reward,  done, info = env.step(action)
        env.render()
        print ("Done: ", done)
        # if (done):
        #     env.reset()
        
    env.finish()
    print (env)
    