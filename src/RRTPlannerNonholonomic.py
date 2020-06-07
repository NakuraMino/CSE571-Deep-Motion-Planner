import numpy as np
from RRTTree import RRTTree
import time
import torch 
import cv2

class RRTPlannerNonholonomic(object):

    def __init__(self, planning_env, bias=0.05, max_iter=10000, num_control_samples=25):
        self.env = planning_env                 # Car Environment
        self.tree = RRTTree(self.env)
        self.bias = bias                        # Goal Bias
        self.max_iter = max_iter                # Max Iterations
        self.num_control_samples = 25           # Number of controls to sample

    def Plan(self, start_config, goal_config):
        # TODO: YOUR IMPLEMENTATION HERE
        
        net = self.getNetwork(1)
        plan_time = time.time()
        plan = [start_config]
        cost = 0
        iters = 0
        # Start with adding the start configuration to the tree.
        # self.tree.AddVertex(start_config)

        curr_state = start_config.copy()
        while not self.env.lax_goal_criterion(curr_state, goal_config) and iters < 300:
            input_state = torch.from_numpy(np.concatenate((curr_state, goal_config), axis=0)).float().T
            action = net((input_state, self.env.torch_map))
            action = action.detach().numpy()
            linear_vel, steer_angle = action[0,0], action[0,1]
            # print(linear_vel, steer_angle)
            x_new, c = self.env.simulate_car(curr_state, linear_vel, steer_angle)
            if x_new is not None:    
                curr_state = x_new.copy()
                plan.append(x_new)
                cost += c
            iters += 1

        plan_time = time.time() - plan_time
        print("Cost: %f" % cost)
        print("Planning Time: %ds" % plan_time)

        return np.concatenate(plan, axis=1)

    def getNetwork(self, version):
        net = None
        if version == 0:
            from rrtnet import RRTNet    
            net = RRTNet()
            net.load_state_dict(torch.load("./models/rrtnet.pth", map_location="cpu"))
        elif version == 1:
            from rrtnet import RRTNet    
            net = RRTNet()
            net.load_state_dict(torch.load("./models/rrtnetnodrop.pth", map_location="cpu"))            
        # net.eval()
        return net

    def extend(self, x_near, x_rand):
        """ Extend method for non-holonomic RRT

            Generate n control samples, with n = self.num_control_samples
            Simulate trajectories with these control samples
            Compute the closest closest trajectory and return the resulting state (and cost)
        """
        # TODO: YOUR IMPLEMENTATION HERE
        return x_near
            
    def sample(self, goal):
        # Sample random point from map
        if np.random.uniform() < self.bias:
            return goal

        return self.env.sample()