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
        p = 0.1
        plan_time = time.time()
        while p <= 0.7:
            net = self.getNetwork(4, p)
            plan = [start_config]
            cost = 0
            iters = 0
            fails = 0
            # Start with adding the start configuration to the tree.
            # self.tree.AddVertex(start_config)
            curr_state = start_config.copy()

            while not self.env.lax_goal_criterion(curr_state, goal_config) and iters < 200:
                input_state = torch.from_numpy(np.concatenate((curr_state, goal_config), axis=0)).float().T
                action = net((input_state, self.env.torch_map))
                action = action.detach().numpy()
                linear_vel, steer_angle = action[0,0], action[0,1]
                linear_vel, steer_angle = self.cap_motion(linear_vel, steer_angle)
                
                x_new, c = self.env.simulate_car(curr_state, linear_vel, steer_angle)
                if x_new is not None:    
                    fails = 0
                    curr_state = x_new.copy()
                    plan.append(x_new)
                    cost += c
                iters += 1
            
            if self.env.lax_goal_criterion(curr_state, goal_config):
                break
            p += 0.2
        p = min(p, 0.7)
        plan_time = time.time() - plan_time
        print("dropout rate: %f" % p)
        print("Num Iters: %d" % iters)
        print("Cost: %f" % cost)
        print("Planning Time: %ds" % plan_time)

        return np.concatenate(plan, axis=1)

    def cap_motion(self, linear_vel, steer_angle):
        if np.abs(linear_vel) > self.env.max_linear_vel:
            linear_vel *= (self.env.max_linear_vel / np.abs(linear_vel))
        if np.abs(steer_angle) > self.env.max_steer_angle:
            steer_angle *= (self.env.max_steer_angle / np.abs(steer_angle))
        return linear_vel, steer_angle

    def replanner(self, plan):
        '''
        TODO: Come up with some way to smoothen path
        - perhaps smoothen path by taking states that are close to each other and removing them
        - using the planner algorithm to plan from xt to xt_1 (instead of xt to goal) and then use that as a new path?
        - it would probably make more sense to have another neural planner that takes in xt, xt_1 as input and predicts action
        - ^ actually, scrap that idea. if we have xt and xt_1, we should be able to directly calculate the action taken.
        '''
        pass

    def getNetwork(self, version, p):
        net = None
        from rrtnet import RRTNet    
        net = RRTNet(p)
        if version == 0:
            net.load_state_dict(torch.load("./models/RRTNet.pth", map_location="cpu"))
        elif version == 1:
            net.load_state_dict(torch.load("./models/RRTNetNoDrop.pth", map_location="cpu")) 
        elif version == 2:
            net.load_state_dict(torch.load("./models/RRTNet200.pth", map_location="cpu")) 
        elif version == 3:
            net.load_state_dict(torch.load("./models/RRTwoNet.pth", map_location="cpu"))
        elif version == 4:
            net.load_state_dict(torch.load("./models/RRTTRRNet.pth", map_location="cpu"))
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
