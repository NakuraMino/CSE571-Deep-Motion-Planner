import numpy as np
from RRTTree import RRTTree
import time

class RRTPlannerNonholonomic(object):

    def __init__(self, planning_env, bias=0.05, max_iter=10000, num_control_samples=25):
        self.env = planning_env                 # Car Environment
        self.tree = RRTTree(self.env)
        self.bias = bias                        # Goal Bias
        self.max_iter = max_iter                # Max Iterations
        self.num_control_samples = 25           # Number of controls to sample

    def Plan(self, start_config, goal_config):
        # TODO: YOUR IMPLEMENTATION HERE

        plan_time = time.time()

        # Start with adding the start configuration to the tree.
        self.tree.AddVertex(start_config)

        plan = []
        plan.append(start_config)
        plan.append(goal_config)

        cost = 0
        plan_time = time.time() - plan_time

        print("Cost: %f" % cost)
        print("Planning Time: %ds" % plan_time)

        return np.concatenate(plan, axis=1)

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