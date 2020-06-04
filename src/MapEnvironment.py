import numpy as np
from matplotlib import pyplot as plt
import cv2
import torch

class MapEnvironment(object):
    
    def __init__(self, mapfile, start, goal, epsilon=0.01):

        # Obtain the boundary limits
        # self.map = np.loadtxt('map1.txt')
        # print(self.map)
        
        map_image = cv2.imread(mapfile,0)
        self.map_image = map_image.copy()
        whites = map_image >= 250
        blacks = map_image < 250
        map_image[whites] = 0
        map_image[blacks] = 1
        self.map = map_image.astype(np.float64)
        self.torch_map = torch.from_numpy(self.map).unsqueeze(0)

        self.xlimit = [0, np.shape(self.map)[1]-1]
        self.ylimit = [0, np.shape(self.map)[0]-1]

        self.goal = goal
        self.epsilon = epsilon

        # Check if start and goal are within limits and collision free
        if not self.state_validity_checker(start) or not self.state_validity_checker(goal):
            raise ValueError('Start and Goal state must be within the map limits');
            exit(0)

    def sample(self):
        # Sample random clear point from map
        clear = np.argwhere(self.map == 0)
        idx = np.random.choice(len(clear))
        return clear[idx, :].reshape((2, 1))

    def goal_criterion(self, config, goal_config):
        """ Return True if config is close enough to goal

            @param config: a [2 x 1] numpy array of a state
            @param goal_config: a [2 x 1] numpy array of goal state
        """        
        return self.compute_distance(config, goal_config) < self.epsilon

    def compute_distance(self, start_config, end_config):
        """ A function which computes the distance between
            two configurations. 

            @param start_config: a [2 x 1] numpy array of current state
            @param end_config: a [2 x 1] numpy array of goal state
        """
        # TODO: YOUR IMPLEMENTATION HERE
        return 0


    def state_validity_checker(self, config):
        """ Return True if all states are valid

            @param config: a [2 x n] numpy array of states
        """
        # TODO: YOUR IMPLEMENTATION HERE
        y, x = config[0,0], config[1,0]
        return self.map[int(y),int(x)] == 0

    def edge_validity_checker(self, config1, config2):
        """ Return True if edge is valid

            @param config1: a [2 x 1] numpy array of state
            @param config2: a [2 x 1] numpy array of state
        """
        assert(config1.shape == (2, 1))
        assert(config2.shape == (2, 1))
        n = max(self.xlimit[1], self.ylimit[1])
        x_vals = np.linspace(config1[0], config2[0], n).reshape(1, n)
        y_vals = np.linspace(config1[1], config2[1], n).reshape(1, n)
        configs = np.vstack((x_vals, y_vals))
        return self.state_validity_checker(configs)

    def h(self, config):
        """ Heuristic function for A*

            @param config: a [2 x 1] numpy array of state
        """
        # TODO: YOUR IMPLEMENTATION HERE
        return 0

    def init_visualizer(self):
        """ Initialize visualizer
        """

        self.fig = plt.figure()
        self.ax1 = self.fig.add_subplot(1, 1, 1)

        # Plot img
        visit_map = 1 - np.copy(self.map) # black is obstacle, white is free space
        self.ax1_img = self.ax1.imshow(visit_map, interpolation="nearest", cmap="gray")

    def visualize_plan(self, plan=None, tree=None, visited=None):
        """
            Visualize the final path
            @param plan: a final [2 x n] numpy array of states
        """
        visit_map = 1 - np.copy(self.map) # black is obstacle, white is free space
        
        self.ax1.cla()

        if visited is not None:
            visit_map[visited == 1] = 0.5
        self.ax1.imshow(visit_map, interpolation="nearest", cmap="gray")

        if tree is not None:
            for idx in range(len(tree.vertices)):
                if idx == tree.GetRootID():
                    continue
                econfig = tree.vertices[idx]
                sconfig = tree.vertices[tree.edges[idx]]
                x = [sconfig[0], econfig[0]]
                y = [sconfig[1], econfig[1]]
                self.ax1.plot(y, x, 'r')

        if plan is not None:
            for i in range(np.shape(plan)[1] - 1):
                x = [plan[0,i], plan[0,i+1]]
                y = [plan[1,i], plan[1,i+1]]
                plt.plot(y, x, 'b', linewidth=3)
                self.fig.canvas.draw()
                plt.pause(.025) 

        self.fig.canvas.draw()
        plt.pause(1e-10) 

