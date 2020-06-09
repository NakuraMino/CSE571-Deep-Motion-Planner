import numpy as np
from CarEnvironment import CarEnvironment

start = np.zeros((3,1))
start[0,0], start[1,0], start[2,0] = 81, 97, 1.896820872	

goal = np.zeros((3,1))
goal[0,0], goal[1,0], goal[2,0] = 65.52469766, 79.2351592, 2.674787626
map_path = './images/1.jpg'
planning_env = CarEnvironment(map_path, 0)

print(planning_env.compute_distance(start, goal))
            