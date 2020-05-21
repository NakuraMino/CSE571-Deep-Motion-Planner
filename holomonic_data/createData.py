import argparse
import numpy as np
import matplotlib.pyplot as plt

from MapEnvironment import MapEnvironment
from AStarPlanner import AStarPlanner
from map_utils import Map

def main(planning_env, planner, start, goal, argplan = 'astar'):

    # Notify.
    # input('Press any key to begin planning...')

    planning_env.init_visualizer()

    # Plan.
    plan = planner.Plan(start, goal)

    # Visualize the final path.
    tree = None
    visited = None
    if argplan != 'astar':
        tree = planner.tree
    else:
        visited = planner.visited
    # TODO: Comment out later
    planning_env.visualize_plan(plan, tree, visited)
    # plt.show()
    return plan

def get_label(xt, xtt):
    dy = xtt[0] - xt[0]
    dx = xtt[1] - xt[1]
    if dx == -1 and dy == 1:
        return 3
    if dx == 0 and dy == 1:
        return 5
    if dx == 1 and dy == 1:
        return 8
    if dx == -1 and dy == 0:
        return 2
    if dx == 1 and dy == 0:
        return 7
    if dx == -1 and dy == -1:
        return 1
    if dx == 0 and dy == -1:
        return 4
    if dx == 1 and dy == -1:
        return 6

def get_random_state(env):
    state = np.zeros((2,1))
    state[0,0] = np.random.randint(0, env.ylimit[1])
    state[1,0] = np.random.randint(0, env.xlimit[1])
    while not env.state_validity_checker(state):
        state = np.zeros((2,1))
        state[0,0] = np.random.randint(0, env.ylimit[1])
        state[1,0] = np.random.randint(0, env.xlimit[1])
    return state

if __name__ == "__main__":
    import csv 
    import os
    import cv2

    parser = argparse.ArgumentParser(description='script for testing planners')

    parser.add_argument('-p', '--planner', type=str, default='astar',
                        help='The planner to run (astar, rrt, rrtstar, nonholrrt)')
    parser.add_argument('-s', '--start', nargs='+', type=float, default=0)
    parser.add_argument('-g', '--goal', nargs='+', type=float, default=0)
    parser.add_argument('-eps', '--epsilon', type=float, default=1.0, help='Epsilon for A*')

    args = parser.parse_args()

    # First setup the environment and the robot.
    dim = 2 # change to 3 for holonomic
    
    image_num = 0
    
    with open("data.csv", mode='w', newline='') as csv_file:
        csv_writer = csv.writer(csv_file, delimiter=',')
        for dirname, dirnames, filenames in os.walk('../train_maps'):
    
            while True:
                for subdirname in dirnames:
                    map_path = dirname + "/" + subdirname + "/floorplan.yaml"
                    
                    img_path = "./images/" + str(image_num) + ".jpg"
                    if image_num == 200:
                        break
                    image_num += 1

                    m = Map(map_path, laser_max_range=4, downsample_factor=1)
                    im = m.return_image()
                    cv2.imwrite(img_path, im)

                    args.start = get_random_state(m)
                    args.goal = get_random_state(m)

                    planning_env = MapEnvironment(m, args.start, args.goal, image_num)

                    # Next setup the planner
                    planner = AStarPlanner(planning_env, args.epsilon)
                    
                    plan = main(planning_env, planner, args.start, args.goal, args.planner)
                    
                    if plan.shape[1] > 2:
                        for i in range(plan.shape[1] - 1):
                            xt = plan[:,i]
                            xtt = plan[:,i + 1]
                            y = get_label(xt, xtt)
                            csv_writer.writerow([xt[0],xt[1],xtt[0],xtt[1],map_path,y])