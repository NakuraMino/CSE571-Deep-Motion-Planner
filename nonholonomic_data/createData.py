import argparse
import numpy as np
import matplotlib.pyplot as plt

from CarEnvironment import CarEnvironment
from RRTPlannerNonholonomic import RRTPlannerNonholonomic
from map_utils import Map

def main(planning_env, planner, start, goal, argplan = 'astar'):

    # Notify.
    # input('Press any key to begin planning...')

    planning_env.init_visualizer()

    # Plan.
    plan, actions = planner.Plan(start, goal)

    # Visualize the final path.
    tree = None
    visited = None
    tree = planner.tree
    # TODO: Comment out later
    planning_env.visualize_plan(plan, tree, visited)
    # plt.show()
    return plan, actions

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
    dim = 3 # change to 3 for holonomic
    
    image_num = 0
    total_paths = 1
    with open("test_data.csv", mode='w', newline='') as csv_file:
        csv_writer = csv.writer(csv_file, delimiter=',')
        for dirname, dirnames, filenames in os.walk('../train_maps'):
            while image_num < total_paths:
                for subdirname in dirnames:
                    if image_num == total_paths:
                        break
                    map_path = dirname + "/" + subdirname + "/floorplan.yaml"
                    
                    img_path = "./images/" + str(image_num) + ".jpg"
                    
                    m = Map(map_path, laser_max_range=4, downsample_factor=1)
                    im = m.return_image()
                    cv2.imwrite(img_path, im)

                    planning_env = CarEnvironment(m, image_num)
                    image_num += 1

                    args.start = planning_env.start
                    args.goal = planning_env.goal

                    # Next setup the planner
                    planner = RRTPlannerNonholonomic(planning_env, args.epsilon)
                    
                    plan, actions = main(planning_env, planner, args.start, args.goal, args.planner)
                    
                    if plan.shape[1] > 2:
                        gool = plan[0,-1]
                        for i in range(plan.shape[1] - 1):
                            xt = plan[:,i]
                            y = actions[i]
                            csv_writer.writerow([xt[0],xt[1],gool[0],gool[1],img_path,y[0], y[1]])