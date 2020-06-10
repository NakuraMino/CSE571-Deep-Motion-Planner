import argparse
import numpy as np
import matplotlib.pyplot as plt

from MapEnvironment import MapEnvironment
from AStarPlanner import AStarPlanner

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
    
    image_num = 102 # how many maps do you already have
    total_paths = 250 # what map # do you want to end on?
    with open("data.csv", mode='a', newline='') as csv_file:
        csv_writer = csv.writer(csv_file, delimiter=',')
        for dirname, dirnames, filenames in os.walk('../test_maps'):
            while image_num < total_paths:
                for subdirname in dirnames:
                    
                    map_path = dirname + "/" + subdirname + "/floor_trav_0_v2.png"
                    
                    img_path = "./images/" + str(image_num) + ".png"
                    ticks_path = "./ticks/" + str(image_num) + ".png"

                    planning_env = MapEnvironment(map_path, image_num)
                    cv2.imwrite(img_path, planning_env.get_map())
                    planning_env.init_visualizer()
                    planning_env.visualize_plan(path=ticks_path)

                    image_num += 1

                    for i in range(2):
                        if i == 0:
                            planning_env.setStates()
                        else:
                            planning_env.setRandomStates()

                        args.start = planning_env.start
                        args.goal = planning_env.goal

                        # Next setup the planner
                        planner = AStarPlanner(planning_env, args.epsilon)
                        
                        plan = main(planning_env, planner, args.start, args.goal, args.planner)
                        
                        if plan.shape[1] > 2:
                            for i in range(plan.shape[1] - 1):
                                xt = plan[:,i]
                                xtt = plan[:,i + 1]
                                y = get_label(xt, xtt)
                                csv_writer.writerow([xt[0],xt[1],args.goal[0,0],args.goal[1,0],img_path,y])
                        
                    if image_num == total_paths:
                        break