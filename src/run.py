import argparse
import numpy as np
import matplotlib.pyplot as plt

from MapEnvironment import MapEnvironment
from CarEnvironment import CarEnvironment
from AStarPlanner import AStarPlanner
from RRTPlannerNonholonomic import RRTPlannerNonholonomic

def main(planning_env, planner, start, goal, argplan = 'astar'):

    # Notify.
    input('Press any key to begin planning...')

    planning_env.init_visualizer()

    # Plan.
    plan = planner.BDPlan(start, goal)

    # Visualize the final path.
    tree = None
    visited = None
    if argplan != 'astar':
        tree = planner.tree
    else:
        visited = planner.visited
    planning_env.visualize_plan(plan, tree, visited)
    plt.show()


if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='script for testing planners')

    parser.add_argument('-m', '--map', type=str, default='map1.txt',
                        help='The environment to plan on')
    parser.add_argument('-p', '--planner', type=str, default='astar',
                        help='The planner to run (astar, nonholrrt)')
    parser.add_argument('-t', '--type', type=str, default='train', help='train or test')
    parser.add_argument('-i', '--index', type=str, default='0', help='image index') 
    parser.add_argument('-s', '--start', nargs='+', type=float, required=True)
    parser.add_argument('-g', '--goal', nargs='+', type=float, required=True)

    args = parser.parse_args()
    image_folder = ''
    if args.type == 'test':
        image_folder = 'test_'
    im_path = '../holomonic_data/' + image_folder + 'images/' + args.index + '.png'

    # First setup the environment and the robot.
    dim = 3 if args.planner == 'nonholrrt' else 2
    args.start = np.array(args.start).reshape(dim, 1)
    args.goal = np.array(args.goal).reshape(dim, 1)

    if args.planner == 'nonholrrt':
    	planning_env = CarEnvironment(args.map, args.start, args.goal)
    else:
	    planning_env = MapEnvironment(im_path, args.start, args.goal)

    # Next setup the planner
    if args.planner == 'astar':
        planner = AStarPlanner(planning_env)
    elif args.planner == 'nonholrrt':
    	planner = RRTPlannerNonholonomic(planning_env)
    else:
        print('Unknown planner option: %s' % args.planner)
        exit(0)

    main(planning_env, planner, args.start, args.goal, args.planner)
