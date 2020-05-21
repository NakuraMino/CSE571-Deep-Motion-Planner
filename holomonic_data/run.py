import argparse
import numpy as np
import matplotlib.pyplot as plt

from MapEnvironment import MapEnvironment
from AStarPlanner import AStarPlanner

def main(planning_env, planner, start, goal, argplan = 'astar'):

    # Notify.
    input('Press any key to begin planning...')

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
    plt.show()

    return plan




if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='script for testing planners')

    parser.add_argument('-m', '--map', type=str, default='map1.txt',
                        help='The environment to plan on')    
    parser.add_argument('-p', '--planner', type=str, default='astar',
                        help='The planner to run (astar, rrt, rrtstar, nonholrrt)')
    parser.add_argument('-s', '--start', nargs='+', type=float, required=True)
    parser.add_argument('-g', '--goal', nargs='+', type=float, required=True)
    parser.add_argument('-eps', '--epsilon', type=float, default=1.0, help='Epsilon for A*')
    parser.add_argument('-eta', '--eta', type=float, default=1.0, help='eta for RRT/RRT*')
    parser.add_argument('-b', '--bias', type=float, default=0.05, help='Goal bias for RRT/RRT*')

    args = parser.parse_args()

    # First setup the environment and the robot.
    dim = 3 if args.planner == 'nonholrrt' else 2
    args.start = np.array(args.start).reshape(dim, 1)
    args.goal = np.array(args.goal).reshape(dim, 1)
    
    planning_env = MapEnvironment(args.map, args.start, args.goal)

    # Next setup the planner
    planner = AStarPlanner(planning_env, args.epsilon)
    
    plan = main(planning_env, planner, args.start, args.goal, args.planner)

    for xt in plan:
        
        pass