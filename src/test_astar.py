import argparse
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

from MapEnvironment import MapEnvironment
from CarEnvironment import CarEnvironment
from AStarPlanner import AStarPlanner
from RRTPlannerNonholonomic import RRTPlannerNonholonomic

if __name__ == "__main__":
    
    # Run on testing dataset. Change this to 'train'
    # to run on training dataset.
    args_type = 'test'

    # Whether to show the plot. Keep this to False (unless
    # you want to see the plots for each point in the entire
    # dataset!). If you want to see the plot, use run_astar.py
    # with appropriate arguments
    args_plotting = 'False'

    # True uses the bidirectional planner.
    # False uses unidirectional planner.
    args_bi_directional = 'True'

    if args_type == 'test':
        dataset = pd.read_csv('../holomonic_data/test_data.csv', header=None)
    else:
        dataset = pd.read_csv('../holomonic_data/data.csv')

    # Iterate over entire dataset
    failed = 0
    # dataset_length = len(dataset)
    dataset_length = 0
    map_path_prev = dataset.iloc[0][4]
    for k in range(len(dataset)):
        map_path = dataset.iloc[k][4]

        # Keep this block if only one start, goal configuration per map is to be tested
        # and comment it if entire dataset is to be tested.
        # {
        # if k != 0 and map_path_prev == map_path:
            # continue
        # map_path_prev = map_path
        # }

        args_index = dataset.iloc[k][4][14:-4]
        args_start = (dataset.iloc[k][0], dataset.iloc[k][1])
        args_goal = (dataset.iloc[k][2], dataset.iloc[k][3])

        dataset_length += 1

        print('-------------------------------')
        #print('index =', args_index)
        print('index = {}, map = {}'.format(k, args_index))

        image_folder = ''
        if args_type == 'test':
            image_folder = 'test_'
        im_path = '../holomonic_data/' + image_folder + 'images/' + args_index + '.png'

        # First setup the environment and the robot.
        dim = 2
        args_start = np.array(args_start).reshape(dim, 1).astype(float)
        args_goal = np.array(args_goal).reshape(dim, 1).astype(float)

        planning_env = MapEnvironment(im_path, args_start, args_goal)

        # Next setup the planner
        planner = AStarPlanner(planning_env)
        
        # Plan.
        if args_bi_directional == 'True':
            plan = planner.BDPlan(args_start, args_goal)
        else:
            plan = planner.Plan(args_start, args_goal)

        # Visualize the final path.
        tree = None
        visited = planner.visited

        if planner.NN_failed:
            # print('NN failed to give path from start pos to goal pos!')
            failed += 1
            print('NN failed for map {}, s={}, g={}!'.format(args_index, args_start.T, args_goal.T))
        # else:
            # print('NN successfully found path from start to goal!')

        if args_plotting == 'True':
            planning_env.init_visualizer()
            planning_env.visualize_plan(plan, tree, visited)
            plt.show()
    successful = dataset_length - failed
    print('Successful paths: {}/{}, ({}%)'.format(successful, dataset_length, 100*successful/dataset_length))
