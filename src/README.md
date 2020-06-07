### IMPORTANT:

Github prohibits uploading files >100mb. This means my ShootingStarNet.pth cannot be uploaded onto github. Instead, please directly download the pth file from:

> https://drive.google.com/file/d/1M6Z6rcYCouqpj0gG-C8XT-sHF26TkHpG/view?usp=sharing

### SRC Folder 

To test AStarNet, or any other neural network model, please run 

> python run_astar.py -t -i -s -g
> python run_rrt.py -t -s -s -g

where:
- t is whether we're using training data or test data (test or train) *
- i is the index of the image we're using (should be 0-199)
- s is the start state
- g is the goal state 

* not currently available for use

#### Few personal notes:

So far, I implemented two planning algorithms. 
- a normal planner that starts from the start state and tries to find a path to the goal state. 
- a bidirectional planner that extends from both the start state and the goal state.
  - I also include a simple replanning algorithm that detects any states that can be removed from the path

Below are a quick list of commands that work/don't work for each of the AStar planners 

##### Works for both 

> python run_astar.py -i 1 -s 20 10 -g 80 50
> python run_astar.py -s 98 44 -g 33 96

##### Unidirectional Planner (No replanning)

> python run_astar.py -i 15 -s 60 10 -g 80 120

##### Works Bidirectional Planner with replanning

> python run_astar.py -i 15 -s 60 10 -g 15 40

##### Doesn't Work for Either

> python run_astar.py -i 0 -s 30 70 -g 75 70