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

So far, I implemented two planning algorithms for A*. 
- a normal planner that starts from the start state and tries to find a path to the goal state. 
- a bidirectional planner that extends from both the start state and the goal state.
  - I also include a simple replanning algorithm that detects any states that can be removed from the path
In addition, I also implemented a planner for the Nonholonomic car. This planner seems to work well with a dropout rate 
of around 0.1-0.2.


Below are a quick list of commands that work/don't work for each of the planners
> Key:
> - *   : works for both the normal and bidirectional planner 
> - **  : works only for normal planner
> - *** : works for bidirectional planner only
> - ^   : Nonholonomic car

##### Commands that work

> python run_astar.py -i 1 -s 20 10 -g 80 50   * 
> python run_astar.py -s 98 44 -g 33 96        *
> python run_astar.py -i 15 -s 60 10 -g 80 120 **
> python run_astar.py -i 15 -s 60 10 -g 15 40  ***
   
> python run_rrt.py -i 75 -s 100 70 3.14 -g 80 10 6.2   (really struggles but eventually gets there)
> python run_rrt.py -i 1 -s 100 30 2 -g 20 110 0.5      (straight line)
> python run_rrt.py -i 37 -s 90 110 6 -g 20 60 2        (curvy boi)

##### Doesn't Work 

> python run_astar.py -i 0 -s 30 70 -g 75 70

> python run_rrt.py -i 37 -s 90 110 1 -g 20 60 2        (curvy boi but bad start angle, works with p=0.7)