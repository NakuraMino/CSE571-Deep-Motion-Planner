### IMPORTANT:

Github prohibits uploading files > 100mb. This means my ShootingStarNet.pth cannot be uploaded onto github. Instead, please directly download the pth file from:

> https://drive.google.com/file/d/1M6Z6rcYCouqpj0gG-C8XT-sHF26TkHpG/view?usp=sharing

### SRC Folder 

To test AStarNet, or any other neural network model, please run 

> python run.py -p -t -i -s -g

where:
- p is the planner we are imitating (ex: astar or nonholrrt)
- t is whether we're using training data or test data (test or train)
- i is the index of the image we're using (should be 0-199)
- s is the start state
- g is the goal state 

#### Few personal notes:

start/goal/map combinations that work well:
- lots of open space
- no obstacles
> python run.py -p astar -i 1 -s 20 10 -g 80 50
> python run.py -p astar -s 98 44 -g 33 96

start/goal/map combinations that don't work well:
- shortest direct path includes obstacles
> python run.py -p astar -i 0 -s 30 70 -g 75 70
 