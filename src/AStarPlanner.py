import numpy as np
import torch
import cv2

class AStarPlanner(object):    
    def __init__(self, planning_env):
        self.env = planning_env
        self.nodes = {}
        self.visited = np.zeros(self.env.map.shape)

    def Plan(self, start_config, goal_config):
        # TODO: YOUR IMPLEMENTATION HERE

        # from afitnet import AStarNet
        # net.load_state_dict(torch.load("./models/afitnet.pth", map_location="cpu"))
        
        from astarnet import AStarNet
        net = AStarNet()
        net.load_state_dict(torch.load("./models/astarnet.pth", map_location="cpu"))
        net.eval()

        plan = []
        plan.append(start_config.copy())
        cost = 0
        iters = 0
        curr_state = start_config
        while np.sum(curr_state != goal_config) != 0 and iters < 100:
            input_state = torch.from_numpy(np.concatenate((curr_state, goal_config), axis=0)).float().T
            
            action = net((input_state, self.env.torch_map))
            action = torch.argmax(action) + 1
            delta, c = self.action_to_delta(action)
            curr_state += delta
            plan.append(np.copy(curr_state))
            cost += c
            iters += 1
    
        print(plan)
        
        state_count = len(plan)
        print("States Expanded: %d" % state_count)
        print("Cost: %f" % cost)

        return np.concatenate(plan, axis=1)


    def getViableAction(self, action, curr_state):
        index = torch.argmax(action) + 1
        delta, c = self.action_to_delta(index)
        next_state = curr_state + delta
        while not self.env.state_validity_checker(next_state):
            torch.delete()
            break
            pass
        a = x + 1
        return next_state
    
    def action_to_delta(self, action):
        delta = np.zeros((2,1)) # [y, x]
        cost = 1
        if action == 3:
            delta[0], delta[1] = 1, -1
            cost = np.sqrt(2) 
        elif action == 5:
            delta[0], delta[1] = 1, 0
        elif action == 8:
            delta[0], delta[1] = 1, 1
            cost = np.sqrt(2)
        elif action == 2:
            delta[0], delta[1] = 0, -1
        elif action == 7:
            delta[0], delta[1] = 0, 1
        elif action == 1:
            delta[0], delta[1] = -1, -1
            cost = np.sqrt(2)
        elif action == 4:
            delta[0], delta[1] = -1, 0
        elif action == 6:
            delta[0], delta[1] = -1, 1
            cost = np.sqrt(2)
        return delta, cost
        


