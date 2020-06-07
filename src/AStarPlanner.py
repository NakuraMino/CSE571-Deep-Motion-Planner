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

        net = None
        version = 1

        if version == 0:
            from astarnet import AStarNet    
            net = AStarNet()
            net.load_state_dict(torch.load("./models/astarnet500.pth", map_location="cpu"))
        elif version == 1:
            from shootingstarnet import ShootingStarNet
            net = ShootingStarNet()
            net.load_state_dict(torch.load("./models/ShootingStarNet.pth", map_location="cpu"))
        net.eval()

        plan = []
        plan.append(start_config.copy())
        cost = 0
        iters = 0
        curr_state = start_config
        self.visit(curr_state)
        while np.sum(np.abs(curr_state - goal_config)) >= 3 and iters < 300:
            input_state = torch.from_numpy(np.concatenate((curr_state, goal_config), axis=0)).float().T
            
            action = net((input_state, self.env.torch_map))
            # action = torch.argmax(action) + 1
            # delta, c = self.action_to_delta(action)
            delta, c = self.getViableAction(action, curr_state)
            if delta is None:
                break
            curr_state += delta
            plan.append(np.copy(curr_state))
            cost += c
            iters += 1
        
        state_count = len(plan)
        print("States Expanded: %d" % state_count)
        print("Cost: %f" % cost)

        return np.concatenate(plan, axis=1)

    def BDPlan(self, start_config, goal_config):
        '''
        Bi-Directional Planning Algorithm that incrementally plans from goal_state and
        start_state. 
        '''
        net = self.getNetwork(1)
        none, cost, forward = 0, 0, True
        start_plan, goal_plan = [start_config], [goal_config]
        curr, dest = start_config.copy(), goal_config.copy()

        while np.sum(np.abs(curr - dest)) > np.sqrt(2):
            delta, c = None, None
            if forward:
                delta, c = self.extendPoint(curr, dest, net)
            else:
                delta, c = self.extendPoint(dest, curr, net)
            if delta is not None:
                cost, none = c + cost, 0
                if forward:
                    x_new = curr.copy() + delta
                    start_plan.append(x_new)
                    curr = x_new.copy()
                else:
                    x_new = dest.copy() + delta
                    goal_plan.insert(0, x_new)
                    dest = x_new.copy()
            else:
                none += 1
            # print("curr", curr)
            # print("dest", dest)
            forward = not forward
            if none >= 2:
                break
        start_plan, goal_plan = self.replanning(start_plan, goal_plan)
        plan = start_plan
        plan.extend(goal_plan)
        state_count = len(plan)
        print("Cost: %f" % cost)
        print("States Expanded: %d" % state_count)
        
        return np.concatenate(plan, axis=1)
    
    def extendPoint(self, curr, dest, net):
        input_state = torch.from_numpy(np.concatenate((curr, dest), axis=0)).float().T
        action = net((input_state, self.env.torch_map))
        delta, c = self.getViableAction(action, curr)
        return delta, c

    def replanning(self, start_plan, goal_plan):
        '''
        optimizes path by detecting unnecessary visited states and removing them
        '''
        for i in range(len(start_plan)):
            start_xt = start_plan[i]
            for j in range(len(goal_plan)):
                goal_xt = goal_plan[j]
                if self.dist(goal_xt, start_xt) <= np.sqrt(2):
                    return start_plan[0:i], goal_plan[j:]
        return start_plan, goal_plan

    def dist(self, start, goal):
        return np.sqrt(np.sum((start - goal) ** 2))

    def getNetwork(self, version):
        net = None
        if version == 0:
            from astarnet import AStarNet    
            net = AStarNet()
            net.load_state_dict(torch.load("./models/astarnet500.pth", map_location="cpu"))
        elif version == 1:
            from shootingstarnet import ShootingStarNet
            net = ShootingStarNet()
            net.load_state_dict(torch.load("./models/ShootingStarNet.pth", map_location="cpu"))
        net.eval()
        return net

    def getViableAction(self, action, curr_state):
        action = action.detach().numpy()
        index = np.argmax(action) + 1
        delta, c = self.action_to_delta(index)
        next_state = curr_state + delta
        count = 0
        while self.already_visited(next_state) or not self.env.state_validity_checker(next_state):    
            action = np.delete(action, index - 1)
            if action.shape[0] == 0:
                return None, None
            index = np.argmax(action) + 1
            delta, c = self.action_to_delta(index)
            next_state = curr_state + delta        
            count += 1
        self.visit(next_state)
        return delta, c

    def visit(self, state):
        y, x = state[0, 0], state[1, 0]
        self.visited[int(y), int(x)] = 1
    
    def already_visited(self, state):
        y, x = state[0, 0], state[1, 0]
        return self.visited[int(y), int(x)] == 1
    
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