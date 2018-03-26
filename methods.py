from __future__ import print_function
# Use priority queues from Python libraries, don't waste time implementing your own
# Check https://docs.python.org/2/library/heapq.html

from heapq import *
from collections import deque


class Agent:
    def __init__(self, grid, start, goal, type):
        self.actions = [(0, -1), (-1, 0), (0, 1), (1, 0)]  # down, left, up, right
        self.grid = grid  # this is just the grid, no clue what it is used for but cool
        self.came_from = {}  # used to track from where you came from i guess... this should be a stack
        self.visited = set()  # this is like the result vector
        self.start = start  # this is the start node
        self.grid.nodes[start[0]][start[1]].start = True  # essentially converts the tuple to a corresponding node
        self.goal = goal  # the node square you are trying to reach
        self.grid.nodes[goal[0]][goal[1]].goal = True
        self.new_plan(type)

    def new_plan(self, type):
        self.finished = False
        self.failed = False
        self.type = type

        if self.type == "dfs":  # checks if depth-first search
            self.frontier = [self.start]  # frontier is a list and you are adding self.start to it
            self.visited = set()
        elif self.type == "bfs":
            self.frontier = deque()  # making a deque data structure
            self.frontier.append(self.start)  # adding the start tuple to this
            self.visited = set()
        elif self.type == "ucs":  # [Hint] you probably also need something like this: heappush(self.frontier, (0, self.start))
            self.frontier = [(0, self.start)]  # (weight, (x,y) ) -> this will be the priority queue later
            self.visited = set()
            self.dict = {}  # this will hold the min value node so far
        # [Hint] you need a dictionary that keeps track of cost
        elif self.type == "astar":
            self.frontier = [(0, self.start)]  # (f, (x,y)) -> this will be the priority queue later
            self.visited = set()
            self.dict = {self.start: 0}  # this will hold the min g value of the grid node

    def show_result(self):
        current = self.goal
        while not current == self.start:
            current = self.came_from[current]
            self.grid.nodes[current[0]][current[1]].in_path = True

    def make_step(self):
        # [Hint] dfs and bfs should look very similar
        if self.type == "dfs":
            self.dfs_step()
        elif self.type == "bfs":
            self.bfs_step()
        # [Hint] ucs and astar should look very similar
        elif self.type == "ucs":
            self.ucs_step()
        elif self.type == "astar":
            self.astar_step()

    def dfs_step(self):
        # checks if the frontier list contains the start node
        if not self.frontier:
            self.failed = True  # set the true flag
            print("no path")
            return
        current = self.frontier.pop()
        print("popped: ", current)
        # sets the node to be visited and not make it the frontier anymore
        self.grid.nodes[current[0]][current[1]].visited = True
        self.grid.nodes[current[0]][current[1]].frontier = False
        self.visited.add(current)  # add the node to the visited list
        # parts [0,-1] for example to x = 0, y = -1 and so forth ... this is essentially unpacking the tuple
        for x, y in self.actions:  # [(0,-1),(-1,0),(0,1),(1,0)] loops through each of these
            # updating the coordinate
            nextstep = (current[0] + x, current[1] + y)
            # if you disable this, you will be going throguh every single adjacent node each time ... rip run-time
            # See what happens if you disable this check here -> also freezes and continues try to update without going anywhere
            if nextstep in self.visited or nextstep in self.frontier:  # already been here
                print("expanded before: ", nextstep)
                continue
            # making sure you are in the matrix
            if 0 <= nextstep[0] < self.grid.row_range:
                if 0 <= nextstep[1] < self.grid.col_range:
                    # checks to see if it is a puddle
                    if not self.grid.nodes[nextstep[0]][nextstep[1]].puddle:
                        if nextstep == self.goal:
                            self.finished = True
                        self.frontier.append(nextstep) # adding adjacent node to the frontier
                        self.grid.nodes[nextstep[0]][nextstep[1]].frontier = True # setting the node to be true in the frontier
                        self.came_from[nextstep] = current   # updating the dictionary with info about from which node this adjacent node came from
                        print("pushed: ", nextstep)
                    else:
                        print("puddle at: ", nextstep)
                else:
                    print("out of column range: ", nextstep)
            else:
                print("out of row range: ", nextstep)

    def bfs_step(self):
        # checks if the frontier list contains the start node
        if not self.frontier:
            self.failed = True  # set the true flag1
            print("no path")
            return
        current = self.frontier.popleft()  # always pop left, insert right
        print("popped: ", current)
        # sets the node to be visited and not make it the frontier anymore
        self.grid.nodes[current[0]][current[1]].visited = True
        self.grid.nodes[current[0]][current[1]].frontier = False
        self.visited.add(current)  # add the node to the visited list
        # ...check dfs
        for x, y in self.actions:  # [(0,-1),(-1,0),(0,1),(1,0)] loops through each of these
            # ...check dfs
            nextstep = (current[0] + x, current[1] + y)
            # ... check dfs
            # See what happens if you disable this check here -> break this code after u finish the assignment
            if nextstep in self.visited or nextstep in self.frontier:  # already been here
                print("expanded before: ", nextstep)
                continue
            # making sure you are in the matrix
            if 0 <= nextstep[0] < self.grid.row_range:
                if 0 <= nextstep[1] < self.grid.col_range:
                    # checks to see if it is a puddle
                    if not self.grid.nodes[nextstep[0]][nextstep[1]].puddle:
                        if nextstep == self.goal:
                            self.finished = True
                        # ... check dfs
                        self.frontier.append(nextstep)
                        # ... check dfs
                        self.grid.nodes[nextstep[0]][nextstep[1]].frontier = True
                        # ... check dfs
                        self.came_from[nextstep] = current
                        print("pushed: ", nextstep)
                    else:
                        print("puddle at: ", nextstep)
                else:
                    print("out of column range: ", nextstep)
            else:
                print("out of row range: ", nextstep)

    def ucs_step(self):
        # [Hint] you can get the cost of a node by node.cost()
        # checks if the frontier list contains the start node
        if not self.frontier:  # if frontier is empty
            self.failed = True  # set the true flag
            print("no path")
            return
        current = heappop(self.frontier)  # always should pop the lowest weight and rearrange the list
        self.visited.add(current[1])  # add the node to the visited set
        print("heap popped : ", current)
        # sets the node to not make it the frontier anymore
        self.grid.nodes[current[1][0]][current[1][1]].visited = True
        self.grid.nodes[current[1][0]][current[1][1]].frontier = False
        if (current[1] == self.goal):
            self.finished = True  # at the point of popping, we know the end node with the smallest weight has been reached
            print(("Final cost: " + str(current[0])))
            return

        # going through all adjacent nodes
        for x, y in self.actions:  # [(0,-1),(-1,0),(0,1),(1,0)] loops through each of these
            # setting nextStep to be the tuple
            nextstep = (current[1][0] + x, current[1][1] + y)  # adjacent node
            # ... check dfs
            # See what happens if you disable this check here -> break this code after u finish the assignment
            if nextstep in self.visited:  # already visited  or ... if nextstep weight is greater than what the already nextstep was
                print("already visited: ", nextstep)
                continue
            # making sure you are in the matrix
            if 0 <= nextstep[0] < self.grid.row_range:
                if 0 <= nextstep[1] < self.grid.col_range:
                    if not self.grid.nodes[nextstep[0]][nextstep[1]].puddle:
                        adjacentCost = current[0] + self.grid.nodes[nextstep[0]][nextstep[1]].cost()  # gets the cost of the current adjacent node
                        if (nextstep in self.dict and adjacentCost >= self.dict[nextstep]):  # that means it exists in the dictionary
                            continue  # no point calculating this, there already exists a path with that node with smaller value
                        self.dict[nextstep] = adjacentCost  # update the cost of the node if doesn't exist or adjacentCost < self.dict[nextstep]
                        heappush(self.frontier, (adjacentCost, nextstep))  # adding adjacent nodes to the heap
                        self.grid.nodes[nextstep[0]][nextstep[1]].frontier = True
                        self.came_from[nextstep] = current[1]
                        print("pushed: ", nextstep)
                    else:
                        self.visited.add(nextstep)
                        print("puddle encountered")
                else:
                    print("out of column range: ", nextstep)
            else:
                print("out of row range: ", nextstep)

    # [Hint] you need to declare a heuristic function for Astar
    def astar_step(self):
        # [Hint] you can get the cost of a node by node.cost()
        # checks if the frontier list contains the start node
        if not self.frontier:  # if frontier is empty
            self.failed = True  # set the true flag
            print("no path")
            return
        current = heappop(self.frontier)  # always should pop the lowest f weight and rearrange the list
        self.visited.add(current[1])  # add the node to the visited set
        print("heap popped : ", current)
        # sets the node to not make it the frontier anymore
        self.grid.nodes[current[1][0]][current[1][1]].visited = True
        self.grid.nodes[current[1][0]][current[1][1]].frontier = False
        if (current[1] == self.goal):
            self.finished = True  # at the point of popping, we know the end node with the smallest weight has been reached
            print(("Final cost: " + str(self.dict[current[1]])))
            return

        # going through all adjacent nodes
        for x, y in self.actions:  # [(0,-1),(-1,0),(0,1),(1,0)] loops through each of these
            # setting nextStep to be the tuple
            nextstep = (current[1][0] + x, current[1][1] + y)  # adjacent node
            # ... check dfs
            # See what happens if you disable this check here -> break this code after u finish the assignment
            if nextstep in self.visited:  # already visited  or ... if nextstep weight is greater than what the already nextstep was
                print("already visited: ", nextstep)
                continue
            # making sure you are in the matrix
            if 0 <= nextstep[0] < self.grid.row_range:
                if 0 <= nextstep[1] < self.grid.col_range:
                    if not self.grid.nodes[nextstep[0]][nextstep[1]].puddle:
                        heuristicValue = abs(self.goal[0] - nextstep[0]) + abs(self.goal[1] - nextstep[1])  # Manhattan Distance
                        adjacentCost = self.dict[current[1]] + self.grid.nodes[nextstep[0]][nextstep[1]].cost() # gets the cost of the current adjacent node
                        adjacentHeuristicCost = adjacentCost + heuristicValue * 10 # this is the f value -> multipled h(n) by 10 to get a faster response
                        if (nextstep in self.dict and adjacentCost >= self.dict[nextstep]):  # that means it exists in the dictionary
                            continue  # no point calculating this, there already exists a path with that node with smaller value
                        self.dict[nextstep] = adjacentCost  # update the cost of the node if doesn't exist or adjacentCost < self.dict[nextstep]
                        heappush(self.frontier, (adjacentHeuristicCost, nextstep))  # adding adjacent nodes to the heap based on the adjacentHeuristicCost
                        self.grid.nodes[nextstep[0]][nextstep[1]].frontier = True
                        self.came_from[nextstep] = current[1]
                        print("pushed: ", nextstep)
                    else:
                        self.visited.add(nextstep)
                        print("puddle encountered")
                else:
                    print("out of column range: ", nextstep)
            else:
                print("out of row range: ", nextstep)
