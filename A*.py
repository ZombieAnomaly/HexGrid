from AStar import PriorityQueue
from termcolor import colored
import time, random



class SquareGrid:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.walls = []
    
    def in_bounds(self, id):
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height
    
    def passable(self, id):
        return id not in self.walls
    
    def neighbors(self, id):
        (x, y) = id
        results = [(x+1, y), (x, y-1), (x-1, y), (x, y+1)]
        if (x + y) % 2 == 0: results.reverse() # aesthetics
        results = filter(self.in_bounds, results)
        results = filter(self.passable, results)
        return results

    def distance(self,a,b):
        (x1, y1) = a
        (x2, y2) = b
        return abs(x1 - x2) + abs(y1 - y2)

class GridWithWeights(SquareGrid):
    def __init__(self, width, height):
        super().__init__(width, height)
        self.weights = {}
    
    def cost(self, from_node, to_node):
        return self.weights.get(to_node, 1)

def dijkstra_search(graph, start, goal):
    frontier = PriorityQueue.PriorityQueue()  #PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while not frontier.empty():
        current = frontier.get()
        
        if current == goal:
            break
        
        for next in graph.neighbors(current):
            new_cost = cost_so_far[current]  + graph.cost(current, next)
            # f(n) = g(n) + h(n)

            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost
                frontier.put(next, priority)
                came_from[next] = current
    
    return came_from, cost_so_far

def Astar_search(graph, start, goal, greedyWeight, dijkstrasWeight):
    frontier = PriorityQueue.PriorityQueue()  #PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while not frontier.empty():
        current = frontier.get()
        
        if current == goal:
            break
        
        for next in graph.neighbors(current):
            new_cost = cost_so_far[current]  + graph.cost(current, next)
            # f(n) = g(n) + h(n)

            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                h = heuristic( graph.distance(next, goal), greedyWeight, dijkstrasWeight, graph.cost(current, next))
                priority = new_cost + h
                print("{}-{} | P: {} | H: {} | g(n): {} | h(n): {} | cost: {}".format(current,next, priority,h, graph.distance(next, start), graph.distance(next, goal), graph.cost(current, next)) )
                frontier.put(next, priority)
                came_from[next] = current
    
    return came_from, cost_so_far

def heuristic(h,w1,w2,cost):
    #(3, 6)-(4, 6) | P: 13.6 | H: 9.6 | g(n): 5 | h(n): 6 | cost: 3
    #(4, 6)-(4, 7) | P: 13.5 | H: 8.5 | g(n): 6 | h(n): 5 | cost: 3
    
    h += h * w1
    c = cost * w2
    return h + c

def reconstruct_path(came_from, start, goal):
    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start) # optional
    path.reverse() # optional
    return path

def from_id_width(id, width):
    return (id % width, id // width)


def draw_tile(graph, id, style, width):
    r = " . "
    if 'weights' in style and id in style['weights']: r = " {} ".format(style['weights'][id])
    if 'number' in style and id in style['number']: r = " {} ".format(style['number'][id])
    if 'point_to' in style and style['point_to'].get(id, None) is not None:
        (x1, y1) = id
        (x2, y2) = style['point_to'][id]
        if x2 == x1 + 1: r = " > "
        if x2 == x1 - 1: r = " < "
        if y2 == y1 + 1: r = " v "
        if y2 == y1 - 1: r = " ^ "
    if 'start' in style and id == style['start']: r = colored(" A ",'red')
    if 'goal' in style and id == style['goal']: r = colored(" Z ", 'blue')
    if 'path' in style and id in style['path']: 
        if id != style['path'][0] and id != style['path'][len(style['path'])-1]:
            r = colored(" @ ", 'yellow')
        elif id == style['path'][0]:
            r = colored(" A ",'red')
        else:
            r = colored(" Z ",'blue')
    if id in graph.walls: r = colored(" # ", 'white')
   
    return r

def draw_grid(graph, width=1, **style):
    
    for y in range(graph.height):
        row = ""
        for x in range(graph.width):
            row += "{}".format(draw_tile(graph, (x, y), style, width))
        print(row)
        print()


def execute(greedyWeight, dijkstrasWeight, weights, gridx, gridy):
    '''
    grid = GridWithWeights(gridx, gridy)     
    grid.weights = weights       
    grid_walls = [from_id_width(id, width=30) for id in [21,22,51,52,81,82,111,112,133,134,141,142,163,164,171,172,173,174,175,193,194,201,202,203,204,205,213,214,223,224,243,244,253,254,273,274,283,284,303,304,313,314,333,334,343,344,373,374,403,404,433,434]]
    grid.walls = grid_walls # long list, [(21, 0), (21, 2), ...]
    '''
    grid = GridWithWeights(gridx, gridy)
    grid.walls = [(1, 7), (1, 8), (2, 7), (2, 8), (3, 7), (3, 8),(7,7),(5,7),(6,7)]

    grid.weights = {loc: 3 for loc in [(3, 4), (3, 5), (4, 1), (4, 2),
                                        (4, 3), (4, 4), (4, 5), (4, 6), 
                                        (4, 7), (4, 8), (5, 1), (5, 2),
                                        (5, 3), (5, 4), (5, 5),(5,6), 
                                        (5, 7), (5, 8), (6, 2), (6, 3), 
                                        (6, 4), (6, 5), (6, 7),
                                        (7, 3), (7, 4), (7, 5)]}
    
    start_time = time.time()

    came_from, cost_so_far = Astar_search(grid, start, goal, greedyWeight, dijkstrasWeight)



    print()
    print('Basic graph with weights')
    draw_grid(grid, width=1, weights=grid.weights ,start=start, goal=goal)
    print()

    print('Graph with pointers to parent nodes')
    draw_grid(grid, width=1, point_to=came_from, start=start, goal=goal)
    print()

    print('Graph with cost so far')
    draw_grid(grid, width=1, number=cost_so_far, start=start, goal=goal)
    print()

    draw_grid(grid, width=3, path=reconstruct_path(came_from, start=start, goal=goal))

    print( len(reconstruct_path(came_from, start=start, goal=goal)) )
    print("--- %s seconds ---" % ((time.time() - start_time) ))

    _input = input('0 - exit | 1 - execute | 2 - increase greedy weight | 3 - decrease greedy weight | 4 - increase dijkstras weight | 5 - decrease dijkstras weight | - ')
    checkInput(_input,greedyWeight, dijkstrasWeight)

start = (1,4)
goal = (7,8)
gridx, gridy = 10,10
greedyWeight = .1
dijkstrasWeight = 1
weights = {}
for i in range(0,gridx):
    for j in range(0,gridy):
        key = (i,j)
        weights[key] = random.randrange(1,10)

def checkInput(_input,greedyWeight, dijkstrasWeight):
    if int(_input) == 1:
        return execute(greedyWeight, dijkstrasWeight, weights, gridx, gridy)
    elif int(_input) == 2:
        greedyWeight += .1
        print("greedyWeight: {}".format(greedyWeight))
        _input = input('0 - exit | 1 - execute | 2 - increase greedy weight | 3 - decrease greedy weight | 4 - increase dijkstras weight | 5 - decrease dijkstras weight | - ')
    elif int(_input) == 3:
        greedyWeight -= .1
        print("greedyWeight: {}".format(greedyWeight))
        _input = input('0 - exit | 1 - execute | 2 - increase greedy weight | 3 - decrease greedy weight | 4 - increase dijkstras weight | 5 - decrease dijkstras weight | - ')
    elif int(_input) == 4:
        dijkstrasWeight += .1
        print("dijkstrasWeight: {}".format(dijkstrasWeight))
        _input = input('0 - exit | 1 - execute | 2 - increase greedy weight | 3 - decrease greedy weight | 4 - increase dijkstras weight | 5 - decrease dijkstras weight | - ')
    elif int(_input) == 5:
        dijkstrasWeight -= .1
        print("dijkstrasWeight: {}".format(dijkstrasWeight))
        _input = input('0 - exit | 1 - execute | 2 - increase greedy weight | 3 - decrease greedy weight | 4 - increase dijkstras weight | 5 - decrease dijkstras weight | - ')
    
    checkInput(_input, greedyWeight, dijkstrasWeight)
    
_input = input('0 - exit | 1 - execute | 2 - increase greedy weight | 3 - decrease greedy weight | 4 - increase dijkstras weight | 5 - decrease dijkstras weight | - ')
checkInput(_input, greedyWeight, dijkstrasWeight)
