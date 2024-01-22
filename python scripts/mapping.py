# Title: Mapping
# Description: A* pathfinding algorithm for the robot (Currently cant recieve data from the robot)
# Author: Jack Fitton
# Version: 1.0
# Date: 21/01/2024
# (c) Copyright 2024 Jack Fitton

import pygame
import time
import heapq
import random

#alive progress
from alive_progress import alive_bar

#colours
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0 , 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
GREY =  (255, 255, 0)
PINK = (128, 128, 128)
YELLOW = (64, 64, 64)



cells = {}
gamesettings = {
    "start": None,
    "end": None,
}

def pickAlgo():
    algo = {
        "1": "A*",
        "2": "Dykstra",
        "3": "Depth First Search",
        "4": "Flood Fill",
        "5": "Rapid Random Tree"
    }

    while True:

        for key, value in algo.items():
            print(f"{key}: {value}")

        chosenAlgo = input("Choose an algorithm: ")

        if algo[chosenAlgo] == "A*":
            print("A* selected")
            break
        elif algo[chosenAlgo] == "Dykstra":
            print("Dykstra selected")
            break
        elif algo[chosenAlgo] == "Depth First Search":
            print("Depth First Search selected")
            break
        elif algo[chosenAlgo] == "Flood Fill":
            print("Flood Fill selected")
            break
        elif algo[chosenAlgo] == "Rapid Random Tree":
            print("Rapid Random Tree selected")
            break
        else:
            print("Invalid choice, try again")


    return algo[chosenAlgo]

algo = pickAlgo()

#Maze is 2m x 1.5m with 10cm cells
GRID_X = 200 // 5
GRID_Y = 150 // 5

PIXELS_X = 1000
PIXELS_Y = 1000

GRID_PIXEL_SIZE_X = PIXELS_X / GRID_X
GRID_PIXEL_SIZE_Y = PIXELS_Y / GRID_Y

# Cell class
# This stores each cells position, state and neightbours
class Cell:
    def __init__(self, x, y, screen):
        self.x = x #centre of cell
        self.y = y #centre of cell
        self.color = WHITE
        self.is_start = False
        self.is_end = False
        self.is_obstacle = False
        self.screen = screen
        self.border = 1
        self.parent = None  

    def set_start(self):
        if self.is_start:
            return
        self.is_start = True
        self.is_end = False
        self.is_obstacle = False
        self.color = GREEN
        self.draw()

    def set_end(self):
        if self.is_end:
            return
        self.is_end = True
        self.is_start = False
        self.is_obstacle = False
        self.color = RED
        self.draw()

    def set_obstacle(self):
        if self.is_obstacle:
            return
        self.is_obstacle = True
        self.is_start = False
        self.is_end = False
        self.color = BLUE
        self.draw()
    
    def set_empty(self):
        self.is_start = False
        self.is_end = False
        self.is_obstacle = False
        self.color = WHITE
        self.draw()

    def draw(self):
        fill_cell(self.screen, self.x, self.y, self.color)

    def isObstacle(self):
        return self.is_obstacle

# Resets the maze
def reset(screen):
    for x in range(GRID_X):
        for y in range(GRID_Y):
            #create cell object
            cells[(x,y)].set_empty()

    gamesettings["start"] = None
    gamesettings["end"] = None
    draw_grid(screen)

def RunAStar(cells):

    #proforms A* pathfinding algorithm
    def aStar(cells):
        start = cells[gamesettings["start"]]
        end = cells[gamesettings["end"]]

        with alive_bar(len(cells) -1 ) as bar:
        
            # Initialize open and closed sets
            open_set = set([start])
            closed_set = set()

            # Initialize scores for start node
            g_score = {node: float('inf') for node in cells}
            g_score[start] = 0
            f_score = {node: float('inf') for node in cells}
            f_score[start] = heuristic(start, end)

            while open_set:
                current = min(open_set, key=lambda x: f_score[x])

                current.color = PINK
                current.draw()
                pygame.display.flip()
                time.sleep(0.01)

                if current == end:
                    return reconstruct_path(current)

                open_set.remove(current)
                closed_set.add(current)

                for neighbor in get_neighbors(current, cells):
                    if neighbor in closed_set or neighbor.is_obstacle: #This is being ignored
                        #print("[A*] Obstacle at ", neighbor.x, neighbor.y)
                        continue

                    tentative_g_score = g_score[current] + distance(current, neighbor)

                    if neighbor not in open_set:
                        open_set.add(neighbor)
                        neighbor.color = YELLOW
                        neighbor.draw()
                        pygame.display.flip()
                        time.sleep(0.01)
                    elif tentative_g_score >= g_score[neighbor]:
                        continue

                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, end)
                    neighbor.parent = current
                    bar()

        return None

    # Heuristic function for A* - Manhattan distance
    def heuristic(a, b):
        # Manhattan distance on a square grid
        return abs(a.x - b.x) + abs(a.y - b.y)

    # Returns a list of neighboring cells
    def get_neighbors(node, cells):
        # Returns a list of neighboring cells
        neighbors = []

        # Check if cell is within bounds
        if node.x > 0:
            neighbors.append(cells[(node.x - 1, node.y)])
        if node.x < GRID_X - 1:
            neighbors.append(cells[(node.x + 1, node.y)])
        if node.y > 0:
            neighbors.append(cells[(node.x, node.y - 1)])
        if node.y < GRID_Y - 1:
            neighbors.append(cells[(node.x, node.y + 1)])

        return neighbors

    # Distance function for A* - As all cells are the same distance apart, this is always 1
    def distance(a, b):
        # Distance between two cells
        return 1

    # Reconstructs the path from the end node - passed to the render function
    def reconstruct_path(node):
        path = []
        current = node
        while current.parent is not None:
            if current.is_obstacle:  # Skip if it's an obstacle
                #print(f"Error: Obstacle at {current.x}, {current.y} found in path")
                break
            path.append(current)
            current = current.parent
        path.append(current)  # Add the start cell
        return path[::-1]

    return aStar(cells)

def RunDykstra(cells):


    def get_neighbors(cell):
        neighbors = []
        directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]  # Up, Down, Left, Right

        for dx, dy in directions:
            x, y = cell.x + dx, cell.y + dy
            if 0 <= x < GRID_X and 0 <= y < GRID_Y and not cells[(x, y)].isObstacle():
                neighbors.append(cells[(x, y)])

        return neighbors
    

    def dijkstra(start, end):
        # Distance from start to every other vertex; initialize with infinity
        dist = {cell: float('inf') for cell in cells.values()}
        dist[start] = 0

        # Priority queue to hold vertices to be processed
        pq = [(0, start.x, start.y, start)]  # Distance and cell with coordinates for comparison

        while pq:
            current_dist, _, _, current_cell = heapq.heappop(pq)

            # Early exit if we reached the end
            if current_cell == end:
                break

            if current_dist > dist[current_cell]:
                continue

            for neighbor in get_neighbors(current_cell):
                distance = current_dist + 1  # Assuming each edge has weight 1

                if neighbor.is_obstacle:
                    continue

                if distance < dist[neighbor]:
                    dist[neighbor] = distance
                    neighbor.parent = current_cell  # Track the path
                    neighbor.color = YELLOW
                    neighbor.draw()
                    pygame.display.flip()
                    heapq.heappush(pq, (distance, neighbor.x, neighbor.y, neighbor))

        # Reconstruct path from end to start
        path = []
        while end:
            path.append(end)
            end = end.parent

        return path[::-1]  # Return reversed path
    
    start = cells[gamesettings["start"]]
    end = cells[gamesettings["end"]]
    return dijkstra(start, end)

def depthFirstSearch(cells):

    def dfs(start, end):
        # Depth-first search algorithm
        stack = [start]
        visited = set()

        while stack:
            current = stack.pop()
            current.color = PINK
            current.draw()
            pygame.display.flip()
            time.sleep(0.01)

            if current == end:
                return reconstruct_path(current)

            visited.add(current)

            for neighbor in get_neighbors(current, cells):
                if neighbor not in visited and not neighbor.is_obstacle:
                    stack.append(neighbor)
                    neighbor.parent = current

        return None

    # Returns a list of neighboring cells
    def get_neighbors(node, cells):
        # Returns a list of neighboring cells
        neighbors = []

        # Check if cell is within bounds
        if node.x > 0:
            neighbors.append(cells[(node.x - 1, node.y)])
        if node.x < GRID_X - 1:
            neighbors.append(cells[(node.x + 1, node.y)])
        if node.y > 0:
            neighbors.append(cells[(node.x, node.y - 1)])
        if node.y < GRID_Y - 1:
            neighbors.append(cells[(node.x, node.y + 1)])

        return neighbors

    # Reconstructs the path from the end node - passed to the render function
    def reconstruct_path(node):
        path = []
        current = node
        while current.parent is not None:
            if current.is_obstacle:
                print(f"Error: Obstacle at {current.x}, {current.y} found in path")
                break
            path.append(current)
            current = current.parent
        path.append(current)  # Add the start cell
        return path[::-1]
    
    start = cells[gamesettings["start"]]
    end = cells[gamesettings["end"]]
    return dfs(start, end)

def RunfloodFill(cells):

    def floodFill(start, end):
        # Distance from start to every other vertex; initialize with infinity
        dist = {cell: float('inf') for cell in cells.values()}
        dist[start] = 0

        # Priority queue to hold vertices to be processed
        pq = [(0, start.x, start.y, start)]

        while pq:

            current_dist, _, _, current_cell = heapq.heappop(pq)

            # Early exit if we reached the end
            if current_cell == end:
                break

            if current_dist > dist[current_cell]:
                continue

            for neighbor in get_neighbors(current_cell):
                distance = current_dist + 1

                if distance < dist[neighbor]:
                    dist[neighbor] = distance
                    neighbor.parent = current_cell

                    neighbor.color = YELLOW
                    neighbor.draw()
                    pygame.display.flip()
                    time.sleep(0.01)
                    heapq.heappush(pq, (distance, neighbor.x, neighbor.y, neighbor))
                
        # Reconstruct path from end to start
        path = []
        while end:
            path.append(end)
            end = end.parent

        return path[::-1]  # Return reversed path
    
    def get_neighbors(cell):
        neighbors = []
        directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]

        for dx, dy in directions:
            x, y = cell.x + dx, cell.y + dy
            if 0 <= x < GRID_X and 0 <= y < GRID_Y and not cells[(x, y)].isObstacle():
                neighbors.append(cells[(x, y)])

        return neighbors
    
    start = cells[gamesettings["start"]]
    end = cells[gamesettings["end"]]

    return floodFill(start, end)

def RunrapidRandomTree(cells):

    MAX_ITERATIONS = 10000

    def get_random_cell():
        x = random.randint(0, GRID_X - 1)
        y = random.randint(0, GRID_Y - 1)
        return cells[(x, y)]

    def get_neighbors(cell):
        neighbors = []
        directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]

        for dx, dy in directions:
            x, y = cell.x + dx, cell.y + dy
            if 0 <= x < GRID_X and 0 <= y < GRID_Y and not cells[(x, y)].isObstacle():
                neighbors.append(cells[(x, y)])

        return neighbors
    
    def heuristic(a, b):
        # Manhattan distance on a square grid
        return abs(a.x - b.x) + abs(a.y - b.y)


    def is_path_free(start_cell, end_cell):
        # Check if the path between two cells is free of obstacles
        # This can be a simple line-of-sight check or a more complex collision check
         return True

    def rrt(start, end):
        tree = {start: None}  # Dictionary to store the tree structure

        for _ in range(MAX_ITERATIONS):
            rand_cell = get_random_cell()
            nearest_cell = min(tree, key=lambda cell: heuristic(cell, rand_cell))

            #fill cell yellow
            rand_cell.color = YELLOW
            rand_cell.draw()
            pygame.display.flip()
            time.sleep(0.01)

            nearest_cell.color = GREY
            nearest_cell.draw()
            pygame.display.flip()


            if is_path_free(nearest_cell, rand_cell):
                tree[rand_cell] = nearest_cell

                if rand_cell == end:
                    break

        # Reconstruct path from end to start if end is reached
        path = []
        current = end
        while current in tree:
            path.append(current)
            current = tree[current]
        
        return path[::-1]

    start = cells[gamesettings["start"]]
    end = cells[gamesettings["end"]]
    res = rrt(start, end)
    print(res)
    return res



#Init maze with a white background and store all the cells in a dict for easy access
def initMaze(screen):
    #make screen white
    screen.fill(WHITE)
    for x in range(GRID_X):
        for y in range(GRID_Y):
            #create cell object
            cells[(x,y)] = Cell(x, y, screen)

# Draw the grid - this is called after every change to the maze, it replaces the borders of each cell and is purely for visualisation
def draw_grid(screen):

    #draw white grid lines
    for x in range(GRID_X):
        pygame.draw.line(screen, BLACK, (x * GRID_PIXEL_SIZE_X, 0), (x * GRID_PIXEL_SIZE_X, PIXELS_Y))
    for y in range(GRID_Y):
        pygame.draw.line(screen, BLACK, (0, y * GRID_PIXEL_SIZE_Y), (PIXELS_X, y * GRID_PIXEL_SIZE_Y))

# Fill a cell with a colour - Not used as cell objects manage their own colour however this could be used to fill a cell with a pattern
def fill_cell(screen, x, y, color):
    rect = pygame.Rect(x * GRID_PIXEL_SIZE_X, y * GRID_PIXEL_SIZE_Y, GRID_PIXEL_SIZE_X, GRID_PIXEL_SIZE_Y)
    pygame.draw.rect(screen, color, rect)


#Setup
pygame.init()
screen = pygame.display.set_mode((PIXELS_X, PIXELS_Y))
initMaze(screen)
draw_grid(screen)
pygame.display.flip()


#Event loop
while True:
    for event in pygame.event.get():
        #if left click is pressed or dragged, set cell to obstacle
        if pygame.mouse.get_pressed()[0]:
            pos = pygame.mouse.get_pos()
            x = pos[0] // GRID_PIXEL_SIZE_X
            y = pos[1] // GRID_PIXEL_SIZE_Y

            cell = cells[(x,y)]

            #if cell is not empty, do nothing
            if cell.is_start or cell.is_end:
                print(f"Cannot set start or end to obstacle for cell {x}, {y}")
                continue

            cell.set_obstacle()
            draw_grid(screen)
            pygame.display.flip()



        #if right click is pressed or dragged, set cell to empty
        if pygame.mouse.get_pressed()[2]:
            pos = pygame.mouse.get_pos()
            x = pos[0] // GRID_PIXEL_SIZE_X
            y = pos[1] // GRID_PIXEL_SIZE_Y
            cells[(x,y)].set_empty()

            #check if cell is start or end
            if gamesettings["start"] == (x,y):
                gamesettings["start"] = None
            if gamesettings["end"] == (x,y):
                gamesettings["end"] = None
            draw_grid(screen)
            pygame.display.flip()

        #if space is clicked, set start and end points
        if event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
            pos = pygame.mouse.get_pos()
            x = pos[0] // GRID_PIXEL_SIZE_X
            y = pos[1] // GRID_PIXEL_SIZE_Y

            if gamesettings["start"] == None:
                gamesettings["start"] = (x,y)
                cells[(x,y)].set_start()
            elif gamesettings["end"] == None and gamesettings["start"] != (x,y):
                gamesettings["end"] = (x,y)
                cells[(x,y)].set_end()
            draw_grid(screen)
            pygame.display.flip()


        #if enter is pressed, run aStar
        if event.type == pygame.KEYDOWN and event.key == pygame.K_RETURN:
            if gamesettings["start"] == None or gamesettings["end"] == None:
                print("Please set a start and end point")
                continue
            if algo == "A*":
                path = RunAStar(cells)
            elif algo == "Dykstra":
                path = RunDykstra(cells)
            elif algo == "Depth First Search":
                path = depthFirstSearch(cells)
            elif algo == "Flood Fill":
                path = RunfloodFill(cells)
            elif algo == "Rapid Random Tree":
                path = RunrapidRandomTree(cells)
            if path:
                for cell in path:
                    if cell.is_start or cell.is_end:
                        continue
                    else:
                        cell.color = GREY
                        cell.draw()
                        draw_grid(screen)
                        pygame.display.flip()
                        time.sleep(0.001)


            

        #if escape is pressed, reset
        if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
            reset(screen)
            pygame.display.flip()

        #if r is pressed, remove A* path but keep obstacles and start/end
        if event.type == pygame.KEYDOWN and event.key == pygame.K_r:
            for x in range(GRID_X):
                for y in range(GRID_Y):
                    if cells[(x,y)].is_obstacle or cells[(x,y)].is_start or cells[(x,y)].is_end:
                        continue
                    else:
                        cells[(x,y)].set_empty()
            cells[gamesettings["start"]].color = GREEN
            cells[gamesettings["end"]].color = RED
            cells[gamesettings["start"]].draw()
            cells[gamesettings["end"]].draw()
            draw_grid(screen)
            pygame.display.flip()

            usrinput = input("Do you want to run a different algorithm? (y/n): ")
            if usrinput == "y":
                algo = pickAlgo()
            else:
                continue

        #if quit is pressed, quit
        if event.type == pygame.QUIT:
            pygame.quit()
            quit()
