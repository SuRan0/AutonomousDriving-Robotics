"""
Breadth-First Search

Uninformed Search
FIFO, queue
Time Complexity: O(b^h)
Space Complexity: O(b^h)
Completeness: Yes
Optimality: Only if costs are all 1
"""

import math
import matplotlib.pyplot as plt

show_animation = True

class BreadthFirstSearchPlanner:
    
    def __init__(self, ox, oy, resolution, robot_radius) -> None:
        """
        Initialization

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        robot_radius: [m]
        """

        self.resolution = resolution
        self.robot_radius = robot_radius
        self.calc_obstacle_map(ox, oy)
        self.motion = self.get_motion_model()
    
    class Node:
        def __init__(self, x, y, cost, parent_index, parent) -> None:
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_index = parent_index
            self.parent = parent
        
        def __str__(self) -> str:
            return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.parent_index)

    def calc_final_path(self, goal_node, closed_set):
        # generat final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [self.calc_grid_position(goal_node.y, self.min_y)]
        n = closed_set[goal_node.parent_index]

        while n is not None:
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            n = n.parent
        
        return rx, ry
    
    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)
    
    def calc_grid_position(self, index, min_pos):
        """
        calculate grid real position
        """
        return index * self.resolution + min_pos
    
    def calc_obstacle_map(self, ox, oy):
        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)] for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.robot_radius:
                        self.obstacle_map[ix][iy] = True
                        break
    
    def calc_xy_index(self, position, min_pos):
        return round((position -  min_pos) / self.resolution)

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0 ,1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]
        
        return motion

    def planning(self, sx, sy, gx, gy):
        """
        Breadth-First Search based planning

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1, None)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1, None)
        
        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while True:
            if len(open_set) == 0:
                print("Open set is empty ...")
                break

            current = open_set.pop(list(open_set.keys())[0])
            c_id = self.calc_grid_index(current)

            # closed set only the mask node
            closed_set[c_id] = current

            # show graph
            if show_animation:
                plt.plot(self.calc_grid_position(current.x, self.min_x),
                         self.calc_grid_position(current.y, self.min_y), "xc")
                # for stopping simulation with the esc key
                plt.gcf().canvas.mpl_connect('key_release_event', lambda event: [exit(0) if event.key == 'escape' else None])
                
                plt.pause(0.01)
            
            if current.x == goal_node.x and current.y == goal_node.y:
                print("Reach Final Goal!")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # expand_grid search, grid_based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id, None)
                n_id = self.calc_grid_index(node)

                # if the node is not safe, do something, and skip the iteration
                if not self.verify_node(node):
                    continue
                
                if (n_id not in closed_set) and (n_id not in open_set):
                    node.parent = current
                    open_set[n_id] = node
        rx, ry = self.calc_final_path(goal_node, closed_set)
        return rx, ry

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collisition check
        if self.obstacle_map[node.x][node.y]:
            return False
        
        return True

def main():
    print(__file__ + " Start!!")

    # start and goal position
    sx = 10.0   # [m]
    sy = 10.0   # [m]
    gx = 50.0   # [m]
    gy = 50.0   # [m]
    grid_size = 2.0 # [m]
    robot_radius = 1.0  # [m]

    # set obstacle positions
    ox, oy = [], []
    for i in range(-10, 60):
        ox.append(i)
        oy.append(-10.0)
    for i in range(-10, 60):
        ox.append(60.0)
        oy.append(i)
    for i in range(-10, 61):
        ox.append(i)
        oy.append(60.0)
    for i in range(-10, 61):
        ox.append(-10.0)
        oy.append(i)
    for i in range(-10, 40):
        ox.append(20.0)
        oy.append(i)
    for i in range(0, 40):
        ox.append(40.0)
        oy.append(60.0 - i)
    
    if show_animation:
        plt.plot(ox, oy, ".k")  # dot, black
        plt.plot(sx, sy, "og")  # big dot, green
        plt.plot(gx, gy, "xb")  # cross, blue
        plt.grid(True)
        plt.axis("equal")

    bfs = BreadthFirstSearchPlanner(ox, oy, grid_size, robot_radius)
    rx, ry = bfs.planning(sx, sy, gx, gy)

    if show_animation:
        plt.plot(rx, ry, "-r")  # line red
        plt.pause(0.01)
        plt.show()

if __name__ == '__main__':
    main()