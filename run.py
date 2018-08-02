
# noinspection PyUnresolvedReferences
from HamsterAPI.comm_ble import RobotComm
import Tkinter as tk
import sys
import time
import math
import threading

class Grid(object):
    def __init__(self):
        self.success_paths = []
        self.all_paths = []
    def get_success_paths(self):
        return self.success_paths
    def get_all_paths(self):
        return self.all_paths

class Node(object):
    def __init__(self, x, y, blocks, grid_map, grid, path=None, end=None):
        if path is None:
            path = []
        self.x = x
        self.y = y
        self.node = (self.x, self.y)
        self.blocks = blocks  # set of all the blocks in the map (black list)
        self.grid_map = grid_map  # set of all possible nodes you can get to, including blocks (white list)
        self.end = end
        self.grid = grid

        self.path = path  # a list of nodes path it come from (without this node)
        self.next_nodes = set()  # a set of connected nodes from this node

        # removing black list
        temp_next_nodes = {(self.x + 1, self.y + 0), (self.x - 1, self.y + 0), (self.x + 0, self.y + 1),
                           (self.x + 0, self.y - 1)}
        self.next_nodes = temp_next_nodes - self.blocks

        # removing white list
        temp_nodes_out_of_map = set()
        for next_node in self.next_nodes:
            if next_node not in self.grid_map:
                temp_nodes_out_of_map.add(next_node)
        self.next_nodes = self.next_nodes - temp_nodes_out_of_map

    def get_node(self):
        return self.node

    def get_next_nodes(self):
        return self.next_nodes

    def get_next_explore(self):
        return self.get_next_nodes() - set(self.get_path()) - set(self.get_node())

    def get_map(self):
        return self.grid_map

    def get_blocks(self):
        return self.blocks

    def get_path(self):
        return self.path

    def get_end(self):
        return self.end

    def get_grid(self):
        return self.grid

    def run(self):
        # print str(self.get_node()) + " -> " + str(self.get_next_explore()) + " path = " + str(self.get_path())
        if self.get_end() == self.get_node():
            path = self.path + [self.node]
            self.get_grid().success_paths.append(path)
            print "FIND THE PATH:", str(path)
        if len(self.get_next_explore()) is not 0:
            for next_node in self.get_next_explore():
                # print "trying to append", str(self.get_path()), "with", str(self.get_node())
                next_path = self.get_path() + [self.get_node()]  # BUG: inplace bug, be careful because .append() does not create a new list
                node = Node(x=next_node[0], y=next_node[1], blocks=self.get_blocks(), grid_map=self.get_map(), grid=self.get_grid(), path=next_path, end=self.get_end())
                node.run()
                # return path  # catch ending return
        else:
            path = self.path + [self.node]
            self.get_grid().all_paths.append(path)
            # print "PATH FINISHED:", str(path)
            # return path  # ending return in each path

class RobotBehaviorThread(threading.Thread):
    def __init__(self, robotList, path, orientation):
        self.current_node = path[0]
        self.current_task = -1
        self.orientation = orientation
        # self.gui_handle = gui_handle
        # self.go = False
        # self.done = False
        super(RobotBehaviorThread, self).__init__()
        self.robot_list = robotList
        self.robot = robotList[0]
        # self.meet_unexpected_obstacle = False
        # self.unexpected_obstacle_node = None
        self.path = path

    def run(self):
        for i, node in enumerate(self.path):
            # if self.meet_unexpected_obstacle:
            #     # start re-plan the path
            #     global global_start
            #     global_start = self.current_node
            #     global global_end
            #     global_end = global_end
            #     print("Unexpected: " + str(self.unexpected_obstacle_node))
            #     global global_blocks
            #     global_blocks = global_blocks.union(set([self.unexpected_obstacle_node]))
            #
            #     grid_map, all_paths, success_paths, best_path = find_best_path(global_rows, global_cols, global_start, global_end, global_blocks)
            #     self.gui_handle.refresh(global_blocks, grid_map, all_paths, success_paths, global_start, global_end, best_path)
            #     behaviors = RobotBehaviorThread(self.robot_list, best_path, self.orientation, self.gui_handle)
            #     behaviors.setDaemon(True)
            #     behaviors.start()
            #     break  # end thread
            # self.current_node = node
            #############################################
            #############################################

            self.current_task = i
            if i != 0:
                from_node_x = self.path[i-1][0]
                from_node_y = self.path[i-1][1]
                to_node_x = node[0]
                to_node_y = node[1]
                d_x = to_node_x - from_node_x
                d_y = to_node_y - from_node_y
                print "I want to move this direction: (" + str(d_x) + ", " + str(d_y) + ")"
                # robot should only move 1 block at a time
                # I love turning left more than turning right
                if d_x == 1:
                    if self.orientation != 0:
                        if self.orientation >= 180:
                            while self.orientation != 0:
                                self.turn_left()
                        if self.orientation < 180:
                            while self.orientation != 0:
                                self.turn_right()
                    self.go_front()
                elif d_x == -1:
                    if self.orientation != 180:
                        if self.orientation >= 0:
                            while self.orientation != 180:
                                self.turn_left()
                        if self.orientation < 0:
                            while self.orientation != 180:
                                self.turn_right()
                        self.go_front()
                if d_y == 1:
                    if self.orientation != 270:
                        if self.orientation >= 90:
                            while self.orientation != 270:
                                self.turn_left()
                        if self.orientation < 90:
                            while self.orientation != 270:
                                self.turn_right()
                        self.go_front()
                elif d_y == -1:
                    if self.orientation != 90:
                        if self.orientation >= 270:
                            while self.orientation != 90:
                                self.turn_left()
                        if self.orientation < 270:
                            while self.orientation != 90:
                                self.turn_right()
                        self.go_front()
        self.current_node = self.path[len(self.path) - 1]
        self.robot.set_musical_note(40)
        time.sleep(0.2)
        self.robot.set_musical_note(41)
        time.sleep(0.2)
        self.robot.set_musical_note(42)
        time.sleep(0.2)
        self.robot.set_musical_note(0)

    def add_orientation(self, degree):
        self.orientation = self.orientation + degree
        if self.orientation < 0:
            self.orientation = self.orientation + 360
        elif self.orientation >= 360:
            self.orientation = self.orientation - 360

    def go_front(self):
        while self.robot:
            # check if obstacle in front
            prox_l = None
            prox_r = None
            # make sure that the prox is accurate
            if 50 > self.robot.get_proximity(0) > 40 and 50 > self.robot.get_proximity(1) > 40:  # threshold
                prox_l = self.robot.get_proximity(0)
                prox_r = self.robot.get_proximity(1)
            # if prox_l and prox_r:
            #     # obstacle detection
            #     self.meet_unexpected_obstacle = True
            #     self.unexpected_obstacle_node = self.path[self.current_task + 1]
            #
            #     # notice
            #     self.robot.set_musical_note(40)
            #     time.sleep(0.2)
            #     self.robot.set_musical_note(40)
            #     time.sleep(0.2)
            #     self.robot.set_musical_note(40)
            #     time.sleep(0.2)
            #     self.robot.set_musical_note(0)
            #     break  # stop going front

            left_black = self.robot.get_floor(0) < 50
            right_black = self.robot.get_floor(1) < 50
            self.robot.set_musical_note(0)

            if left_black == right_black:
                if left_black:
                    self.robot.set_musical_note(40)
                    self.robot.set_wheel(0, 20)
                    self.robot.set_wheel(1, 20)
                    time.sleep(0.5)
                    self.robot.set_wheel(0, 0)
                    self.robot.set_wheel(1, 0)
                    self.robot.set_musical_note(0)
                    break
                self.robot.set_wheel(0, 20)
                self.robot.set_wheel(1, 20)
            elif left_black == True and right_black == False:  # turn left
                self.robot.set_wheel(0, -20)
                self.robot.set_wheel(1, 20)
            elif left_black == False and right_black == True:  # turn right
                self.robot.set_wheel(0, 20)
                self.robot.set_wheel(1, -20)

            time.sleep(0.001)  # time refresh
        time.sleep(1)  # time sleep when get to node
    def turn_left(self):
        self.robot.set_wheel(0, -50)
        self.robot.set_wheel(1, 50)
        time.sleep(0.29850746268656716417910447761194 *2)  # 25 circle + 45 degree = 9045 degree. sleep(1)=150.75 degree, 45 degree = 0.29850746268656716417910447761194
        self.add_orientation(90)
        self.robot.set_wheel(0, 0)
        self.robot.set_wheel(1, 0)
    def turn_right(self):
        self.robot.set_wheel(0, 50)
        self.robot.set_wheel(1, -50)
        time.sleep(0.29850746268656716417910447761194 *2)  # 25 circle + 45 degree = 9045 degree. sleep(1)=150.75 degree, 45 degree = 0.29850746268656716417910447761194
        self.add_orientation(-90)
        self.robot.set_wheel(0, 0)
        self.robot.set_wheel(1, 0)

def find_best_path(rows, cols, start, end, blocks):
    grid_map = set()
    for row in range(rows + 1):  # BUG: remember to add 1
        for col in range(cols + 1):  # BUG: remember to add 1
            grid_map.add((row, col))
    grid = Grid()
    node = Node(x=start[0], y=start[1], blocks=blocks, grid_map=grid_map, grid=grid, path=[], end=end)
    node.run()

    all_paths = grid.get_all_paths()
    for path in all_paths:
        length = len(path)
        # print "ALL_PATH: len=", str(length), str(path)

    best_path = None
    best_len = 100
    success_paths = grid.get_success_paths()
    for path in success_paths:
        length = len(path)
        if length < best_len:
            best_len = length
            best_path = path
            # print "UPDATE_BEST: len=", str(length), str(path)
        # print "SUCCESS_PATH: len=", str(length), str(path)
    print "OUR CHAIMPION IS:", str(best_path)
    return grid_map, all_paths, success_paths, best_path

# ===== Configuration =====
global_rows = 4
global_cols = 3
global_start = (0, 0)
global_end = (4, 0)
global_blocks = {(1, 1), (3, 0), (2, 2)}
global_orientation = 90
# ===== Configuration =====

def main():
    # instantiate COMM object
    gMaxRobotNum = 1  # max number of robots to control
    comm = RobotComm(gMaxRobotNum)
    comm.start()
    print 'Bluetooth starts'
    robotList = comm.robotList

    # settings


    # grid_map, all_paths, success_paths, best_path = find_best_path(global_rows, global_cols, global_start, global_end, global_blocks)

    # root = tk.Tk()
    # gui_handle.draw_virtual_world()  # this method runs in main thread
    # gui_handle = GridGraphDisplay(frame=root, blocks=global_blocks, grid_map=grid_map, all_path=all_paths, success_paths=success_paths, start=global_start, end=global_end, best_path=best_path)

    import pickle
    list_of_coords = None
    while(list_of_coords == None):
        try:
            outfile = '/Users/admin/Documents/gym/gym/projects/data.txt'
            with open(outfile, 'rb') as fp:
                list_of_coords = pickle.load(fp)
        except Exception as e:
            pass
        time.sleep(1)

    behaviors = RobotBehaviorThread(robotList, list_of_coords, global_orientation)
    behaviors.setDaemon(False)
    behaviors.start()

    # root.mainloop()


    return


class GridGraphDisplay(object):
    def __init__(self, frame, blocks, grid_map, all_path, success_paths, start, end, best_path):
        # default settings
        self.node_distance = 60
        self.node_width = 20
        self.canvas = None
        self.scale = 100
        self.shift_x = 100
        self.shift_y = 100

        # getting info
        self.gui_root = frame
        self.blocks = blocks
        self.grid_map = grid_map
        self.all_path = all_path
        self.success_path = success_paths
        self.start = start
        self.end = end
        self.best_path = best_path

        # graphic info
        self.drawn_blocks = []
        self.nodes = []
        self.lines = []
        self.highlights = []
        self.drawn_start = None
        self.end = None

        self.gui_root.title("Hamster Simulator")
        self.canvas = tk.Canvas(self.gui_root, bg="white", width=440 * 2, height=330 * 2)
        self.canvas.pack()

        self.refresh(blocks, grid_map, all_path, success_paths, start, end, best_path)
        # self.graph = graph
        # self.nodes_location = graph.node_display_locations
        # self.start_node = graph.startNode
        # self.goal_node = graph.goalNode
        return

    def refresh(self, blocks, grid_map, all_path, success_paths, start, end, best_path):
        self.canvas.delete("all")
        # getting info
        self.blocks = blocks
        self.grid_map = grid_map
        self.all_path = all_path
        self.success_path = success_paths
        self.start = start
        self.end = end
        self.best_path = best_path
        self.display_graph()

    # draws nodes and edges in a graph
    def display_graph(self):
        # start canvas

        # draw lines
        for path in self.all_path:
            path_len = len(path)
            # path[0] -> path[path_len-1]
            for i in range(path_len-1):
                node1 = path[i]
                node2 = path[i+1]
                self.draw_lines(node1, node2)

        # draw nodes
        for node in self.grid_map:
            self.draw_node(node)

        # highlight
        self.highlight_path(self.best_path)

        # draw block
        for block in self.blocks:
            self.draw_block(block)

        # draw start
        self.draw_start(self.start)

    # path is a list of nodes ordered from start to goal node
    def highlight_path(self, path):
        for node in path:
            x = node[0] * self.scale + self.shift_x
            y = node[1] * self.scale + self.shift_y
            self.highlights.append(self.create_circle(x, y, width=self.node_width, fill="red", outline="red"))


    # draws a node in given color. The node location info is in passed-in node object
    def draw_node(self, node):
        x = node[0] * self.scale + self.shift_x
        y = node[1] * self.scale + self.shift_y
        self.nodes.append(self.create_circle(x, y, width=self.node_width, fill="blue", outline="blue"))

    # draws an line segment, between two given nodes, in given color
    def draw_lines(self, node1, node2):
        x1 = node1[0] * self.scale + self.shift_x
        y1 = node1[1] * self.scale + self.shift_y
        x2 = node2[0] * self.scale + self.shift_x
        y2 = node2[1] * self.scale + self.shift_y
        self.lines.append(self.canvas.create_line(x1, y1, x2, y2, fill="black"))

    def draw_block(self, block):
        x = block[0] * self.scale + self.shift_x
        y = block[1] * self.scale + self.shift_y
        self.drawn_blocks.append(self.create_circle(x, y, width=self.node_width, fill="white", outline="white"))

    def create_circle(self, x, y, width, fill, outline):
        return self.canvas.create_oval(x-width/2, y-width/2, x+width/2, y+width/2, width=width, fill=fill, outline=outline)

    def draw_start(self, start):
        x = start[0] * self.scale + self.shift_x
        y = start[1] * self.scale + self.shift_y
        self.drawn_start = self.create_circle(x, y, width=self.node_width, fill="yellow", outline="yellow")

if __name__ == "__main__":
    main()