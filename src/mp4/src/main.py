import numpy as np 
import turtle
import argparse
import time
import pickle
import rospy
from gazebo_msgs.msg import  ModelState
from gazebo_msgs.srv import GetModelState
from maze import Maze, Robot
from particle_filter import particleFilter
from SLAM import SLAM

def main(window_width, window_height):
    rospy.init_node("navigator")

    window = turtle.Screen()
    window.setup (width = window_width, height = window_height)

    # Creating the python map for the ECEB environment
    maze = np.zeros((200,200))
    with open('obstacle_list.data', 'rb') as filehandle:
        # read the data as binary data stream
        obstacle = pickle.load(filehandle)
    
    # Create blank SLAM map
    map = np.zeros((200,200,3))

    for (x,y) in obstacle:
        maze[y+100,x+100] = 1

    y_start = 100
    x_start = 15
    width = 120
    height = 75
    maze_ted = np.zeros((height,width),dtype = np.int8)
    for i in range(y_start,y_start+height):
        for j in range(x_start,x_start+width):
            if maze[i,j] == 1:
                    maze_ted[i-y_start,j-x_start] |= 15
            else:
                if(i == 0):
                    maze_ted[i-y_start,j-x_start] |= 1
                elif(i == maze.shape[1]-1):
                    maze_ted[i-y_start,j-x_start] |= 4
                else:
                    if maze[i+1,j] == 1:
                        maze_ted[i-y_start,j-x_start] |= 4
                    if maze[i-1,j] == 1:
                        maze_ted[i-y_start,j-x_start] |= 1
                
                if(j == 0):
                    maze_ted[i-y_start,j-x_start] |= 8
                elif(j == maze.shape[1]-1):
                    maze_ted[i-y_start,j-x_start] |= 2
                else:
                    if maze[i,j+1] == 1:
                        maze_ted[i-y_start,j-x_start] |= 2
                    if maze[i,j-1] == 1:
                        maze_ted[i-y_start,j-x_start] |= 8
    world = Maze(maze = maze_ted, x_start = x_start, y_start = y_start)
    time.sleep(1)
    world.show_maze()

    bob = Robot(x = 0, y = 0,heading = 0, maze = world, sensor_limit = 20)
    
    # Run SLAM
    slam = SLAM( robot=bob, width = window_width, height = window_height, x_start = bob.x + window_width/2, y_start = bob.y + window_height/2, heading = bob.heading)
    slam.runSLAM()
    
    

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description = 'SLAM in maze.')

    # The size of the python map
    window_width_default = 1200
    window_height_default = 750

    # Default values for the parameters for particle filter
    
    parser.add_argument('--w', type = int, help = 'Map width.', default = window_width_default)
    parser.add_argument('--h', type = int, help = 'Map height', default = window_height_default)
    
    argv = parser.parse_args()

    window_width = argv.w
    window_height = argv.h
    
    main(window_width = window_width, window_height = window_height)
