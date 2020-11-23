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

def main(window_width, window_height, num_particles, sensor_limit):
    rospy.init_node("navigator")

    window = turtle.Screen()
    window.setup (width = window_width, height = window_height)

    # Creating the python map for the ECEB environment
    maze = np.zeros((200,200))
    with open('obstacle_list.data', 'rb') as filehandle:
        # read the data as binary data stream
        obstacle = pickle.load(filehandle)

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

    bob = Robot(x = 0, y = 0,heading = 0, maze = world, sensor_limit = sensor_limit)
 
    # Run PF localization
    pf = particleFilter(bob = bob, world = world, num_particles = num_particles, sensor_limit = sensor_limit,
                        x_start = x_start, y_start = y_start)
    
    pf.runFilter()

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description = 'Particle filter in maze.')

    # The size of the python map
    window_width_default = 1200
    window_height_default = 750

    # Default values for the parameters for particle filter
    num_particles_default = 1000
    sensor_limit_default = 15
    
    parser.add_argument('--num_particles', type = int, help = 'Number of particles used in particle filter.', default = num_particles_default)
    parser.add_argument('--sensor_limit', type = float, help = 'The distance in Gazebo the sensor can sense. ', default = sensor_limit_default)
    
    argv = parser.parse_args()

    window_width = window_width_default
    window_height = window_height_default
    num_particles = argv.num_particles
    sensor_limit = argv.sensor_limit
    
    main(window_width = window_width, window_height = window_height, num_particles = num_particles, sensor_limit = sensor_limit)