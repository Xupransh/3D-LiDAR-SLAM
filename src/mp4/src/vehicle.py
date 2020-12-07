import rospy
from gazebo_msgs.msg import  ModelState
from controller import bicycleModel
import time
from SLAM import SLAM
from maze import Maze, Robot
import argparse
import numpy as np
if __name__ == "__main__":
    rospy.init_node("model_dynamics")
    model = bicycleModel()

    parser = argparse.ArgumentParser(description = 'SLAM in maze.')

    # The size of the python map
    window_width_default = 3000
    window_height_default = 3000

    # Default values for the parameters for particle filter
    
    parser.add_argument('--w', type = int, help = 'Map width.', default = window_width_default)
    parser.add_argument('--h', type = int, help = 'Map height', default = window_height_default)
    
    argv = parser.parse_args()

    window_width = argv.w
    window_height = argv.h
    maze = np.zeros((200,200))
    endList = 0
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
    bob = Robot(x = 0, y = 0,heading = 0, maze = world, sensor_limit = 20)
    
    # Run SLAM
    currModelState =  model.getModelState()
    euler = model.quaternion_to_euler(currModelState.pose.orientation.x,
                                    currModelState.pose.orientation.y,
                                    currModelState.pose.orientation.z,
                                    currModelState.pose.orientation.w)
    slam = SLAM( robot=bob, width = window_width, height = window_height, x_start = currModelState.pose.position.x , y_start = currModelState.pose.position.y, heading = euler[2])
    pos_list = [[100,53],[80,57],[60,56],[50,57],[40,58],[35,55],[34,44],[40,39],[45,40],[55,40],[68,40],[75,30],[75,28],[83,22],[104,22],[110,34],[102,39],[96,47]]
    pos_idx = 0

    targetState = ModelState()
    targetState.pose.position.x = pos_list[pos_idx][0] + 15 - 100
    targetState.pose.position.y = pos_list[pos_idx][1] + 100 - 100

    start = time.time()
    rate = rospy.Rate(100)  # 100 Hz

    while not rospy.is_shutdown():
        rate.sleep()  # Wait a while before trying to get a new state

        currState =  model.getModelState()
        if not currState.success:
            continue
        slam.updateMap(currState)
    

        distToTargetX = abs(targetState.pose.position.x - currState.pose.position.x)
        distToTargetY = abs(targetState.pose.position.y - currState.pose.position.y)


        if(distToTargetX < 1 and distToTargetY < 1):
            pos_idx = pos_idx+1
            pos_idx = pos_idx % len(pos_list)
            targetState = ModelState()
            targetState.pose.position.x = pos_list[pos_idx][0] + 15 - 100
            targetState.pose.position.y = pos_list[pos_idx][1] + 100 - 100
            print("reached",pos_list[pos_idx][0],pos_list[pos_idx][1])
        else:
            model.setModelState(currState, targetState)

    rospy.spin()
