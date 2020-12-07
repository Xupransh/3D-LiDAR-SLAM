import numpy as np
from maze import Maze, Particle, Robot
import bisect
import rospy
from gazebo_msgs.msg import  ModelState
from gazebo_msgs.srv import GetModelState
import shutil
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud2
from scipy.integrate import ode
import cv2
import matplotlib.pyplot as plt
from maze import Maze, Robot

def vehicle_dynamics(t, vars, vr, delta):
    curr_x = vars[0]
    curr_y = vars[1] 
    curr_theta = vars[2]
    
    dx = vr * np.cos(curr_theta)
    dy = vr * np.sin(curr_theta)
    dtheta = delta
    return [dx,dy,dtheta]

class SLAM:
    def __init__(self, robot, width, height, x_start, y_start, heading):
        self.robot = robot
        self.width = width
        self.height = height
        self.map = np.zeros((width, height), np.uint8) # RGB image for trajectory/obstacle mapping
        self.x_start = x_start              # The starting position of the map in the gazebo simulator
        self.y_start = y_start
        self.original_heading = heading             # The starting position of the map in the gazebo simulator
        #self.modelStatePub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
        #self.controlSub = rospy.Subscriber("/gem/control", Float32MultiArray, self.__controlHandler, queue_size = 1)
        #self.birdsEyeViewPub = rospy.Publisher("/slam/map", self.map, queue_size=1)
        #self.pointCloudSub = rospy.Subscriber("/velodyne_points", PointCloud2, self.__pointCloudHandler, queue_size=10)
        self.control = []                 # A list of control signal from the vehicle
        
        return
    
    def __controlHandler(self,data):
        """
        Description:
            Subscriber callback for /gem/control. Store control input from gem controller to be used in particleMotionModel.
        """
        tmp = list(data.data)
        self.control.append(tmp)

    def getModelState(self):
        """
        Description:
            Requests the current state of the polaris model when called
        Returns:
            modelState: contains the current model state of the polaris vehicle in gazebo
        """

        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            modelState = serviceResponse(model_name='polaris')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: "+str(exc))
        return modelState

    # def currentPoseToArray(self, currentPose):
    #     position = currentPose.pose.position
    #     velocity = currentPose.twist.linear
    #     orientation = currentPose.pose.orientation
    #     roll, pitch, yaw = quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)
        
    #     return [position.x, position.y, yaw, velocity.x]
       
    def updateMap(self, state):
        print(state)
        x_points,y_points = self.robot.lidar.getCurrentPoints()
        curr_lidar = np.zeros((1000,1000),np.uint8)
        x_pos = self.robot.x
        y_pos = self.robot.y


        length =  len(x_points)
        for i in range(0, length):
            #print("loop", i)
            theta = np.degrees(self.robot.heading)
            
            rot = np.array([[np.cos(theta), -np.sin(theta),0],[np.sin(theta), np.cos(theta), 0],[0,0,1]])
            curr_i = np.array([x_points[i], y_points[i], 0])
            curr = np.transpose(curr_i)
            trans = np.transpose(np.matmul(rot,curr))

            x = int(x_pos*trans[0])
            y = int(y_pos*trans[1])

            
            
 
            #curr_lidar[x_points[i]*10+200,y_points[i]*10+200]= 255
            self.map[x,y]= 255
        #cv2.imshow("lidar",curr_lidar)
        cv2.imshow("map",self.map)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


            ###############
