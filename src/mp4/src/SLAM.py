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
        self.map = np.zeros((width, height, 3)) # RGB image for trajectory/obstacle mapping
        self.x_start = x_start              # The starting position of the map in the gazebo simulator
        self.y_start = y_start
        self.original_heading = heading             # The starting position of the map in the gazebo simulator
        self.modelStatePub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
        self.controlSub = rospy.Subscriber("/gem/control", Float32MultiArray, self.__controlHandler, queue_size = 1)
        self.birdsEyeViewPub = rospy.Publisher("/slam/map", self.map, queue_size=1)
        self.pointCloudSub = rospy.Subscriber("/velodyne_points", PointCloud2, self.__pointCloudHandler, queue_size=10)
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
    
    def updateMap():
        x_points,y_points = self.robot.lidar.getCurrentPoints()
        
        x_pos = self.robot.x
        y_pos = self.robot.y

        self.map[x_pos,y_pos]= [0,255,0]

        for i range(length):
            x = np.floor(x_points[i]*np.cos(self.robot.heading))
            y = np.floor(y_points[i]*np.sin(self.robot.heading))

            x += self.width/2 + x_pos
            y += self.height/2 +y_pos

            self.map[x,y]= [255,255,255]
            cv2.imshow(self.map)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

    def runSLAM(self):
        """
        Description:
            Run LiDAR SLAM
        """
        while True:
            ## TODO #####
            
            # Display robot and update and show map
            # self.world.show_particles(particles = self.particles, show_frequency = 10)
            # self.world.show_robot(robot = self.robot)
            # self.world.clear_objects()
            self.updateMap()

            ###############
