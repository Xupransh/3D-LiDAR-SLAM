import numpy as np
from maze import Maze, Particle, Robot
import bisect
import rospy
from gazebo_msgs.msg import  ModelState
from gazebo_msgs.srv import GetModelState
import shutil
from std_msgs.msg import Float32MultiArray
from scipy.integrate import ode

def vehicle_dynamics(t, vars, vr, delta):
    curr_x = vars[0]
    curr_y = vars[1] 
    curr_theta = vars[2]
    
    dx = vr * np.cos(curr_theta)
    dy = vr * np.sin(curr_theta)
    dtheta = delta
    return [dx,dy,dtheta]

class particleFilter:
    def __init__(self, bob, world, num_particles, sensor_limit, x_start, y_start):
        self.num_particles = num_particles  # The number of particles for the particle filter
        self.sensor_limit = sensor_limit    # The sensor limit of the sensor
        particles = list()
        for i in range(num_particles):
            x = np.random.uniform(0, world.width)
            y = np.random.uniform(0, world.height)
            particles.append(Particle(x = x, y = y, maze = world, sensor_limit = sensor_limit))
        self.particles = particles          # Randomly assign particles at the begining
        self.bob = bob                      # The estimated robot state
        self.world = world                  # The map of the maze
        self.x_start = x_start              # The starting position of the map in the gazebo simulator
        self.y_start = y_start              # The starting position of the map in the gazebo simulator
        self.modelStatePub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
        self.controlSub = rospy.Subscriber("/gem/control", Float32MultiArray, self.__controlHandler, queue_size = 1)
        self.control = []                   # A list of control signal from the vehicle
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

    def weight_gaussian_kernel(self,x1, x2, std = 5000):
        tmp1 = np.array(x1)
        tmp2 = np.array(x2)
        return np.sum(np.exp(-((tmp2-tmp1) ** 2) / (2 * std)))

    def updateWeight(self, readings_robot):
        """
        Description:
            Update the weight of each particles according to the sensor reading from the robot 
        Input:
            readings_robot: List, contains the distance between robot and wall in [front, right, rear, left] direction.
        """

        ## TODO #####


        ###############
        pass

    def resampleParticle(self):
        """
        Description:
            Perform resample to get a new list of particles 
        """
        particles_new = list()

        ## TODO #####


        ###############

        self.particles = particles_new

    def particleMotionModel(self):
        """
        Description:
            Estimate the next state for each particle according to the control input from actual robot 
        """
        ## TODO #####


        ###############
        pass


    def runFilter(self):
        """
        Description:
            Run PF localization
        """
        while True:
            ## TODO #####
            # Finish this function to have the particle filter running
            
            # Read sensor msg
            
            # Display robot and particles on map 
            self.world.show_particles(particles = self.particles, show_frequency = 10)
            self.world.show_robot(robot = self.bob)
            [est_x,est_y] = self.world.show_estimated_location(particles = self.particles)
            self.world.clear_objects()

            ###############
