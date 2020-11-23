import numpy as np 
import turtle
import bisect
import argparse
from scipy.integrate import ode
import rospy
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
from ackermann_msgs.msg import AckermannDrive
from lidarProcessing import LidarProcessing

class Maze(object):

    def __init__(self, maze = None, x_start = None, y_start = None, num_rows = None, num_cols = None, wall_prob = None, random_seed = None):
        '''
        maze: 2D numpy array. 
        passages are coded as a 4-bit number, with a bit value taking 
        0 if there is a wall and 1 if there is no wall. 
        The 1s register corresponds with a square's top edge, 
        2s register the right edge,
        4s register the bottom edge, 
        and 8s register the left edge. 
        (numpy array)
        '''
        self.maze = maze
        self.num_rows = maze.shape[0]
        self.num_cols = maze.shape[1]
        self.fix_maze_boundary()
        # self.fix_wall_inconsistency()
 
        self.height = self.num_rows
        self.width = self.num_cols
        self.x_start = x_start
        self.y_start = y_start

        self.turtle_registration()

    def turtle_registration(self):
        turtle.register_shape('tri', ((-3, -2), (0, 3), (3, -2), (0, 0)))

    def fix_maze_boundary(self):
        '''
        Make sure that the maze is bounded.
        '''
        for i in range(self.num_rows):
            self.maze[i,0] |= 8
            self.maze[i,-1] |= 2
        for j in range(self.num_cols):
            self.maze[0,j] |= 1
            self.maze[-1,j] |= 4

    def permissibilities(self, cell):
        '''
        Check if the directions of a given cell are permissible.
        Return:
        (up, right, down, left)
        '''
        cell_value = self.maze[cell[0], cell[1]]
        return (cell_value & 1 == 0, cell_value & 2 == 0, cell_value & 4 == 0, cell_value & 8 == 0)

    def colide_wall(self, y, x):
        '''
        Check if given position collide into a wall.
        Return:
        True if collide into a wall, else False
        '''

        if x >= 120 or x < 0 or y >= 75 or y < 0:
            return True
        cell_value = self.maze[y,x]
        if cell_value == 15:
            return True
        else:
            return False

    def show_maze(self):
        '''
        Display the maze
        '''

        turtle.setworldcoordinates(0, 0, self.width * 1.005, self.height * 1.005)

        wally = turtle.Turtle()
        wally.speed(0)
        wally.width(1.5)
        wally.hideturtle()
        turtle.tracer(0, 0)

        for i in range(self.num_rows):
            for j in range(self.num_cols):
                permissibilities = self.permissibilities(cell = (i,j))
                turtle.up()
                wally.setposition((j, i))
                # Set turtle heading orientation
                # 0 - east, 90 - north, 180 - west, 270 - south
                wally.setheading(0)
                if not permissibilities[0]:
                    wally.down()
                else:
                    wally.up()
                wally.forward(1)
                wally.setheading(90)
                wally.up()
                if not permissibilities[1]:
                    wally.down()
                else:
                    wally.up()
                wally.forward(1)
                wally.setheading(180)
                wally.up()
                if not permissibilities[2]:
                    wally.down()
                else:
                    wally.up()
                wally.forward(1)
                wally.setheading(270)
                wally.up()
                if not permissibilities[3]:
                    wally.down()
                else:
                    wally.up()
                wally.forward(1)
                wally.up()

        turtle.update()


    def weight_to_color(self, weight):

        return '#%02x00%02x' % (int(weight * 255), int((1 - weight) * 255))


    def show_particles(self, particles, show_frequency = 10):

        turtle.shape('tri')

        for i, particle in enumerate(particles):
            if i % show_frequency == 0:
                turtle.setposition((particle.x, particle.y))
                turtle.setheading((particle.heading*180/np.pi)%360)
                turtle.color(self.weight_to_color(particle.weight))
                turtle.stamp()
        
        turtle.update()

    def show_estimated_location(self, particles):
        '''
        Show average weighted mean location of the particles.
        '''

        x_accum = 0
        y_accum = 0
        heading_cos_accum = 0
        heading_sin_accum = 0
        weight_accum = 0

        num_particles = len(particles)

        for particle in particles:

            weight_accum += particle.weight
            x_accum += particle.x * particle.weight
            y_accum += particle.y * particle.weight
            # heading_accum += ((particle.heading*180/np.pi)%360) * particle.weight
            heading_cos_accum += np.cos(particle.heading) * particle.weight
            heading_sin_accum += np.sin(particle.heading) * particle.weight

        if weight_accum == 0:
            return False

        x_estimate = x_accum / weight_accum
        y_estimate = y_accum / weight_accum
        heading_sin_accum = heading_sin_accum / weight_accum
        heading_cos_accum = heading_cos_accum / weight_accum
        heading_estimate = np.arctan2(heading_sin_accum, heading_cos_accum) * 180 / np.pi

        turtle.color('orange')
        turtle.setposition(x_estimate, y_estimate)
        turtle.setheading(heading_estimate)
        turtle.shape('turtle')
        turtle.stamp()
        turtle.update()
        return [x_estimate,y_estimate]

    def show_robot(self, robot):

        turtle.color('green')
        turtle.shape('turtle')
        turtle.shapesize(0.7, 0.7)
        turtle.setposition((robot.x, robot.y))
        turtle.setheading((robot.heading*180/np.pi)%360)
        turtle.stamp()
        turtle.update()

    def clear_objects(self):
        turtle.clearstamps()


    def sensor_model(self, coordinates, sensor_limit, orientation = 0):
        '''
        Measure the distance of coordinates to nearest walls at four directions in vehicle frame.
        Return:
        (up, right, rear, left)
        '''

        x, y = coordinates

        # Measure distance between wall and vehicle in front direction
        pos_x = x
        pos_y = y
        d1 = 0
        dx = np.cos(orientation) * 1 - np.sin(orientation) * 0
        dy = np.sin(orientation) * 1 + np.cos(orientation) * 0
        while not self.colide_wall(round(pos_y),round(pos_x)) and d1 < sensor_limit:
            pos_x = pos_x + dx
            pos_y = pos_y + dy
            d1 += 1

        # Measure distance between wall and vehicle in right direction
        pos_x = x
        pos_y = y
        d2 = 0
        dx = np.cos(orientation-np.pi/2) * 1 - np.sin(orientation-np.pi/2) * 0
        dy = np.sin(orientation-np.pi/2) * 1 + np.cos(orientation-np.pi/2) * 0
        while not self.colide_wall(round(pos_y),round(pos_x)) and d2 < sensor_limit:
            pos_x = pos_x + dx
            pos_y = pos_y + dy
            d2 += 1
        
        # Measure distance between wall and vehicle in rear direction
        pos_x = x 
        pos_y = y
        d3 = 0
        dx = np.cos(orientation-np.pi) * 1 - np.sin(orientation-np.pi) * 0
        dy = np.sin(orientation-np.pi) * 1 + np.cos(orientation-np.pi) * 0
        while not self.colide_wall(round(pos_y),round(pos_x)) and d3 < sensor_limit:
            pos_x = pos_x + dx
            pos_y = pos_y + dy
            d3 += 1

        # Measure distance between wall and vehicle in left direction
        pos_x = x
        pos_y = y
        d4 = 0
        dx = np.cos(orientation+np.pi/2) * 1 - np.sin(orientation+np.pi/2) * 0
        dy = np.sin(orientation+np.pi/2) * 1 + np.cos(orientation+np.pi/2) * 0
        while not self.colide_wall(round(pos_y),round(pos_x)) and d4 < sensor_limit:
            pos_x = pos_x + dx
            pos_y = pos_y + dy
            d4 += 1

        ## TODO: Add 4 additional sensor directions #####
        # Measure distance between wall and vehicle in front right direction
            

        # Measure distance between wall and vehicle in rear right direction
        

        # Measure distance between wall and vehicle in rear left direction
        
        
        # Measure distance between wall and vehicle in rear right direction
        
        
        ###############


        # Return readings from sensor model in front, right, rear, left direction
        return [d1*100, d2*100, d3*100, d4*100]


class Particle(object):

    def __init__(self, x, y, maze, heading = None, weight = 1.0, sensor_limit = None, noisy = False):

        if heading is None:
            heading = np.random.uniform(0,2*np.pi)

        self.x = x              # The x position of particle
        self.y = y              # The y position of particle 
        self.heading = heading  # Orientation of particles 
        self.weight = weight    # Weight of particle
        self.maze = maze        # The map of the maze
        self.sensor_limit = sensor_limit    # The sensing limit of sensor
        
        # Add random noise to the particle at initialization
        if noisy:
            std = 0.05
            self.x = self.add_noise(x = self.x, std = std)
            self.y = self.add_noise(x = self.y, std = std)
            self.heading = self.add_noise(x = self.heading, std = np.pi * 2 * 0.05)

        # Fix invalizd particles outside the map
        self.fix_invalid_particles()


    def fix_invalid_particles(self):

        # Fix invalid particles
        if self.x < 0:
            self.x = 0
        if self.x > self.maze.width:
            self.x = self.maze.width * 0.9999
        if self.y < 0:
            self.y = 0
        if self.y > self.maze.height:
            self.y = self.maze.height * 0.9999
        self.heading = self.heading % (np.pi*2)

    @property
    def state(self):
        return (self.x, self.y, self.heading)

    def add_noise(self, x, std):
        return x + np.random.normal(0, std)

    def read_sensor(self):
        """
        Description:
        Get the sensor reading through sensor model for each particle
        """
        readings = self.maze.sensor_model(coordinates = (self.x, self.y), orientation = self.heading, sensor_limit = self.sensor_limit)

        return readings

    def try_move(self, offset, maze, noisy = False):        
        curr_theta = self.heading
        curr_x = self.x
        curr_y = self.y
        dx = offset[0]
        dy = offset[1]
        dtheta = offset[2]
        dpos = np.sqrt(dx**2+dy**2)
        dtheta_world = offset[3] + curr_theta
        self.heading = (curr_theta+dtheta)%(np.pi*2) 
        x = dpos*np.cos(dtheta_world)+curr_x
        y = dpos*np.sin(dtheta_world)+curr_y

        gj1 = int(self.x)
        gi1 = int(self.y)
        gj2 = int(x)
        gi2 = int(y)

        # Check if the particle is still in the maze
        if gi2 < 0 or gi2 >= maze.num_rows or gj2 < 0 or gj2 >= maze.num_cols:
            self.x = np.random.uniform(0, maze.width)
            self.y = np.random.uniform(0, maze.height)
            return False

        self.x = x
        self.y = y
        return True

class Robot(Particle):
    def __init__(self, x, y, maze, heading = None, sensor_limit = None, noisy = True):

        super(Robot, self).__init__(x = x, y = y, maze = maze, heading = heading, sensor_limit = sensor_limit, noisy = noisy)
        # The actual Lidar mounted on the vehicle
        self.lidar = LidarProcessing(resolution = 0.1, 
            side_range = (-sensor_limit, sensor_limit), fwd_range = (-sensor_limit, sensor_limit),
            height_range = (-1.5, 0.5))
       
    def getModelState(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            modelState = serviceResponse(model_name='gem')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: "+str(exc))
        return modelState

    def read_sensor(self):
        # Read 
        curr_state = self.getModelState()
        readings_tmp = self.lidar.processLidar()
        x = curr_state.pose.position.x
        y = curr_state.pose.position.y
        euler = self.quaternion_to_euler(curr_state.pose.orientation.x,
                                    curr_state.pose.orientation.y,
                                    curr_state.pose.orientation.z,
                                    curr_state.pose.orientation.w)
        self.heading = euler[2] % (2*np.pi)
        self.x = (x+100-self.maze.x_start)
        self.y = (y+100-self.maze.y_start)
        return readings_tmp

    def quaternion_to_euler(self, x, y, z, w):

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = np.arcsin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)
        return [roll, pitch, yaw]
