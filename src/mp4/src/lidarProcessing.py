import math
import numpy as np

import rospy
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

import copy



class LidarProcessing:
    def __init__(self, resolution=0.5, side_range=(-5., 5.), fwd_range=(0., 15.),
                         height_range=(-1, 1)):
        self.resolution = resolution
        self.side_range = side_range
        self.fwd_range = fwd_range
        self.height_range = height_range
        self.sensor_limit = (abs(fwd_range[0]) + abs(fwd_range[1]) + abs(side_range[0]) + abs(side_range[1]))/4

        self.cvBridge = CvBridge()

        # random size initial image
        self.__birds_eye_view =  np.zeros((200, 100))

        self.birdsEyeViewPub = rospy.Publisher("/mp4/BirdsEye", Image, queue_size=1)
        self.pointCloudSub = rospy.Subscriber("/velodyne_points", PointCloud2, self.__pointCloudHandler, queue_size=10)
        x_img = np.floor(-0 / self.resolution).astype(np.int32)
        self.vehicle_x = x_img - int(np.floor(self.side_range[0] / self.resolution))

        y_img = np.floor(-0 / self.resolution).astype(np.int32)
        self.vehicle_y = y_img + int(np.ceil(self.fwd_range[1] / self.resolution))

        self.x_front = float('nan')
        self.y_front = float('nan')

        self.x_rear = float('nan')
        self.y_rear = float('nan')

        self.x_left = float('nan')
        self.y_left = float('nan')

        self.x_right = float('nan')
        self.y_right = float('nan')

        self.x_front_left = float('nan')
        self.y_front_left = float('nan')

        self.x_front_right = float('nan')
        self.y_front_right = float('nan')

        self.x_rear_left = float('nan')
        self.y_rear_left = float('nan')

        self.x_rear_right = float('nan')
        self.y_rear_right = float('nan')

    def getBirdsEyeView(self):
        return self.__birds_eye_view


    def __pointCloudHandler(self, data):
        """
            Callback function for whenever the lidar point clouds are detected

            Input: data - lidar point cloud

            Output: None

            Side Effects: updates the birds eye view image
        """
        gen = point_cloud2.readgen = point_cloud2.read_points(cloud=data, field_names=('x', 'y', 'z', 'ring'))

        lidarPtBV = []
        for p in gen:
            lidarPtBV.append((p[0],p[1],p[2]))

        self.construct_birds_eye_view(lidarPtBV)

    def construct_birds_eye_view(self, data):
        """
            Call back function that get the distance between vehicle and nearest wall in given direction
            The calculated values are stored in the class member variables

            Input: data - lidar point cloud
        """
        # create image from_array
        x_max = 1 + int((self.side_range[1] - self.side_range[0]) / self.resolution)
        y_max = 1 + int((self.fwd_range[1] - self.fwd_range[0]) / self.resolution)
        im = np.zeros([y_max, x_max], dtype=np.uint8)

        if len(data) == 0:
            return im

        # Reference: http://ronny.rest/tutorials/module/pointclouds_01/point_cloud_birdseye/
        data = np.array(data)

        x_points = data[:, 0]
        y_points = data[:, 1]
        z_points = data[:, 2]

        # Only keep points in the range specified above
        x_filter = np.logical_and((x_points >= self.fwd_range[0]), (x_points <= self.fwd_range[1]))
        y_filter = np.logical_and((y_points >= -self.side_range[1]), (y_points <= -self.side_range[0]))
        # z_filter = np.logical_and(())
        filter = np.logical_and(x_filter, y_filter)
        indices = np.argwhere(filter).flatten()

        x_points = x_points[indices]
        y_points = y_points[indices]
        z_points = z_points[indices]

        def scale_to_255(a, min_val, max_val, dtype=np.uint8):
            a = (((a-min_val) / float(max_val - min_val) ) * 255).astype(dtype)
            tmp = copy.deepcopy(a)
            a[:] = 0
            a[tmp>128] = 255
            return a

        # clip based on height for pixel Values
        pixel_vals = np.clip(a=z_points, a_min=self.height_range[0], a_max=self.height_range[1])

        pixel_vals = scale_to_255(pixel_vals, min_val=self.height_range[0], max_val=self.height_range[1])
        
        # Getting sensor reading for left       
        filter_left = np.logical_and((x_points>-0.1), (x_points<0.1))
        filter_left = np.logical_and(filter_left, y_points > 0)
        filter_left = np.logical_and(filter_left, pixel_vals > 128)
        indices = np.argwhere(filter_left).flatten()

        self.x_left = np.mean(x_points[indices])
        self.y_left = np.mean(y_points[indices])
        
        # Getting sensor reading for right       
        filter_right = np.logical_and((x_points>-0.1), (x_points<0.1))
        filter_right = np.logical_and(filter_right, y_points < 0)
        filter_right = np.logical_and(filter_right, pixel_vals > 128)
        indices = np.argwhere(filter_right).flatten()

        self.x_right = np.mean(x_points[indices])
        self.y_right = np.mean(y_points[indices])
        
        # Getting sensor reading for rear       
        filter_rear = np.logical_and((y_points>-0.1), (y_points<0.1))
        filter_rear = np.logical_and(filter_rear, x_points < 0)
        filter_rear = np.logical_and(filter_rear, pixel_vals > 128)
        indices = np.argwhere(filter_rear).flatten()

        self.x_rear = np.mean(x_points[indices])
        self.y_rear = np.mean(y_points[indices])

        # Getting sensor reading for front       
        filter_front = np.logical_and((y_points>-0.1), (y_points<0.1))
        filter_front = np.logical_and(filter_front, x_points > 0)
        filter_front = np.logical_and(filter_front, pixel_vals > 128)
        indices = np.argwhere(filter_front).flatten()

        self.x_front = np.mean(x_points[indices])
        self.y_front = np.mean(y_points[indices])
        
        ## TODO: Add 4 additional sensor directions #####
        # Handle sensor at 4 diagnal direction
        # Getting sensor reading for front left
        

        # Getting sensor reading for rear left
        

        # Getting sensor reading for front right
        

        # Getting sensor reading for rear right
        
        ###############

        # convert points to image coords with resolution
        x_img = np.floor(-y_points / self.resolution).astype(np.int32)
        y_img = np.floor(-x_points / self.resolution).astype(np.int32)

        # shift coords to new original
        x_img -= int(np.floor(self.side_range[0] / self.resolution))
        y_img += int(np.ceil(self.fwd_range[1] / self.resolution))
            
        im[y_img, x_img] = pixel_vals

        self.__birds_eye_view = im

        img = self.__birds_eye_view.astype(np.uint8)
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        # Visualize sensor readings at 4 orthogonal direction
        center = (self.vehicle_x, self.vehicle_y)
        cv2.circle(img, center, 5, (0,0,255), -1, 8, 0)
        
        center = self.convert_to_image(self.x_front, self.y_front)
        cv2.circle(img, center, 5, (0,255,0), -1, 8, 0)
        if not np.isnan(self.x_front) and not np.isnan(self.y_front):
            cv2.arrowedLine(img, (self.vehicle_x,self.vehicle_y), center, (255,0,0))
        
        center = self.convert_to_image(self.x_rear, self.y_rear)
        cv2.circle(img, center, 5, (0,255,0), -1, 8, 0)
        if not np.isnan(self.x_rear) and not np.isnan(self.y_rear):
            cv2.arrowedLine(img, (self.vehicle_x,self.vehicle_y), center, (255,0,0))
        
        center = self.convert_to_image(self.x_left, self.y_left)
        cv2.circle(img, center, 5, (0,255,0), -1, 8, 0)
        if not np.isnan(self.x_left) and not np.isnan(self.y_left):
            cv2.arrowedLine(img, (self.vehicle_x,self.vehicle_y), center, (255,0,0))
        
        center = self.convert_to_image(self.x_right, self.y_right)
        cv2.circle(img, center, 5, (0,255,0), -1, 8, 0)
        if not np.isnan(self.x_right) and not np.isnan(self.y_right):
            cv2.arrowedLine(img, (self.vehicle_x,self.vehicle_y), center, (255,0,0))
        
        # Visualize sensor readings at 4 diagnal direction
        center = self.convert_to_image(self.x_front_left, self.y_front_left)
        cv2.circle(img, center, 5, (0,255,0), -1, 8, 0)
        if not np.isnan(self.x_front_left) and not np.isnan(self.y_front_left):
            cv2.arrowedLine(img, (self.vehicle_x,self.vehicle_y), center, (255,0,0))

        center = self.convert_to_image(self.x_rear_left, self.y_rear_left)
        cv2.circle(img, center, 5, (0,255,0), -1, 8, 0)
        if not np.isnan(self.x_rear_left) and not np.isnan(self.y_rear_left):
            cv2.arrowedLine(img, (self.vehicle_x,self.vehicle_y), center, (255,0,0))

        center = self.convert_to_image(self.x_front_right, self.y_front_right)
        cv2.circle(img, center, 5, (0,255,0), -1, 8, 0)
        if not np.isnan(self.x_front_right) and not np.isnan(self.y_front_right):
            cv2.arrowedLine(img, (self.vehicle_x,self.vehicle_y), center, (255,0,0))

        center = self.convert_to_image(self.x_rear_right, self.y_rear_right)
        cv2.circle(img, center, 5, (0,255,0), -1, 8, 0)
        if not np.isnan(self.x_rear_right) and not np.isnan(self.y_rear_right):
            cv2.arrowedLine(img, (self.vehicle_x,self.vehicle_y), center, (255,0,0))
        birds_eye_im = self.cvBridge.cv2_to_imgmsg(img, 'bgr8')

        self.birdsEyeViewPub.publish(birds_eye_im)


    def convert_to_image(self, x, y):
        x_img = np.floor(-y / self.resolution).astype(np.int32)
        y_img = np.floor(-x / self.resolution).astype(np.int32)

        x_img -= int(np.floor(self.side_range[0] / self.resolution))
        y_img += int(np.ceil(self.fwd_range[1] / self.resolution))
        return (x_img, y_img)

    def processLidar(self):
        """
            Publishes birds eye view image

            Inputs: None

            Outputs: None
        """

        front = np.sqrt(self.x_front**2+self.y_front**2)
        rear = np.sqrt(self.x_rear**2+self.y_rear**2)
        left = np.sqrt(self.x_left**2+self.y_left**2)
        right = np.sqrt(self.x_right**2+self.y_right**2)
        front_left = np.sqrt(self.x_front_left**2+self.y_front_left**2)
        front_right = np.sqrt(self.x_front_right**2+self.y_front_right**2)
        rear_left = np.sqrt(self.x_rear_left**2+self.y_rear_left**2)
        rear_right = np.sqrt(self.x_rear_right**2+self.y_rear_right**2)

        if np.isnan(front):
            front = self.sensor_limit
        if np.isnan(right):
            right = self.sensor_limit
        if np.isnan(rear):
            rear = self.sensor_limit
        if np.isnan(left):
            left = self.sensor_limit
        if np.isnan(front_left):
            front_left = self.sensor_limit
        if np.isnan(front_right):
            front_right = self.sensor_limit
        if np.isnan(rear_left):
            rear_left = self.sensor_limit
        if np.isnan(rear_right):
            rear_right = self.sensor_limit
        
        return [front*100, right*100, rear*100, left*100]


        