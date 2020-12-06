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
    def __init__(self, resolution=0.5, side_range=(-2., 2.), fwd_range=(0., 2.),
                         height_range=(0.45, 0.46)):
        self.resolution = resolution
        self.side_range = side_range
        self.fwd_range = fwd_range
        self.height_range = (-1.992, 3)
        self.sensor_limit = (abs(fwd_range[0]) + abs(fwd_range[1]) + abs(side_range[0]) + abs(side_range[1]))/4
        self.current_x_points = []
        self.current_y_points = []
        self.current_z_points = []
        self.cvBridge = CvBridge()

        # random size initial image
        self.__birds_eye_view =  np.zeros((200, 100))

        #self.birdsEyeViewPub = rospy.Publisher("/mp4/BirdsEye", Image, queue_size=1)
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
        
        self.x_points = []
        self.y_points = []
        self.z_points = []

    def getBirdsEyeView(self):
        return self.__birds_eye_view

    def getCurrentPoints(self):
        return self.current_x_points, self.current_y_points,  self.current_z_points


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
    def transformPointCloud(self):
            Call back function that get the distance between vehicle and nearest wall in given direction
            The calculated values are stored in the class member variables
            Input: data - lidar point cloud
        """
        # create image from_array
        x_max = 1 + int((self.side_range[1] - self.side_range[0]) / self.resolution)
        y_max = 1 + int((self.fwd_range[1] - self.fwd_range[0]) / self.resolution)
        im = np.zeros([y_max, x_max], dtype=np.uint8)
        self.__raw_point_cloud = data


        if len(data) == 0:
            return im

        # Reference: http://ronny.rest/tutorials/module/pointclouds_01/point_cloud_birdseye/
        data = np.array(data)

        x_points = data[:, 0]
        y_points = data[:, 1]
        z_points = data[:, 2]
        f = open('z.txt', 'w')
        f.write(str(z_points))
        f.close()
    

        # Only keep points in the range specified above
        x_filter = np.logical_and((x_points >= self.fwd_range[0]), (x_points <= self.fwd_range[1]))
        y_filter = np.logical_and((y_points >= -self.side_range[1]), (y_points <= -self.side_range[0]))
        # z_filter = np.logical_and(())
        filter = np.logical_and(x_filter, y_filter)
        indices = np.argwhere(filter).flatten()

        self.x_points = x_points[indices]
        self.y_points = y_points[indices]
        self.z_points = z_points[indices]
        #print("og:",len(x_points))
        #print("maxz:", np.max(z_points))
        #print("minz:", np.min(z_points))
        def scale_to_255(a, min_val, max_val, dtype=np.uint8):
            a = (((a-min_val) / float(max_val - min_val) ) * 255).astype(dtype)
            tmp = copy.deepcopy(a)
            a[:] = 0
            a[tmp>128] = 255
            return a

        # clip based on height for pixel Values
        i  = 0
        length = len(self.x_points)
        while i < length:
            if (z_points[i] < self.height_range[0] or z_points[i] > self.height_range[1]):
                self.x_points = np.delete(self.x_points, i)
                self.y_points = np.delete(self.y_points, i)
                self.z_points = np.delete(self.z_points, i)
                i -= 1
                length -= 1
                #print("hit")
            i+=1
        #print("new:",len(self.x_points))
        self.current_x_points = self.x_points
        self.current_y_points = self.y_points
        self.current_z_points = self.z_points

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
