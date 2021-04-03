#!/usr/bin/env python
import rospy
import roslib
import rospkg
from rospy.numpy_msg import numpy_msg

import sys
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointField
from geometry_msgs.msg import Point32
from hinge_detection.msg import custom_pc_msg
from plyfile import PlyData, PlyElement
from std_msgs.msg import Header


import numpy as np
import matplotlib


class input_point_cloud:
    def __init__(self):
        
        self._pc_subscriber = rospy.Subscriber("/amazon_ws/hinge_detection/filtered_cloud", custom_pc_msg, self.init_pointcloud,  queue_size = 1)
        self.process_flag = False           #This flag gives the filtering status
        #self.__compute_edge_length()
    
    def init_pointcloud(self, input_msg):
        print("Reading input pointcloud")
        
        input_points = []
        for point in point_cloud2.read_points(input_msg.published_point_cloud,  skip_nans=True):
            input_points.append((point[0],point[1],point[2]))
        #self.filtered_cloud = np.array([(-2,2,2),(-2,-1,2),(-2,-1,3),(1,2,1),(0,0,0),(1,-1,1),(2,2,2)], dtype=[('x','f8'), ('y','f8'), ('z','f8')])
        self.filtered_cloud = np.array(input_points,dtype=[('x','f8'), ('y','f8'), ('z','f8')])
        print(self.filtered_cloud[np.where(self.filtered_cloud['x']<0)])
        self.current_id = input_msg.id
        self.process_flag = True
        self.__compute_edge_length()


    def __compute_edge_length(self):
        
        #Extracting the min_x value from the point cloud
        print("In Compute Edge Length")
        self.sorted_cloud_x = np.sort(self.filtered_cloud, order=('x','y'))
        print("Printing sorted x")
        print(self.sorted_cloud_x)

        min_x_ind = np.where(self.sorted_cloud_x['x'] == self.sorted_cloud_x['x'][0])
        
        #print(min_x_ind[-1][-1])
        self.__point_1 = np.array([self.sorted_cloud_x[min_x_ind[-1][-1]][0],self.sorted_cloud_x[min_x_ind[-1][-1]][1],self.sorted_cloud_x[min_x_ind[-1][-1]][2]])
       
        #Extracting the min_y value from the point cloud
        self.sorted_cloud_y = np.sort(self.filtered_cloud, order=('y','z'))
    
        min_y_ind = np.where(self.sorted_cloud_y['y'] == self.sorted_cloud_y['y'][0])
        #self.__point_2 = np.array([self.sorted_cloud_y[min_x_ind[-1][-1]][0],self.sorted_cloud_y[min_x_ind[-1][-1]][1],self.sorted_cloud_y[min_x_ind[-1][-1]][2]])
        self.__point_2 = np.array([self.sorted_cloud_y[-1][0],self.sorted_cloud_y[-1][1],self.sorted_cloud_y[-1][2]])
        print("Printing sorted y")
        print(self.sorted_cloud_y)
        print(self.__point_1)
        print(self.__point_2)
        
        print("Before Linalg")
        self.edge_length = np.linalg.norm(self.__point_1 - self.__point_2)
        print(self.edge_length)


def main(args):
    rospy.init_node('point_cloud', anonymous=True)
    pub = rospy.Publisher("/amazon_ws/hinge_detection/filtered_cloud", custom_pc_msg, queue_size=10)

    ip_pointcloud = custom_pc_msg()
    ip_pointcloud.id = 1 
    

    input_pc = input_point_cloud()
    rospack = rospkg.RosPack()

    print("Done Initialization, waiting for callback")
    plyfile_path = rospack.get_path('hinge_detection') + "/data/L42.ply"
    plydata = PlyData.read(plyfile_path)
    points = plydata.elements[0].data
    ip_pointcloud.published_point_cloud.height = 1
    ip_pointcloud.published_point_cloud.width = len(points)

    fields = [
        PointField('x', 0, PointField.FLOAT64, 1),
        PointField('y', 4, PointField.FLOAT64, 1),
        PointField('z', 8, PointField.FLOAT64, 1),
        PointField('r', 12, PointField.FLOAT64, 1),
        PointField('g', 16, PointField.FLOAT64, 1),
        PointField('b', 20, PointField.FLOAT64, 1)
    ]
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "map"
    points_list = []
    for i in range(0,len(points),3):
        points_list.append([points[i][0],points[i][1],points[i][2],points[i][3],points[i][4],points[i][5]])

    ip_pointcloud.published_point_cloud = point_cloud2.create_cloud(header, fields, points_list)
    print(points_list)
    pub.publish(ip_pointcloud)
    

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Hinge Detector detector module")


if __name__ == '__main__':
    main(sys.argv)
