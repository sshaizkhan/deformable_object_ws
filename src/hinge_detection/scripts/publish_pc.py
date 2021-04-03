#!/usr/bin/env python
import rospy
import roslib
from rospy.numpy_msg import numpy_msg

import sys
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point32
from plyfile import PlyData, PlyElement
from hinge_detection.msg import custom_pc_msg


def read_ply_file(file_path):

