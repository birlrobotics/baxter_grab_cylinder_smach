#!/usr/bin/python
#coding:utf-8

import rospy
import math
from geometry_msgs.msg import *
import numpy as np
import tf
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry


def get_pose_from_stamped_tf(trans, rot):
    Pose = geometry_msgs.msg.PoseStamped()
    Pose.pose.position.x = trans[0]
    Pose.pose.position.y = trans[1]
    Pose.pose.position.z = trans[2]
    Pose.pose.orientation.x = rot[0]
    Pose.pose.orientation.y = rot[1]
    Pose.pose.orientation.z = rot[2]
    Pose.pose.orientation.w = rot[3]

    return Pose


def convertPlanarQuat2Phi(rot):
    return 2 * math.atan2(rot[2], rot[3])


def print_Stamped_Pose(Pose):
    print ("tx: ", Pose.pose.position.x,
           "ty: ", Pose.pose.position.y,
           "tz: ", Pose.pose.position.z,
           "rx: ", Pose.pose.orientation.x,
           "ry: ", Pose.pose.orientation.y,
           "rz: ", Pose.pose.orientation.z,
           "rw: ", Pose.pose.orientation.w,)
