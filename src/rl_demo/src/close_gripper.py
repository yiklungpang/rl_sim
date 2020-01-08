#!/usr/bin/env python

import copy
import json
import moveit_commander
import moveit_msgs.msg
import os
import random
import rospy
import rospkg
import sys
import tf
import time
import math
from rl_demo.srv import SimpleMovement
from gazebo_msgs.srv import DeleteModel, SpawnModel, GetModelState
from geometry_msgs.msg import *
from control_msgs.msg import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String, Header
from rospy.rostime import Duration
from math import pi



def jointTrajectoryCommand():
    # Initialize the node
    rospy.init_node('joint_control')

    pub = rospy.Publisher('/gripper/command', JointTrajectory, queue_size=10)
    jt = JointTrajectory()

    jt.header.stamp = rospy.Time.now()
    jt.header.frame_id = ""

    jt.joint_names.append("gripper_finger1_joint" )


    jt = JointTrajectory()
    jt.joint_names.append("gripper_finger1_joint")
    point = JointTrajectoryPoint()
    point.positions.append(0.33)
    point.time_from_start = rospy.Duration.from_sec(1.0)
    jt.points.append(point)

    # pub.publish(jt)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(jt)
        rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    try:
        jointTrajectoryCommand()
    except rospy.ROSInterruptException: pass