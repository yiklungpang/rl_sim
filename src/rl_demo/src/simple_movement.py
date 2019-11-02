#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Yik Lung Pang


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
from rl_demo.srv import SimpleMovement
from gazebo_msgs.srv import DeleteModel, SpawnModel, GetModelState
from geometry_msgs.msg import *
from std_msgs.msg import String
from math import pi


def main(req):
    """Main function to perform the experiment and record the effects
    Parameters
    ----------
    req : list of str
        The list of tools and objects names
    """
    rospy.loginfo('-------------------------------------------------')
    rospy.loginfo('Starting simple movement')
    rospy.loginfo('-------------------------------------------------')

    # Initialise moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

    # Interface to the group of joints belonging to the left UR5 arm
    group_name = 'manipulator'
    group = moveit_commander.MoveGroupCommander(group_name)
    
    # Spawn the object
    rospack = rospkg.RosPack()
    object_urdf_path = str(rospack.get_path('rl_demo')+'/models/cube.urdf')
    
    spawn_object('box', object_urdf_path, 0.5, 1.0, 0.05, 0, 0, 0)

    # Move arm to middle of the table
    move_arm(group, 0.5, 0.05, 0.05)

def simple_movement_server():
    """Set up the server for the record visual service
    """

    # Initiate this node
    rospy.init_node('simple_movement_server', anonymous=True)
    s = rospy.Service('simple_movement', SimpleMovement, main)

    # keep node running
    rospy.spin()



def move_arm(group, pos_x, pos_y, pos_z):
    """Move the arm
    Parameters
    ----------
    group : MoveGroupCommander
        MoveGroupCommander of the arm to be controlled
    pos_x : target x coordinate
        X coordinate of target position
    pos_y : target y coordinate
        y coordinate of target position
    pos_z : target z coordinate
        z coordinate of target position
    """

    # Define waypoint
    startpose = group.get_current_pose().pose
    startpose.position.x = pos_x
    startpose.position.y = pos_y
    startpose.position.z = pos_z

    # Compute and execute path
    plan,_ = group.compute_cartesian_path([startpose], 0.01, 0.0)
    group.execute(plan, wait=True)
    group.stop()


def spawn_object(model_name, model_path, spawn_x, spawn_y, spawn_z, row, pitch, yaw):
    """Spawn the object in the specified position and pose
    Parameters
    ----------
    model_name : str
        The name of the tool or object to be spawned
    model_path : str
        Absolute path to the object's urdf
    spawn_x : float
        X position for the object to spawn in
    spawn_y : float
        Y position for the object to spawn in
    spawn_z : float
        Z position for the object to spawn in
    row : float
        Row pose for the object to spawn in
    pitch : float
        Pitch pose for the object to spawn in
    yaw : float
        Yaw pose for the object to spawn in
    """

    # Wait until Gazebo spawning service is available
    rospy.loginfo('-------------------------------------------------')
    rospy.loginfo('Spawining object')
    rospy.loginfo('-------------------------------------------------')
    print("Waiting for gazebo services...")
    rospy.wait_for_service("/gazebo/spawn_urdf_model")
    print("Got it.")

    # Call spawning service
    try:
        spawn_model = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)

        # Open urdf file
        with open(model_path, "r") as f:
            model_xml = f.read()
        
        # Spawn the object
        quaternion = tf.transformations.quaternion_from_euler(row, pitch, yaw)
        model_pose = Pose(Point(x=spawn_x, y=spawn_y, z=spawn_z), Quaternion(quaternion[0],quaternion[1],quaternion[2],quaternion[3]))
        spawn_model(model_name, model_xml, "", model_pose, "world")
        print("Object spawned.")

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


## Main function
if __name__ == '__main__':
    try:
        simple_movement_server()
    except rospy.ROSInterruptException:
        pass