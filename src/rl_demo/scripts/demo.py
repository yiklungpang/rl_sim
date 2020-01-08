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


import rospy
import roslaunch
import rospkg
from rl_demo.srv import SimpleMovement
from std_msgs.msg import String

def demo():

    # Initiate node
    rospy.init_node('demo', anonymous=True)
    
    simpleMovement()

def simpleMovement():

    # Launch new session of Gazebo and rViz
    # Launch image_saver node to retrieve camera feed
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    rospack = rospkg.RosPack()
    cli_args1 = [rospack.get_path('rl_gazebo')+'/launch/rl_simulation.launch']
    roslaunch_file1 = [(roslaunch.rlutil.resolve_launch_arguments(cli_args1)[0])]
    parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file1)
    parent.start()

    # Wait to make sure simulation is loaded completely
    rospy.sleep(3)

    # Start recording
    # simple_movement = rospy.ServiceProxy('/simple_movement', SimpleMovement)
    # try:
    #     simple_movement()
    # except rospy.ServiceException, e:
    #     print "Service call failed: %s" % e


    rospy.sleep(300)
    # Kill the simulation and other nodes
    # parent.shutdown()


## Main function
if __name__ == '__main__':
    try:
        demo()
    except rospy.ROSInterruptException:
        pass
