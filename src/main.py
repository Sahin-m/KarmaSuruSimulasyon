#!/usr/bin/env python

import os
import rospy
from karmasim_ros_wrapper.msg import VelCmd, CarControls
from karmasim_dev_pkg.msg import SampleMessage

def readFile(filename):
    filehandle = open(filename)
    print(filehandle.read())
    filehandle.close()

if __name__ == "__main__":
    rospy.init_node('contester_node')

    #Relative path example
    script_dir = os.path.dirname(__file__)
    rel_path = '../res/ydc_formal.txt'
    abs_file_path = os.path.join(script_dir, rel_path)
    readFile(abs_file_path)

    uav_pub = rospy.Publisher('uav_cmd', VelCmd, queue_size=10)
    ugv_pub = rospy.Publisher('ugv_cmd', CarControls, queue_size=10)
    sample_msg_pub = rospy.Publisher('/karmasim_node/sample_message', SampleMessage, queue_size=10)
    print('Hello World')
    contest_parameters = rospy.get_param('/scenario')
    world_boundaries = contest_parameters["world_boundaries"]
    print('World Boundaries')
    print(world_boundaries)