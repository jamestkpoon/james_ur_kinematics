#!/usr/bin/python

import rospy
from james_ur_kinematics.srv import *
from tf.transformations import quaternion_from_euler

from std_msgs.msg import Float32MultiArray, Bool
from time import time, sleep

# ros init, moveTo svc
rospy.init_node('ur_kin_test_node', anonymous=True)

rospy.wait_for_service('/vrep/ur5/command/moveTo')
m2_ = rospy.ServiceProxy('/vrep/ur5/command/moveTo', MoveTo)

# grab something

pose_tuple_ = [ 0.8, 0.2, 1.1 ] + list(quaternion_from_euler(0,1.5708,0))
m2_(MoveToRequest(ur_state=pose_tuple_, gripper_state=False))

pose_tuple_[2] = 0.95
m2_(MoveToRequest(ur_state=pose_tuple_, gripper_state=True))
