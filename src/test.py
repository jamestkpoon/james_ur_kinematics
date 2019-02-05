#!/usr/bin/python

import rospy, tf_conversions
from james_ur_kinematics.srv import *

quaternion_from_euler = tf_conversions.transformations.quaternion_from_euler

# ros init, moveTo svc
rospy.init_node('thread_test_node', anonymous=True)

# grab an object
rospy.wait_for_service('ur5/command/moveTo')
m2_ = rospy.ServiceProxy('ur5/command/moveTo', MoveTo)

pose_tuple_ = [ 0.5, -0.2, 0.0 ] + list(quaternion_from_euler(0,1.5708,0)) + [ 5.0 ]
m2_(MoveToRequest(ur_state=pose_tuple_, gripper_state=True))

pose_tuple_[2] = 0.2
m2_(MoveToRequest(ur_state=pose_tuple_, gripper_state=True))

pose_tuple_[1] = 0.2
m2_(MoveToRequest(ur_state=pose_tuple_, gripper_state=True))

pose_tuple_[2] = 0.0
m2_(MoveToRequest(ur_state=pose_tuple_, gripper_state=False))