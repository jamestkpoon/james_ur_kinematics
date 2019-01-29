#!/usr/bin/python

import rospy
from james_ur_kinematics.srv import *
from tf.transformations import quaternion_from_euler

# ros init, moveTo svc
rospy.init_node('thread_test_node', anonymous=True)

# grab an object
print('  Grabbing something ...')
rospy.wait_for_service('ur5/command/moveTo')
m2_ = rospy.ServiceProxy('ur5/command/moveTo', MoveTo)

pose_tuple_ = [ 0.5, 0.0, 0.1 ] + list(quaternion_from_euler(0,1.5708,0)) + [ 10.0 ]
m2_(MoveToRequest(ur_state=pose_tuple_, gripper_state=False))

print('  n u t')

