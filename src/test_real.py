#!/usr/bin/python

import rospy, tf_conversions
from james_ur_kinematics.srv import *

quaternion_from_euler = tf_conversions.transformations.quaternion_from_euler



## ros init, moveTo svc
#rospy.init_node('thread_test_node', anonymous=True)

topic_ = '/ur5/command/moveToPoses'
rospy.wait_for_service(topic_)
m2_ = rospy.ServiceProxy(topic_, MoveTo)

## body positions
#from mujoco_ros.srv import *

#rospy.wait_for_service('/mujoco/get_relative_pose_bodies')
#obj_pose_svc_ = rospy.ServiceProxy('/mujoco/get_relative_pose_bodies', GetRelativePoseBodies)

#def obj_pose(obj_name):
#    res_ = obj_pose_svc_(GetRelativePoseBodiesRequest(bodyA_name='world', bodyB_name=obj_name))
#    
#    return res_.relpose

## reset simulation
#from std_srvs.srv import Empty,EmptyRequest
#reset_svc_ = rospy.ServiceProxy('/mujoco/reset', Empty)
#reset_svc_(EmptyRequest())

#rospy.sleep(1.0)



## go through some motions
#nut_pose_ = obj_pose('nut')
#pose_tuple_ = [ nut_pose_.position.x, nut_pose_.position.y, nut_pose_.position.z+0.1 ] + list(quaternion_from_euler(0,1.5708,0))
pose_tuple_ = [ 0.55, 0.2, 0.1 ] + list(quaternion_from_euler(0,1.5708,0)) + [ 5.0 ]
m2_(MoveToRequest(ur_state=pose_tuple_, gripper_state=True))

#pose_tuple_[2] = 0.01
#m2_(MoveToRequest(ur_state=pose_tuple_, gripper_state=True))

#pose_tuple_[0] = 0.58
#pose_tuple_[1] = -0.2
#pose_tuple_[2] = 0.072
#m2_(MoveToRequest(ur_state=pose_tuple_, gripper_state=True))

print('  Reached')

#rospy.sleep(1.0)

#pose_tuple_[1] = -0.1
#m2_(MoveToRequest(ur_state=pose_tuple_, gripper_state=True))

#shaft_pose_ = obj_pose('shaft')
#pose_tuple_ = [ shaft_pose_.position.x, shaft_pose_.position.y, shaft_pose_.position.z+0.01 ] + list(quaternion_from_euler(0,1.5708,0))
#m2_(MoveToRequest(ur_state=pose_tuple_, gripper_state=True))

#pose_tuple_[2] = shaft_pose_.position.z
#m2_(MoveToRequest(ur_state=pose_tuple_, gripper_state=True))

#pose_tuple_[2] += 0.01
#m2_(MoveToRequest(ur_state=pose_tuple_, gripper_state=True))

##pose_tuple_[2] = 0.15
##m2_(MoveToRequest(ur_state=pose_tuple_, gripper_state=True))

#pose_tuple_[1] = -0.2
#m2_(MoveToRequest(ur_state=pose_tuple_, gripper_state=True))

#pose_tuple_[2] = 0.08
#m2_(MoveToRequest(ur_state=pose_tuple_, gripper_state=True))



## Fake  N U T

#THREAD_PITCH = 0.015 # normal movement (m) per revolution
#from math import pi

#def turn_fastener_ik(turn_ang, pitch):
#    # zero yaw (just for this example), grab
#    pose_tuple_[3:] = list(quaternion_from_euler(0,1.5708,0))
#    m2_(MoveToRequest(ur_state=pose_tuple_, gripper_state=True))
#    # twist and release
#    pose_tuple_[2] -= THREAD_PITCH * turn_ang/(2*pi)
#    pose_tuple_[3:] = list(quaternion_from_euler(0,1.5708,turn_ang))
#    m2_(MoveToRequest(ur_state=pose_tuple_, gripper_state=False))    

#for i in range(10): turn_fastener_ik(-pi/3, THREAD_PITCH)

#pose_tuple_[2] = 1.0
#m2_(MoveToRequest(ur_state=pose_tuple_, gripper_state=False))

##from std_msgs.msg import Float32MultiArray

##def get_joint_positions():
##    msg_ = rospy.wait_for_message('/vrep/ur5/measurements/joint_positions', Float32MultiArray)
##    return list(msg_.data[1:])

##def turn_fastener_jpos(turn_ang, pitch):
##    # zero last joint, grab
##    jpos_ = get_joint_positions()
##    jpos_[-1] = 0.0
##    m2_(MoveToRequest(ur_state=jpos_, gripper_state=True))
##    # twist and release
##    jpos_[-1] = -turn_ang
##    m2_(MoveToRequest(ur_state=jpos_, gripper_state=False))
