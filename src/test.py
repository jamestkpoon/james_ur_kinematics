#!/usr/bin/python

import rospy
from james_ur_kinematics.srv import *
from tf.transformations import quaternion_from_euler

from std_msgs.msg import Float32MultiArray, Bool
from time import time, sleep

### ros init, comms
rospy.init_node('ur_kin_test_node', anonymous=True)

ik_svc_ = rospy.ServiceProxy('/ur5_kin/IK', IK)
fk_svc_ = rospy.ServiceProxy('/ur5_kin/FK', FK)

jang_pub_ = rospy.Publisher('/vrep/ur5/command/joint_positions', Float32MultiArray, queue_size=1)
gripper_pub_ = rospy.Publisher('/vrep/ur5/command/gripper', Bool, queue_size=1)
ur_mov_timeout_ = 0.5; ur_mov_param_ = '/vrep/ur5/moving'

sleep(1.0)

### useful functions

def test_fk(ik_req):
    ik_jangs_ = ik_svc_(ik_req).joint_angles
    nsol_ = len(ik_jangs_) / 6
    if nsol_ is 0: print('  No solutions found!'); return 0
    else:
        sol_0_ = ik_jangs_[:6]
        fk_res_ = fk_svc_(joint_angles=sol_0_).ee_pose;
        print(sol_0_); print(fk_res_)
        return 1

def move_to_pose(ik_req):
    ik_jangs_ = ik_svc_(ik_req).joint_angles
    nsol_ = len(ik_jangs_) / 6
    if nsol_ is 0: print('  No solutions found!'); return 0
    else:
        # publish first solution to UR joint position controller
        sol_0_ = ik_jangs_[:6]
        jang_pub_.publish(Float32MultiArray(data=map(float, sol_0_)))
        # wait for movement to start and stop
        t_ = time(); mov_ = False
        while (time()-t_)<ur_mov_timeout_ and not mov_:
            sleep(0.01); mov_ = rospy.get_param(ur_mov_param_)
        while mov_: sleep(0.01); mov_ = rospy.get_param(ur_mov_param_)
        
        return 1
        
def gripper_close():
    gripper_pub_.publish(Bool(data=True)); sleep(1.0)
    
def gripper_open():
    gripper_pub_.publish(Bool(data=False)); sleep(0.1)
    
### peg insert test script

ik_req_ = IKRequest()
ik_req_.ee_pose.position.x = 0.8
ik_req_.ee_pose.position.y = 0.2
ik_req_.ee_pose.position.z = 1.0
quat_ = quaternion_from_euler(0,1.5708,0)
ik_req_.ee_pose.orientation.x = quat_[0]
ik_req_.ee_pose.orientation.y = quat_[1]
ik_req_.ee_pose.orientation.z = quat_[2]
ik_req_.ee_pose.orientation.w = quat_[3]

move_to_pose(ik_req_)

# pick up cylinder
ik_req_.ee_pose.position.z = 0.952
move_to_pose(ik_req_)
gripper_close()

# insert
ik_req_.ee_pose.position.z = 1.2
move_to_pose(ik_req_)
ik_req_.ee_pose.position.y = -0.2
move_to_pose(ik_req_)
#ik_req_.ee_pose.position.z = 1.05
#move_to_pose(ik_req_)
gripper_open()
ik_req_.ee_pose.position.z = 1.2
move_to_pose(ik_req_)
