#!/usr/bin/python

import rospy
from james_ur_kinematics.srv import *

# inverse kinematics
ik_svc_ = rospy.ServiceProxy('/ur5_kin/IK', IK)
ik_req_ = IKRequest()
ik_req_.ee_pose.position.x = -0.534414901796
ik_req_.ee_pose.position.y = 0.416874021199
ik_req_.ee_pose.position.z = 1.76794107839
ik_req_.ee_pose.orientation.x = 0.178969645397
ik_req_.ee_pose.orientation.y = 0.258660721261
ik_req_.ee_pose.orientation.z = 0.90002231394
ik_req_.ee_pose.orientation.w = -0.301702389308
print(ik_req_.ee_pose)
ik_jangs_ = ik_svc_(ik_req_).joint_angles

# forward kinematics
fk_svc_ = rospy.ServiceProxy('/ur5_kin/FK', FK)
nsol_ = len(ik_jangs_) / 6
print('  Found %d IK solutions. Running through FK ...' % nsol_)
for s in range(nsol_):
    sI_ = s * 6; eI_ = sI_ + 48
    v_ = ik_jangs_[sI_:eI_]; print(v_)
    fk_res_ = fk_svc_(joint_angles=v_).ee_pose; print(fk_res_)
