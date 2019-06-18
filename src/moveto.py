#!/usr/bin/python

import rospy
from james_ur_kinematics.srv import *
from std_msgs.msg import Float64MultiArray, Bool

from time import time, sleep

class MoveTo_node():
    def __init__(self):
        rospy.init_node('ur_moveto_node', anonymous=True)
        
        rospy.wait_for_service('/ur5_kin/IK')
        self._ik = rospy.ServiceProxy('/ur5_kin/IK', IK)
        self._jang_pub = rospy.Publisher('ur5/command/joint_positions',
            Float64MultiArray, queue_size=1)
        self._gripper_pub = rospy.Publisher('ur5/command/gripper', Bool, queue_size=1)
        sleep(1.0); self._ur_mov_timeout = 0.5; self._ur_mov_param = 'ur5/moving'
        
        rospy.Service('ur5/command/moveTo', MoveTo, self._m2_cb)
        
        rospy.spin()
        
    def _m2_cb(self, req):
        # disambiguate requested ur_state
        len_ = len(req.ur_state)
        if (len_ < 6): jangs_ = [] # invalid
        elif (len_ == 7): # pose 7-tuple; take first soln from IK 
            ik_req_ = IKRequest()
            ik_req_.ee_pose.position.x = req.ur_state[0]
            ik_req_.ee_pose.position.y = req.ur_state[1]
            ik_req_.ee_pose.position.z = req.ur_state[2]
            ik_req_.ee_pose.orientation.x = req.ur_state[3]
            ik_req_.ee_pose.orientation.y = req.ur_state[4]
            ik_req_.ee_pose.orientation.z = req.ur_state[5]
            ik_req_.ee_pose.orientation.w = req.ur_state[6]
            jangs_ = self._ik(ik_req_).joint_angles[:6]
        else: jangs_ = req.ur_state
        
#        print(jangs_)
        
        # operate joints
        if (len(jangs_) > 0):
            self._jang_pub.publish(Float64MultiArray(data=jangs_))
            t_ = time(); mov_ = False
            while ((time()-t_) < self._ur_mov_timeout) and not mov_:
                sleep(0.01); mov_ = rospy.get_param(self._ur_mov_param)
            while mov_: sleep(0.01); mov_ = rospy.get_param(self._ur_mov_param)
        
        # operate gripper
        self._gripper_pub.publish(Bool(data=req.gripper_state))
        if req.gripper_state: sleep(5.0) # closing
        else: sleep(0.5) # opening
        
        return []
        
if __name__ == '__main__':
    m2_node_ = MoveTo_node()
