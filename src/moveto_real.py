#!/usr/bin/python

import rospy
from james_ur_kinematics.srv import *
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Bool

from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from actionlib_msgs.msg import GoalStatusArray

from time import time, sleep

JOINT_NAMES = [ 'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint' ]

class MoveTo_node():
    def __init__(self):
        rospy.init_node('ur_moveto_node', anonymous=True)
        
        rospy.wait_for_service('/ur5_kin/IK')
        self._ik = rospy.ServiceProxy('/ur5_kin/IK', IK)
        self._jang_pub = rospy.Publisher('follow_joint_trajectory/goal',
            FollowJointTrajectoryActionGoal, queue_size=1)
        self._gripper_pub = rospy.Publisher('ur5/command/gripper', Bool, queue_size=1)
        
        rospy.Service('ur5/command/moveToPoses', MoveTo, self._m2p_cb)
        rospy.Service('ur5/command/moveToJoints', MoveTo, self._m2j_cb)
        
        sleep(2.0)
        
        rospy.spin()
        
    def _m2_cb(self, req, req_type):
        assert (req_type == 'p' or req_type == 'j')
        
        req_ln_ = len(req.ur_state)
        if req_ln_ > 0:
            if req_type == 'p': n_steps_ = int((req_ln_-1) / 7) # 7-tuple pose
            elif req_type == 'j': n_steps_ = int((req_ln_-1) / 6) # 6-tuple joint position
            
            time_per_step_ = req.ur_state[-1] / n_steps_ # time between each step
            
            # create message
            goal_msg_ = FollowJointTrajectoryActionGoal()
            goal_msg_.goal.trajectory.joint_names = JOINT_NAMES
            goal_msg_.goal.trajectory.points = [ JointTrajectoryPoint() ] * n_steps_
            
            for i in range(n_steps_):
                if req_type == 'p': # joint positions from IK
                    sI_ = i*7; pose_ = req.ur_state[sI_:sI_+7]
                    ik_req_ = IKRequest(ee_pose=Pose(
                        position=Point(*pose_[:3]), orientation=Quaternion(*pose_[3:])))
                    jangs_ = self._ik(ik_req_).joint_angles[:6]
                elif req_type == 'j': # copy joint positions
                    sI_ = i*6; jangs_ = req.ur_state[sI_:sI_+6]
                
                # append
                goal_msg_.goal.trajectory.points[i].positions = jangs_
                tfs_ = rospy.Duration((i + 1) * time_per_step_)
                goal_msg_.goal.trajectory.points[i].time_from_start = tfs_
                
            # final state stuff
            goal_msg_.goal.trajectory.points[-1].velocities = [ 0.0 ] * 6
            goal_msg_.goal.trajectory.points[-1].accelerations = [ 0.0 ] * 6
            
            # publish
            self._jang_pub.publish(goal_msg_)
            # wait for movement to finish
            sleep(req.ur_state[-1])
            while True:
                status_msg_ = rospy.wait_for_message(
                    'follow_joint_trajectory/status', GoalStatusArray).status_list[0]
                if status_msg_.status == status_msg_.ACTIVE: sleep(0.1)
                else: break
        
        # operate gripper
        self._gripper_pub.publish(Bool(data=req.gripper_state))
        if req.gripper_state: sleep(2.0) # closing
        else: sleep(0.5) # opening
        
        return []
        
    def _m2p_cb(self, req):
        return self._m2_cb(req, 'p')
        
    def _m2j_cb(self, req):
        return self._m2_cb(req, 'j')
        
if __name__ == '__main__':
    m2_node_ = MoveTo_node()
