#include <ros/ros.h>
#include <james_ur_kinematics/FK.h>
#include <james_ur_kinematics/IK.h>

#include <tf/transform_listener.h>

#include <ur_kinematics/ur_kin.h>
//#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

using namespace james_ur_kinematics;
using namespace Eigen;

//// joint angle adjustments for differing 0 position definitions

std::vector<double> ext_joint_offsets; const double TWOPI = 2 * M_PI;
  
void normalize_angles(std::vector<double>& vec)
{
  for(size_t j=0; j<vec.size(); j++)
    if(std::isfinite(vec[j]))
    {
      while(vec[j] > M_PI) vec[j] -= TWOPI;
      while(vec[j] < -M_PI) vec[j] += TWOPI;
    }
}

void jangs_from_ext(std::vector<double>& out, const std::vector<double>& in)
{
  for(int i=0; i<6; i++)
    out[i] = in[i] + ext_joint_offsets[i];
  
  normalize_angles(out);
}

void jangs_to_ext(std::vector<double>& vec, const int nr)
{
  int i = 0;
  for(int r=0; r<nr; r++)
    for(int c=0; c<6; c++)
      { vec[i] -= ext_joint_offsets[c]; i++; }
      
  normalize_angles(vec);
}

//// static transforms for fixed-base UR

tf::StampedTransform world_robot_stf;
tf::Transform world_robot_istf, gripper_ee_stf, gripper_ee_istf;

void get_ee_pose(geometry_msgs::Pose& pose, const tf::Transform& raw_robot_tf)
{
  tf::Transform world_ee_tf_ = world_robot_stf * raw_robot_tf * gripper_ee_stf;
  tf::poseTFToMsg(world_ee_tf_, pose);
}

void get_robot_j6_TF(tf::Transform& rj6_tf, const geometry_msgs::Pose& pose)
{
  tf::Transform world_ee_tf_; tf::poseMsgToTF(pose, world_ee_tf_);
  rj6_tf = world_robot_istf * (world_ee_tf_ * gripper_ee_istf);
}

//// FK

bool fwd_svc(FK::Request& req, FK::Response& res)
{
  // apply ur_kinematics -> VREP angular origin offsets
  std::vector<double> jangs_(6); jangs_from_ext(jangs_, req.joint_angles);
  // get 4x4 affine transformation double[], convert to Eigen matrix
  double M_[16]; ur_kinematics::forward(jangs_.data(), M_);
  Affine3d F_; F_.matrix() = Map<Matrix<double,4,4,RowMajor> >(M_);
//  tf::poseEigenToMsg(F_, res.ee_pose); // return raw joint position 
  // return gripper position wrt world frame
  tf::Transform raw_tf_; tf::poseEigenToTF(F_, raw_tf_);
  get_ee_pose(res.ee_pose, raw_tf_);

  return 1;
}

//// IK

bool inv_svc(IK::Request& req, IK::Response& res)
{
  // affine matrix of desired robot base to joint 6
  tf::Transform rj6_tf_; get_robot_j6_TF(rj6_tf_, req.ee_pose);
  Affine3d F_; tf::poseTFToEigen(rj6_tf_, F_);
  // try solve for IK solutions
  Matrix4d FT_ = F_.matrix(); FT_.transposeInPlace(); // needs row-major
  double sols_[48]; int nsols_ = ur_kinematics::inverse(FT_.data(), sols_);
  // populate return vector, apply inverse angular origin offsets
  res.joint_angles.clear();
  if(nsols_ != 0)
  {
    std::copy(&sols_[0],&sols_[nsols_*6], std::back_inserter(res.joint_angles));
    jangs_to_ext(res.joint_angles, nsols_);
  }
  
  return 1;
}

int main(int argc, char**argv)
{
  ros::init(argc, argv, "ur5_kin_node");
  ros::NodeHandle nh_("~");
  
  // get static transforms for end-effector FK, and their inverses for ee IK
  tf::TransformListener tflr_; ros::Duration(1.0).sleep();
  tflr_.lookupTransform("world", "ur5_base_link", ros::Time(0), world_robot_stf);
  tf::StampedTransform gripper_rot_adj_, gripper_ee_stf_;
  tflr_.lookupTransform("gripper_link", "gripper_link_", ros::Time(0), gripper_rot_adj_);
  tflr_.lookupTransform("gripper_link", "gripper_ee", ros::Time(0), gripper_ee_stf_);
  gripper_ee_stf = gripper_rot_adj_.inverse() * gripper_ee_stf_;
  world_robot_istf = world_robot_stf.inverse(); gripper_ee_istf = gripper_ee_stf.inverse();
  
  int ext_sw_; nh_.getParam("ext_switch", ext_sw_);
  switch(ext_sw_)
  {
    case 1 : // v-rep
    {
      double jofs_[6] = { M_PI/2, -M_PI/2, 0.0, -M_PI/2, 0.0, M_PI/2 };
      ext_joint_offsets = std::vector<double>(jofs_, jofs_+6);
      break;
    }
    default: ext_joint_offsets = std::vector<double>(6, 0.0);
  }
  
  ros::ServiceServer fwd_srv_ = nh_.advertiseService("/ur5_kin/FK", &fwd_svc),
    inv_srv_ = nh_.advertiseService("/ur5_kin/IK", &inv_svc);  
    
  ros::spin();
  
  return 0;
}
