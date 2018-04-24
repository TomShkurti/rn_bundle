//
// Created by william on 24/04/18.
//

#include "rn_skills_as/DavinciSkillsActionServer.h"

DavinciSkillsActionServer::DavinciSkillsActionServer(ros::NodeHandle nodeHandle, string name):
    nh_(nodeHandle),
    as_(nh_, name,  boost::bind(&DavinciSkillsActionServer::executeCB, this, _1), false),
    rnNeedleDrivingPlanner(nh_) {

    ROS_INFO("Constructing a da Vinci Skills Action Server.");

    const time_t ctt = time(0);
    std::cout << asctime(localtime(&ctt)) << std::endl;

}

//DavinciSkillsActionServer::executeCB(const rn_skills_as::NeedleDriveLiteGoalConstPtr &goal) {
//
//
//
//
//
//}


void go(
    psm_controller & psm,
    double x,
    double y,
    double z,
    double a,
    double g,
    double t
){
  // Start at the last known position.
  sensor_msgs::JointState js;
  psm.get_fresh_psm_state(js);
  trajectory_msgs::JointTrajectoryPoint trajectory_point_prev;
  trajectory_point_prev.time_from_start = ros::Duration(0.0);
  trajectory_point_prev.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  trajectory_point_prev.positions = js.position;

  //Calculate joint-space version of trajectory using kinematics.
  davinci_kinematics::Inverse kin = davinci_kinematics::Inverse();
  Eigen::Vector3d tip_origin = Eigen::Vector3d(x, y, z);
  Eigen::Vector3d x_vec = Eigen::Vector3d(sin(a), cos(a), 0.0);
  Eigen::Vector3d z_vec = Eigen::Vector3d(0.0, 0.0, -1.);

  Eigen::Vector3d y_vec = z_vec.cross(x_vec);
  Eigen::Affine3d des_gripper_affine;
  Eigen::Matrix3d R;
  R.col(0) = x_vec;
  R.col(1) = y_vec;
  R.col(2) = z_vec;
  des_gripper_affine.linear() = R;
  des_gripper_affine.translation() = tip_origin;
  kin.ik_solve(des_gripper_affine);
  davinci_kinematics::Vectorq7x1 q_vec1 = kin.get_soln();
  //q_vec1[5] = q_vec1[5] + g / 2.0;
  q_vec1[6] = q_vec1[6] + g;

  trajectory_msgs::JointTrajectoryPoint trajectory_point_new;
  trajectory_point_new.positions.resize(14);
  trajectory_msgs::JointTrajectoryPoint trajectory_point_after;
  trajectory_point_after.positions.resize(14);
  for(int i = 0; i < 7; i++){
    //CHANGE *HERE* TODEAL WITH ARM-THRASHING AND WRONG PSM ISSUES
    trajectory_point_new.positions[i] = q_vec1[i];
    trajectory_point_new.positions[i + 7] = 0.0;//PSM2 should always be at 0.
    trajectory_point_after.positions[i] = q_vec1[i];
    trajectory_point_after.positions[i + 7] = 0.0;
  }
  trajectory_point_new.time_from_start = ros::Duration(t);
  trajectory_point_after.time_from_start = ros::Duration(t + 0.1);

  //Pack into one msg.
  trajectory_msgs::JointTrajectory des_trajectory;
  des_trajectory.points.clear();
  des_trajectory.joint_names.clear();
  des_trajectory.header.stamp = ros::Time::now();
  //ROS_INFO("IN %f , %f", trajectory_point_prev.positions[5], trajectory_point_prev.positions[6]);
  //ROS_INFO("NEW %f , %f", trajectory_point_new.positions[5], trajectory_point_new.positions[6]);
  //ROS_INFO("OUT %f , %f", trajectory_point_after.positions[5], trajectory_point_after.positions[6]);
  des_trajectory.points.push_back(trajectory_point_prev);
  des_trajectory.points.push_back(trajectory_point_new);
  des_trajectory.points.push_back(trajectory_point_after);

  //Send it off
  psm.move_psm(des_trajectory);
  ros::Duration(t).sleep();
}