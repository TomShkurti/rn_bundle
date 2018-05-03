//
// Created by william on 28/03/18.
// @ HMS Prince of Wales
//

#include <cwru_davinci_kinematics/davinci_inv_kinematics.h>
#include <cwru_davinci_control/psm_controller.h>
#include "rn_skills_as/RnNeedleDrivingPlanner.h"

void goToLocationPointingDown(psm_controller &psm,
                              double x,
                              double y,
                              double z);

void executeTrajectory(psm_controller &psm,
                       trajectory_msgs::JointTrajectory &needleDriveTraj);



int main(int argc, char **argv) {

  ros::init(argc, argv, "test_controller");
  ros::NodeHandle node;

  int arm = 1;

  /// Phase I

  psm_controller psm(1, node);

  ROS_INFO("This is a controller test main. Will start in 1 sec.");
  ros::Duration(1).sleep();

//  ROS_INFO("Phase I: calling goToLocationPointingDown().");
//  goToLocationPointingDown(psm, 0, 0.05, -0.15);
//  ros::Duration(2).sleep();

  /// Phase II

  ROS_INFO("Phase II: Testing needle drive planner");
  RnNeedleDrivingPlanner rnNeedleDrivingPlanner(node);
  trajectory_msgs::JointTrajectory needleDriveTraj;
  Eigen::Vector3d pt_entry, pt_exit;
  geometry_msgs::PointStamped needle_entry_pt;
  geometry_msgs::PointStamped needle_exit_pt;
  geometry_msgs::TransformStamped grasp_transform;
  Eigen::Affine3d eigen_grasp_transform;

  eigen_grasp_transform = rnNeedleDrivingPlanner.getDefaultNeedleAffineWrtGraspOneFrame();

  pt_entry << 0, 0, -0.15;
  pt_exit << 0, 0.007, -0.15;

  needle_entry_pt = rnNeedleDrivingPlanner.ConvertEigenVec3dToGeoMsgsPtStamped(pt_entry);
  needle_exit_pt = rnNeedleDrivingPlanner.ConvertEigenVec3dToGeoMsgsPtStamped(pt_exit);

  bool test = false;

  rnNeedleDrivingPlanner.setDefaultGraspTfSearchResolution(36);

  test = rnNeedleDrivingPlanner.requestOneNeedleDrivingTrajectoryInBaseFrame(arm,
                                                                             needle_entry_pt,
                                                                             needle_exit_pt,
                                                                             needleDriveTraj,
                                                                             grasp_transform);

  if (test) {
    double velocity = 0.01; // meter/sec
    rnNeedleDrivingPlanner.setTrajectoryVelocity(0.02, needleDriveTraj);
    ROS_WARN("Will execute the plan in 1 sec.");
    ros::Duration(1).sleep();
    // executeTrajectory(psm, needleDriveTraj);

  } else {
    ROS_ERROR("FAILED TO GET A TRAJECTORY!");
  }



}

void goToLocationPointingDown(psm_controller &psm,
                              double x,
                              double y,
                              double z) {


  davinci_kinematics::Inverse ik_solver;
  davinci_kinematics::Vectorq7x1 q_vec;

  Eigen::Vector3d tip_origin;
  Eigen::Vector3d x_vec, y_vec, z_vec;
  Eigen::Matrix3d tip_rotation;
  Eigen::Affine3d des_affine;
  double norm;

  double time = 7;

  trajectory_msgs::JointTrajectoryPoint trajPoint_0;
  trajectory_msgs::JointTrajectoryPoint trajPoint;
  trajectory_msgs::JointTrajectoryPoint trajPoint_2;
  trajectory_msgs::JointTrajectory traj;

  trajPoint.positions.resize(7);
  trajPoint_2.positions.resize(7);


  // FIXME this will cause bug: it never gets and the process got stuck here,
  // rostopic echo joint states worked well.
  sensor_msgs::JointState js;
  psm.get_fresh_psm_state(js);

  trajPoint_0.time_from_start = ros::Duration(0);
  trajPoint_0.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  trajPoint_0.positions = js.position;

  std::cout << "trajPoint_0:" << std::endl << trajPoint_0 << std::endl;


  // Deduce the gripper rotation w/rt base frame first
  norm = sqrt(x*x + y*y);
  x_vec << x/norm, y/norm, 0;
  z_vec << 0, 0, -1;
  y_vec = z_vec.cross(x_vec);
  tip_rotation.col(0) = x_vec;
  tip_rotation.col(1) = y_vec;
  tip_rotation.col(2) = z_vec;



  // Fill in the affine
  tip_origin << x, y, z;
  des_affine.linear() = tip_rotation;
  des_affine.translation() = tip_origin;


  // Sent to the IK solver
  ik_solver.ik_solve(des_affine);
  q_vec = ik_solver.get_soln();


  // Fill in the traj msgs
  for (int i = 0; i < 7; i++) {
    trajPoint.positions[i] = q_vec[i];
    // trajPoint.positions[i+7] = 0; // TODO do we need this for PSM2? (so the size is 14?)
    trajPoint_2.positions[i] = q_vec[i];
    // trajPoint_2.positions[i+7] = 0; // TODO do we need this for PSM2? (so the size is 14?)
  }
  trajPoint.time_from_start = ros::Duration(time);
  trajPoint_2.time_from_start = ros::Duration(time+1);
  traj.points.clear();
  traj.joint_names.clear();
  traj.header.stamp = ros::Time::now();
  // traj.points.push_back(trajPoint_0);
  traj.points.push_back(trajPoint);
  // traj.points.push_back(trajPoint_2);
  //

  std::cout << "traj: " << std::endl << traj << std::endl;


  // Order the PSM to move
  ROS_INFO("Going to (%f, %f, %f)", x, y, z);
  psm.move_psm(traj);
  ros::Duration(time).sleep(); // TODO is this necessary?
  ROS_INFO("Done");


}

void executeTrajectory(psm_controller &psm,
                       trajectory_msgs::JointTrajectory &needleDriveTraj) {

  ROS_WARN("Reviewing Plan");

  int size = needleDriveTraj.points.size();

  ros::Duration duration = needleDriveTraj.points[size-1].time_from_start;
  double secs = duration.toSec();

  std::cout << "needleDriveTraj: " << std::endl << needleDriveTraj << std::endl;

  ROS_INFO("secs: %f", secs);

  ros::Duration(5).sleep();

  ROS_WARN("Execute Plan");
  ros::Duration(1).sleep();

  psm.move_psm(needleDriveTraj);

  duration.sleep();

  ROS_WARN("Done");



}