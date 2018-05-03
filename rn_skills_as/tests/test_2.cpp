//
// Created by william on 28/03/18.
//


#include <cwru_davinci_kinematics/davinci_inv_kinematics.h>
#include <cwru_davinci_control/psm_controller.h>
#include "rn_skills_as/RnNeedleDrivingPlanner.h"


void executeTrajectory(psm_controller &psm,
                       trajectory_msgs::JointTrajectory &needleDriveTraj);

int main(int argc, char **argv){

  ros::init(argc, argv, "test_2");
  ros::NodeHandle node;

  psm_controller psm(1, node);

  int arm = 1;

  ROS_INFO("Phase II: Testing needle drive planner");
  RnNeedleDrivingPlanner rnNeedleDrivingPlanner(node);
  trajectory_msgs::JointTrajectory needleDriveTraj;
  Eigen::Vector3d pt_entry, pt_exit;
  geometry_msgs::PointStamped needle_entry_pt;
  geometry_msgs::PointStamped needle_exit_pt;
  geometry_msgs::TransformStamped grasp_transform;
  Eigen::Affine3d eigen_grasp_transform;
  Eigen::Affine3d grasp_transform_eigen;

  eigen_grasp_transform = rnNeedleDrivingPlanner.getDefaultNeedleAffineWrtGraspOneFrame();

  pt_entry << -0.1, 0.08, -0.18;
  pt_exit << -0.1, 0.09, -0.18;

  needle_entry_pt = rnNeedleDrivingPlanner.ConvertEigenVec3dToGeoMsgsPtStamped(pt_entry);
  needle_exit_pt = rnNeedleDrivingPlanner.ConvertEigenVec3dToGeoMsgsPtStamped(pt_exit);

  bool test = false;

  rnNeedleDrivingPlanner.setDefaultGraspTfSearchResolution(36);

//  test = rnNeedleDrivingPlanner.requestOneNeedleDrivingTrajectoryInBaseFrame(arm,
//                                                                             needle_entry_pt,
//                                                                             needle_exit_pt,
//                                                                             needleDriveTraj,
//                                                                             grasp_transform);

  test = rnNeedleDrivingPlanner.requestNeedleDrivingTrajectoryDefaultGraspInBaseFrame(arm,
                                                                                      needle_entry_pt,
                                                                                      needle_exit_pt,
                                                                                      needleDriveTraj);

  if (test) {
    double velocity = 0.02; // meter/sec
    rnNeedleDrivingPlanner.setTrajectoryVelocity(velocity, needleDriveTraj);

    // TODO delete
    std::cout << "needleDriveTraj" << needleDriveTraj << std::endl;

    ROS_WARN("Will execute the plan in 1 sec.");
    ros::Duration(1).sleep();
    executeTrajectory(psm, needleDriveTraj);
  } else {
    ROS_ERROR("FAILED TO GET A TRAJECTORY!");
  }

  grasp_transform_eigen = rnNeedleDrivingPlanner.convertGeoTransformStampedToEigenAffine(grasp_transform);

  rnNeedleDrivingPlanner.printEigenAffine(grasp_transform_eigen);

  return 0;
}


void executeTrajectory(psm_controller &psm,
                       trajectory_msgs::JointTrajectory &needleDriveTraj) {

  ROS_WARN("Reviewing Plan");

  int size = needleDriveTraj.points.size();

  ros::Duration duration = needleDriveTraj.points[size-1].time_from_start;
  double secs = duration.toSec();

  ROS_INFO("secs: %f", secs);

//  ros::Duration(5).sleep();

  ROS_WARN("Execute Plan");
//  ros::Duration(1).sleep();

  psm.move_psm(needleDriveTraj);

  duration.sleep();

  ROS_WARN("Done");



}