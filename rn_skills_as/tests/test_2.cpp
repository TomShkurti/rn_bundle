//
// Created by william on 28/03/18.
//


#include <cwru_davinci_kinematics/davinci_inv_kinematics.h>
#include <cwru_davinci_control/psm_controller.h>
#include "rn_skills_as/RnNeedleDrivingPlanner.h"




void goToLocationPointingDown(psm_controller &psm,
                              double x,
                              double y,
                              double z);

int main(int argc, char **argv) {

  ros::init(argc, argv, "test_2");
  ros::NodeHandle node;

  psm_controller psm(1, node);

  int arm = 1;

  ROS_INFO("Phase II: Testing needle drive planner");
  RnNeedleDrivingPlanner rnNeedleDrivingPlanner(node);
  trajectory_msgs::JointTrajectory needleDriveTraj;
  Eigen::Vector3d pt_entry, pt_exit, pt_entry_cam, pt_exit_cam;
  geometry_msgs::PointStamped needle_entry_pt, needle_entry_pt_cam;
  geometry_msgs::PointStamped needle_exit_pt, needle_exit_pt_cam;
  geometry_msgs::TransformStamped grasp_transform;
  Eigen::Affine3d eigen_grasp_transform;
  Eigen::Affine3d grasp_transform_eigen;

  eigen_grasp_transform = rnNeedleDrivingPlanner.getDefaultNeedleAffineWrtGraspOneFrame();

  pt_entry << -0.1, 0.08, -0.18;
  pt_exit << -0.1, 0.09, -0.18;

  pt_entry_cam = rnNeedleDrivingPlanner.transformPointFromBaseToLtCamFrame(arm, pt_entry);
  pt_exit_cam = rnNeedleDrivingPlanner.transformPointFromBaseToLtCamFrame(arm, pt_exit);



  needle_entry_pt = rnNeedleDrivingPlanner.ConvertEigenVec3dToGeoMsgsPtStamped(pt_entry);
  needle_exit_pt = rnNeedleDrivingPlanner.ConvertEigenVec3dToGeoMsgsPtStamped(pt_exit);

  needle_entry_pt_cam = rnNeedleDrivingPlanner.ConvertEigenVec3dToGeoMsgsPtStamped(pt_entry_cam);
  needle_exit_pt_cam = rnNeedleDrivingPlanner.ConvertEigenVec3dToGeoMsgsPtStamped(pt_exit_cam);

  bool test = false;

  rnNeedleDrivingPlanner.setDefaultGraspTfSearchResolution(36);
  rnNeedleDrivingPlanner.setPathWaypoints(72);

  rnNeedleDrivingPlanner.goToLocationPointingDownFaceForward(psm,  0, 0, -0.1);

  ros::Duration(1).sleep();

  ROS_WARN("Needle grasped.");

//  rnNeedleDrivingPlanner.goToLocationPointingDownFaceForward(psm, 0.01,0.01,-0.1);

//  test = rnNeedleDrivingPlanner.requestOneNeedleDrivingTrajectoryInBaseFrame(arm,
//                                                                             needle_entry_pt,
//                                                                             needle_exit_pt,
//                                                                             needleDriveTraj,
//                                                                             grasp_transform);

//  test = rnNeedleDrivingPlanner.requestNeedleDrivingTrajectoryDefaultGraspInBaseFrame(arm,
//                                                                                      needle_entry_pt,
//                                                                                      needle_exit_pt,
//                                                                                      needleDriveTraj);

  test = rnNeedleDrivingPlanner.requestNeedleDrivingTrajectoryDefaultGrasp(arm,
                                                                           needle_entry_pt_cam,
                                                                           needle_exit_pt_cam,
                                                                                      needleDriveTraj);

//  {
//    Eigen::Quaterniond q;
//    Eigen::Matrix3d mat;
//    q.x() = -0;
//    q.y() = 1;
//    q.z() = 0;
//    q.w() = 0;
//    mat = q.toRotationMatrix();
//    std::cout << std::endl << "DEBUG" << std::endl << mat << std::endl;
//  }

  if (test) {


    double velocity = 0.01; // meter/sec
    rnNeedleDrivingPlanner.setTrajectoryVelocity(velocity, needleDriveTraj);

    // TODO delete
//    std::cout << "needleDriveTraj" << needleDriveTraj << std::endl;

    ROS_WARN("Will execute the plan in 1 sec.");
    ros::Duration(1).sleep();
    rnNeedleDrivingPlanner.executeTrajectory(psm, needleDriveTraj);

//    rnNeedleDrivingPlanner.goToLocationPointingDownFaceVector(psm, 0.01,0.01,-0.1);

//    rnNeedleDrivingPlanner.goToLocationPointingDownFaceForward(psm, 0.01,0.01,-0.1);
  } else {
    ROS_ERROR("FAILED TO GET A TRAJECTORY!");
  }

  grasp_transform_eigen = rnNeedleDrivingPlanner.convertGeoTransformStampedToEigenAffine(grasp_transform);

  rnNeedleDrivingPlanner.printEigenAffine(grasp_transform_eigen);

  return 0;
}




/////
//void goToLocationPointingDown(psm_controller &psm,
//                              double x,
//                              double y,
//                              double z) {
//
//
//  davinci_kinematics::Inverse ik_solver;
//  davinci_kinematics::Vectorq7x1 q_vec;
//
//  Eigen::Vector3d tip_origin;
//  Eigen::Vector3d x_vec, y_vec, z_vec;
//  Eigen::Matrix3d tip_rotation;
//  Eigen::Affine3d des_affine;
//  double norm;
//
//  double time = 7;
//
//  trajectory_msgs::JointTrajectoryPoint trajPoint_0;
//  trajectory_msgs::JointTrajectoryPoint trajPoint;
//  trajectory_msgs::JointTrajectoryPoint trajPoint_2;
//  trajectory_msgs::JointTrajectory traj;
//
//  trajPoint.positions.resize(7);
//  trajPoint_2.positions.resize(7);
//
//
//  // FIXME this will cause bug: it never gets and the process got stuck here,
//  // rostopic echo joint states worked well.
//  sensor_msgs::JointState js;
//  psm.get_fresh_psm_state(js);
//
//  trajPoint_0.time_from_start = ros::Duration(0);
//  trajPoint_0.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//  trajPoint_0.positions = js.position;
//
//  std::cout << "trajPoint_0:" << std::endl << trajPoint_0 << std::endl;
//
//  // Deduce the gripper rotation w/rt base frame first
//  norm = sqrt(x*x + y*y);
//  x_vec << x/norm, y/norm, 0;
//  z_vec << 0, 0, -1;
//  y_vec = z_vec.cross(x_vec);
//  tip_rotation.col(0) = x_vec;
//  tip_rotation.col(1) = y_vec;
//  tip_rotation.col(2) = z_vec;
//
//
//  // Fill in the affine
//  tip_origin << x, y, z;
//  des_affine.linear() = tip_rotation;
//  des_affine.translation() = tip_origin;
//
//
//  // Sent to the IK solver
//  ik_solver.ik_solve(des_affine);
//  q_vec = ik_solver.get_soln();
//
//
//  // Fill in the traj msgs
//  for (int i = 0; i < 7; i++) {
//    trajPoint.positions[i] = q_vec[i];
//    // trajPoint.positions[i+7] = 0; // TODO do we need this for PSM2? (so the size is 14?)
//    trajPoint_2.positions[i] = q_vec[i];
//    // trajPoint_2.positions[i+7] = 0; // TODO do we need this for PSM2? (so the size is 14?)
//  }
//  trajPoint.time_from_start = ros::Duration(time);
//  trajPoint_2.time_from_start = ros::Duration(time+1);
//  traj.points.clear();
//  traj.joint_names.clear();
//  traj.header.stamp = ros::Time::now();
//  // traj.points.push_back(trajPoint_0);
//  traj.points.push_back(trajPoint);
//  // traj.points.push_back(trajPoint_2);
//  //
//
//  std::cout << "traj: " << std::endl << traj << std::endl;
//
//
//  // Order the PSM to move
//  ROS_INFO("Going to (%f, %f, %f)", x, y, z);
//  psm.move_psm(traj);
//  ros::Duration(time).sleep(); // TODO is this necessary?
//  ROS_INFO("Done");
//
//
//}
