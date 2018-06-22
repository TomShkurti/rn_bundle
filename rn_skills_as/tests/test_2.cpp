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

  int arm1 = 1;
  int arm2 = 2;

  double phi_0 = 0.1 * M_PI;
  double phi_t = 0.5 * M_PI;

  psm_controller psm1(arm1, node);
  psm_controller psm2(arm2, node);

  RnNeedleDrivingPlanner rnNeedleDrivingPlanner(node);
  trajectory_msgs::JointTrajectory needleDriveTraj;
  Eigen::Vector3d pt_entry, pt_exit, pt_entry_cam, pt_exit_cam;
  geometry_msgs::PointStamped needle_entry_pt, needle_entry_pt_cam;
  geometry_msgs::PointStamped needle_exit_pt, needle_exit_pt_cam;
  geometry_msgs::TransformStamped grasp_transform;
  Eigen::Affine3d eigen_grasp_transform;
  Eigen::Affine3d grasp_transform_eigen;

  eigen_grasp_transform = rnNeedleDrivingPlanner.getDefaultNeedleAffineWrtGraspOneFrame();

  // Good point for PSM1
//  pt_entry << -0.1, 0.08, -0.18; //Good point for PSM1
//  pt_exit << -0.1, 0.09, -0.18; //Good point for PSM1

  pt_entry << -0.09, 0.08, -0.18; //Good point for PSM1
  pt_exit << -0.09, 0.09, -0.18; //Good point for PSM1

//  pt_entry << 0.09, 0.08, -0.18; //Good point for PSM2
//  pt_exit << 0.09, 0.09, -0.18; //Good point for PSM2

  rnNeedleDrivingPlanner.generateDualPsmOpBoundaryVertices();

  if (0 < 1)
  {
//  pt_entry << -0.12, 0.08, -0.16; //Good point for PSM1 & PSM2 (expressed in PSM1 frame)
//  pt_exit << -0.12, 0.09, -0.16; //Good point for PSM1 & PSM2 (expressed in PSM1 frame)

  pt_entry_cam = rnNeedleDrivingPlanner.transformPointFromBaseToLtCamFrame(arm1, pt_entry);
  pt_exit_cam = rnNeedleDrivingPlanner.transformPointFromBaseToLtCamFrame(arm1, pt_exit);

  needle_entry_pt = rnNeedleDrivingPlanner.ConvertEigenVec3dToGeoMsgsPtStamped(pt_entry);
  needle_exit_pt = rnNeedleDrivingPlanner.ConvertEigenVec3dToGeoMsgsPtStamped(pt_exit);

  needle_entry_pt_cam = rnNeedleDrivingPlanner.ConvertEigenVec3dToGeoMsgsPtStamped(pt_entry_cam);
  needle_exit_pt_cam = rnNeedleDrivingPlanner.ConvertEigenVec3dToGeoMsgsPtStamped(pt_exit_cam);

  bool test = false;

  rnNeedleDrivingPlanner.setDefaultGraspTfSearchResolution(36);
  rnNeedleDrivingPlanner.setPathWaypoints(24);

//  rnNeedleDrivingPlanner.goToLocationPointingDownFaceForward(psm1,  0, 0, -0.1);

  ros::Duration(1).sleep();

  ROS_WARN("Needle grasped.");

//  test = rnNeedleDrivingPlanner.requestOneNeedleDrivingTrajectoryInBaseFrame(arm1,
//                                                                             needle_entry_pt,
//                                                                             needle_exit_pt,
//                                                                             needleDriveTraj,
//                                                                             grasp_transform);

//  test = rnNeedleDrivingPlanner.requestNeedleDrivingTrajectoryDefaultGraspInBaseFrame(arm1,
//                                                                                      needle_entry_pt,
//                                                                                      needle_exit_pt,
//                                                                                      needleDriveTraj);

//  test = rnNeedleDrivingPlanner.requestNeedleDrivingTrajectoryDefaultGrasp(arm1,
//                                                                           needle_entry_pt_cam,
//                                                                           needle_exit_pt_cam,
//                                                                           needleDriveTraj);

//  test = rnNeedleDrivingPlanner.requestOneNeedleDrivingTrajectory(arm1,
//                                                                  needle_entry_pt_cam,
//                                                                  needle_exit_pt_cam,
//                                                                  needleDriveTraj,
//                                                                  grasp_transform);


  rnNeedleDrivingPlanner.updateNeedleAndTissueParameters(needle_entry_pt_cam, needle_exit_pt_cam);

  double phi_pen, phi_em, phi_02, phi_t2;

  phi_pen = rnNeedleDrivingPlanner.getPhiNeedlePenetration();
  phi_em = rnNeedleDrivingPlanner.getPhiNeedleEmergence();

  std::cout << "phi_pen: " << phi_pen << std::endl << "phi_em: " << phi_em << std::endl;

//  phi_02 = - 0.0698132; // 0.0698132 rad = 4 deg
  phi_02 = 0.17;
  phi_t2 = 3.7;

  test = rnNeedleDrivingPlanner.requestNeedleDrivingTrajectoryDefaultGrasp(arm1,
                                                                           needle_entry_pt_cam,
                                                                           needle_exit_pt_cam,
                                                                           phi_02,
                                                                           phi_t2,
                                                                           needleDriveTraj);

//  rnNeedleDrivingPlanner.updatePsmKinematicAvailability(arm1);

//  test = rnNeedleDrivingPlanner.requestOneNeedleDrivingTrajectoryGeneratedGrasp(arm1,
//                                                                                needle_entry_pt_cam,
//                                                                                needle_exit_pt_cam,
//                                                                                phi_02,
//                                                                                phi_t2,
//                                                                                needleDriveTraj,
//                                                                                grasp_transform);



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
    ROS_WARN("Will execute the plan in 1 sec.");
    ros::Duration(1).sleep();
    rnNeedleDrivingPlanner.executeTrajectory(psm1, needleDriveTraj);

    ROS_WARN("DEBUG 001");
    rnNeedleDrivingPlanner.printDebugAffineVessel();

  } else {
    ROS_ERROR("FAILED TO GET A TRAJECTORY!");
  }

  grasp_transform_eigen = rnNeedleDrivingPlanner.convertGeoTransformStampedToEigenAffine(grasp_transform);
  rnNeedleDrivingPlanner.printEigenAffine(grasp_transform_eigen);


  /////

//  test = rnNeedleDrivingPlanner.requestNeedleDrivingTrajectoryDefaultGrasp(arm2,
//                                                                           needle_entry_pt_cam,
//                                                                           needle_exit_pt_cam,
//                                                                           needleDriveTraj);

//    test = rnNeedleDrivingPlanner.requestOneNeedleDrivingTrajectoryGeneratedGrasp(arm2,
//                                                                    needle_entry_pt_cam,
//                                                                    needle_exit_pt_cam,
//                                                                    needleDriveTraj,
//                                                                    grasp_transform);

  test = false;

  if (test) {
    double velocity = 0.01; // meter/sec
    rnNeedleDrivingPlanner.setTrajectoryVelocity(velocity, needleDriveTraj);
    ROS_WARN("Will execute the plan in 1 sec.");
    ros::Duration(1).sleep();
    rnNeedleDrivingPlanner.executeTrajectory(psm2, needleDriveTraj);

    ROS_WARN("DEBUG 002");
    rnNeedleDrivingPlanner.printDebugAffineVessel();

  } else {
    ROS_ERROR("FAILED TO GET A TRAJECTORY!");
  }

  grasp_transform_eigen = rnNeedleDrivingPlanner.convertGeoTransformStampedToEigenAffine(grasp_transform);
  rnNeedleDrivingPlanner.printEigenAffine(grasp_transform_eigen);

}

  return 0;
}




