//
// Created by william on 01/05/18.
// @ HMS Prince of Wales
//


#include <cwru_davinci_kinematics/davinci_inv_kinematics.h>
#include <cwru_davinci_control/psm_controller.h>
#include "rn_skills_as/RnNeedleDrivingPlanner.h"


int main(int argc, char **argv){

  ros::init(argc, argv, "rn_needle_driving_fncs_test");
  ros::NodeHandle node;

  RnNeedleDrivingPlanner rnNeedleDrivingPlanner(node);

  psm_controller psm(1, node);

  trajectory_msgs::JointTrajectory needleDriveTraj;

  Eigen::Vector3d pt_entry, pt_exit;

  geometry_msgs::PointStamped needle_entry_pt;
  geometry_msgs::PointStamped needle_exit_pt;

  geometry_msgs::TransformStamped grasp_transform;

  Eigen::Affine3d eigen_grasp_transform;
  geometry_msgs::TransformStamped default_grasp_transform;

  Eigen::Affine3d grasp_tf;

  bool test;
  int arm = 1;

  pt_entry << 0, 0, -0.15;
  pt_exit << 0, 0.007, -0.15;

  needle_entry_pt = rnNeedleDrivingPlanner.ConvertEigenVec3dToGeoMsgsPtStamped(pt_entry);
  needle_exit_pt = rnNeedleDrivingPlanner.ConvertEigenVec3dToGeoMsgsPtStamped(pt_exit);


  std::string random_char;

  ROS_WARN("TEST SUBJECT 1: requestOneNeedleDrivingTrajectoryInBaseFrame()");

  ROS_INFO("Waiting...");


//  {
//    Eigen::Affine3d affine_0, affine_1;
//    geometry_msgs::TransformStamped geoTf_0;
//
//    affine_0 = rnNeedleDrivingPlanner.getDefaultNeedleAffineWrtGraspOneFrame();
//
//    ROS_WARN("AFFINE 0");
//    rnNeedleDrivingPlanner.printEigenAffine(affine_0);
//    std::cout << std::endl;
//
//    geoTf_0 = rnNeedleDrivingPlanner.convertEigenAffineToGeoTransform(affine_0);
//
//    ROS_WARN("CHECK geoTf_0");
//    std::cout << geoTf_0 << std::endl;
//
//    affine_1 = rnNeedleDrivingPlanner.convertGeoTransformStampedToEigenAffine(geoTf_0);
//
//    ROS_WARN("AFFINE 1");
//    rnNeedleDrivingPlanner.printEigenAffine(affine_1);
//    std::cout << std::endl;
//  }



  davinci_kinematics::Forward fwd_solver;
  davinci_kinematics::Inverse ik_solver;

  davinci_kinematics::Vectorq7x1 vec_test;
  Eigen::Vector3d origin_test;
  Eigen::Matrix3d rotation_test;
  Eigen::Affine3d affine_test, affine_result;
//  origin_test << 0.0117101, 0.0371173, -0.147376;
//  rotation_test << -8.47482e-318,      0.939693,       0.34202,
//                   -0.575319,      0.279748,     -0.768602,
//                   -0.817929,     -0.196771,      0.540623;

//  origin_test << -0.0560296, -0.0576075, -0.0591862;
//
//  rotation_test <<   -0.890662,   0.451223, -0.0558522,
//      -0.106172,  -0.325855,  -0.939439,
//      -0.442096,  -0.830793,   0.338134;
//
//  affine_test.linear() = rotation_test;
//  affine_test.translation() = origin_test;
//
//  int count;
//  count = ik_solver.ik_solve(affine_test);
//  vec_test = ik_solver.get_soln();
//
//  affine_result = fwd_solver.fwd_kin_solve(vec_test);
//  ROS_WARN("RESULT(%d): ", count);
//  rnNeedleDrivingPlanner.printEigenAffine(affine_result);
//  std::cout << "vec_test" << std::endl << vec_test.transpose() << std::endl;
//
//
//
//
//
//
//  std::cout << "Please type in a random char to proceed: ";
//  std::cin >> random_char;
//
//  eigen_grasp_transform = rnNeedleDrivingPlanner.getDefaultNeedleAffineWrtGraspOneFrame();
//
//  // TODO delete
//  ROS_WARN("Default Eigen Affine");
//  rnNeedleDrivingPlanner.printEigenAffine(eigen_grasp_transform);
//
//  default_grasp_transform = rnNeedleDrivingPlanner.convertEigenAffineToGeoTransform(eigen_grasp_transform);
//
//
//  rnNeedleDrivingPlanner.setDefaultGraspTfSearchResolution(36);
//
////    test = rnNeedleDrivingPlanner.requestNeedleDrivingTrajectoryDefaultGraspInBaseFrame(arm,
////                                                                                      needle_entry_pt,
////                                                                                      needle_exit_pt,
////                                                                                      needleDriveTraj);
//
//  test = rnNeedleDrivingPlanner.requestOneNeedleDrivingTrajectoryInBaseFrame(arm,
//                                                                             needle_entry_pt,
//                                                                             needle_exit_pt,
//                                                                             needleDriveTraj,
//                                                                             grasp_transform);
//
////  test = rnNeedleDrivingPlanner.requestOneNeedleDrivingTrajectoryInBaseFrame(arm,
////                                                                             needle_entry_pt,
////                                                                             needle_exit_pt,
////                                                                             default_grasp_transform,
////                                                                             needleDriveTraj);
//
//
//
//
//  ROS_INFO("test result: %d", int(test));
//
//  if (test) {
//
//    grasp_tf = rnNeedleDrivingPlanner.convertGeoTransformStampedToEigenAffine(grasp_transform);
//
//    ROS_INFO("grasp_transform: ");
//    rnNeedleDrivingPlanner.printEigenAffine(grasp_tf);
//
////    std::cout << std::endl;
////    std::cout << "needleDriveTraj: " << std::endl << needleDriveTraj << std::endl;
//
//    ROS_INFO("TEST SUBJECT 2: setTrajectoryVelocity()");
//
//    double velocity = 0.01; // meter/sec
//
//    rnNeedleDrivingPlanner.setTrajectoryVelocity(0.01, needleDriveTraj);
//
//    ROS_WARN("\nINSPECT GRASP_TF");
//    Eigen::Affine3d result_tf;
//    result_tf = rnNeedleDrivingPlanner.convertGeoTransformStampedToEigenAffine(grasp_transform);
//    rnNeedleDrivingPlanner.printEigenAffine(result_tf);
//
//  }

  double x1 = 0;
  double y1 = 0;
  double x2 = 1;
  double y2 = 1;
  double r = 1;
  double origin_x1, origin_y1, origin_x2, origin_y2;

//  rnNeedleDrivingPlanner.solveBinaryQuadraticCircleEquation( x1, y1, x2, y2, r,
//                                                             origin_x1, origin_y1,
//                                                             origin_x2, origin_y2);
//
//  std::cout << std::endl << "origin_x1: " << origin_x1 << std::endl
//            << "origin_y1: " << origin_y1 << std::endl
//            << "origin_x2: " << origin_x2 << std::endl
//            << "origin_y2: " << origin_y2 << std::endl;

  Eigen::Vector3d exit_pt, entry_pt;
  Eigen::Vector3d tip_pt;
  Eigen::Vector3d tissue_normal;


  exit_pt << 0, 0, 0.5;
  tip_pt << 0, 0.001, 0.499;
  tissue_normal << 0, 0, -1; // Note that the cam_z points down


  rnNeedleDrivingPlanner.defineTissueFrameWrtLtCameraViaExitAndTip(exit_pt, tip_pt, tissue_normal, entry_pt);

  rnNeedleDrivingPlanner.printPsiNeedleAxisTiltWrtTissue();

  rnNeedleDrivingPlanner.printNeedleAffineWrtGripperFrames();

  rnNeedleDrivingPlanner.printDebugAffineVessel();

  psm_controller psm1(1, node);
  psm_controller psm2(2, node);

  Eigen::Vector3d pt_in_psm1, pt_in_psm2, pt_in_lt_cam;
  pt_in_psm1 << -0.12, 0, -0.1;
  pt_in_lt_cam = rnNeedleDrivingPlanner.transformPointFromBaseToLtCamFrame(1, pt_in_psm1);
  pt_in_psm2 = rnNeedleDrivingPlanner.transformPointFromLtCamFrameToBase(2, pt_in_lt_cam);

  std::cout << std::endl << "pt_in_psm1: " << pt_in_psm1.transpose() << std::endl;
  std::cout << "pt_in_psm2: " << pt_in_psm2.transpose() << std::endl << std::endl;

  rnNeedleDrivingPlanner.goToLocationPointingDownFaceForward(psm1,  -0.12, 0.01, -0.1);
  rnNeedleDrivingPlanner.goToLocationPointingDownFaceForward(psm2,  0.12, -0.01, -0.1);

  while (ros::ok()) {
    double x1, x2;

    std::cout << "input psm1 x: (-)";
    std::cin >> x1;
    x1 = - x1;
    std::cout << std::endl << "input psm2 x: (+)";
    std::cin >> x2;

    rnNeedleDrivingPlanner.goToLocationPointingPsmLeft(psm1,  x1, 0.01, -0.1);
    rnNeedleDrivingPlanner.goToLocationPointingPsmRight(psm2,  x2, -0.01, -0.1);

  }





  rnNeedleDrivingPlanner.printGraspDepth();



  return 0;
}
