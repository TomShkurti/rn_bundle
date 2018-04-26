//
// Created by william on 28/03/18.
//


#include "rn_skills_as/RnNeedleDrivingPlanner.h"


int main(int argc, char **argv){

  ros::init(argc, argv, "rn_needle_driving_fncs_test");
  ros::NodeHandle node;

  RnNeedleDrivingPlanner rnNeedleDrivingPlanner(node);

  // nNeedleDrivingPlanner.runThisFun();

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


  std::cout << "Please type in a random char to proceed: ";
  std::cin >> random_char;

  eigen_grasp_transform = rnNeedleDrivingPlanner.getDefaultNeedleAffineWrtGraspOneFrame();

  // TODO delete
  ROS_WARN("Default Eigen Affine");
  rnNeedleDrivingPlanner.printEigenAffine(eigen_grasp_transform);

  default_grasp_transform = rnNeedleDrivingPlanner.convertEigenAffineToGeoTransform(eigen_grasp_transform);


  rnNeedleDrivingPlanner.setDefaultGraspTfSearchResolution(36);

//    test = rnNeedleDrivingPlanner.requestNeedleDrivingTrajectoryDefaultGraspInBaseFrame(arm,
//                                                                                      needle_entry_pt,
//                                                                                      needle_exit_pt,
//                                                                                      needleDriveTraj);

  test = rnNeedleDrivingPlanner.requestOneNeedleDrivingTrajectoryInBaseFrame(arm,
                                                                             needle_entry_pt,
                                                                             needle_exit_pt,
                                                                             needleDriveTraj,
                                                                             grasp_transform);

//  test = rnNeedleDrivingPlanner.requestOneNeedleDrivingTrajectoryInBaseFrame(arm,
//                                                                             needle_entry_pt,
//                                                                             needle_exit_pt,
//                                                                             default_grasp_transform,
//                                                                             needleDriveTraj);




  ROS_INFO("test result: %d", int(test));

  if (test) {

    grasp_tf = rnNeedleDrivingPlanner.convertGeoTransformStampedToEigenAffine(grasp_transform);

    ROS_INFO("grasp_transform: ");
    rnNeedleDrivingPlanner.printEigenAffine(grasp_tf);

    std::cout << std::endl;
    std::cout << "needleDriveTraj: " << std::endl << needleDriveTraj << std::endl;



    ROS_INFO("TEST SUBJECT 2: setTrajectoryVelocity()");

    double velocity = 0.01; // meter/sec

    rnNeedleDrivingPlanner.setTrajectoryVelocity(0.01, needleDriveTraj);

    ROS_WARN("\nINSPECT GRASP_TF");
    Eigen::Affine3d result_tf;
    result_tf = rnNeedleDrivingPlanner.convertGeoTransformStampedToEigenAffine(grasp_transform);
    rnNeedleDrivingPlanner.printEigenAffine(result_tf);



  }


  return 0;
}
