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

  bool test;
  int arm = 1;


  needle_entry_pt = rnNeedleDrivingPlanner.ConvertEigenVec3dToGeoMsgsPtStamped(pt_entry);
  needle_exit_pt = rnNeedleDrivingPlanner.ConvertEigenVec3dToGeoMsgsPtStamped(pt_exit);

  test = rnNeedleDrivingPlanner.requestNeedleDrivingTrajectoryDefaultGrasp(arm,
                                                                           needle_entry_pt,
                                                                           needle_exit_pt,
                                                                           needleDriveTraj);






  return 0;
}
