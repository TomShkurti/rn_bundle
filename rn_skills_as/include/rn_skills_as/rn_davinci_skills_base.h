//
// Created by William@HMS Prince of Wales on 28/03/18.
//

#ifndef RN_SKILLS_AS_RN_DAVINCI_SKILLS_BASE_H
#define RN_SKILLS_AS_RN_DAVINCI_SKILLS_BASE_H

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <string>
#include <math.h>

#include <tf/transform_listener.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/QuadWord.h>
#include <eigen3/Eigen/src/Geometry/Transform.h>

#include <cwru_davinci_kinematics/davinci_fwd_kinematics.h>
#include <cwru_davinci_kinematics/davinci_inv_kinematics.h>
#include <cwru_davinci_kinematics/davinci_kinematic_definitions.h>

#include <cwru_davinci_msgs/ArmIndex.h>
#include <cwru_davinci_msgs/ListOfTransformStamped.h>
#include <cwru_davinci_msgs/ListOfPointStamped.h>
#include <cwru_davinci_msgs/ListOfJointTrajectory.h>

#include <cwru_davinci_control/psm_controller.h>

#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int32MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>

typedef Eigen::Matrix<double, 7, 1> Vectorq7x1;

const double LCAMERA_TO_PSM_ONE_TRANSLATION[] = { -0.155, -0.03265, 0.0};
const double LCAMERA_TO_PSM_TWO_TRANSLATION[] = {0.145, -0.03265, 0.0};

namespace rn_davinci_skills {

struct NeedlePhis {
  int needle_op_id;

  // should get updated every time new entry and exit are provided.
  double phi_initial = 0;
  double phi_entry_pt;
  double phi_exit_pt;
  double phi_penetration;
  double phi_emergence;
  double phi_initial_insertion_lower_reference;
  double phi_initial_insertion_upper_reference;
  double phi_final_insertion_lower_reference;
  double phi_final_insertion_upper_reference;

  // should get updated every time a trajectory has been complete.
  double phi_needle_tip;
  double phi_needle_tail;

};


struct PmsKinematicAvailability {
  int id;

  int arm_index;
  int num_continuous_kinematic_ok_sections;
  double section_lower_limits[];
  double section_upper_limits[];

};

}





#endif //RN_SKILLS_AS_RN_DAVINCI_SKILLS_BASE_H
