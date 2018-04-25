//
// Created by William on 28/03/18.
//

//#include "../include/rn_skills_as/RnNeedleDrivingPlanner.h"

#include "rn_skills_as/RnNeedleDrivingPlanner.h"


RnNeedleDrivingPlanner::RnNeedleDrivingPlanner(const ros::NodeHandle &nodeHandle){

  ROS_INFO("Constructing a Needle Planner");

  tissue_normal_ << 0, 0, -1;

  if (!getNeedleParams()){
    throw std::runtime_error("Cannot load needle parameters");
  }

  computeDefaultNeedleWrtGripperTransform();

  getTransforms(); // no restriction implied here.


  // TODO bridge the HMI
  exit_pt_publisher_ = nh_.advertise<geometry_msgs::Point>("exit_points", 1);
  exit_pt_score_publisher_ = nh_.advertise<std_msgs::Int32MultiArray>("exit_points_score", 1);
  exit_pt_array_publisher_ = nh_.advertise<geometry_msgs::Polygon>("exit_point_array", 1);

}


bool RnNeedleDrivingPlanner::getNeedleParams(){

  Eigen::Matrix3d temp_rotation;
  Eigen::Vector3d temp_vec;


  if (nh_.getParam("/needle/radius", needle_radius_))
  {
    ROS_INFO("Needle Radius (%f) acquired from the param server.", needle_radius_);
  } else {
    ROS_ERROR("Could not load the Needle Radius from the ROS parameter server!");
    return false;
  }

  if (nh_.getParam("/needle/grasp_depth", grasp_depth_))
  {
    ROS_INFO("Needle Grasp Depth (%f) acquired from the param server.", grasp_depth_);
  } else {
    ROS_ERROR("Could not load the Needle Grasp Depth from the ROS parameter server!");
    return false;
  }

  if (nh_.getParam("/needle/height", needle_axis_ht_))
  {
    ROS_INFO("Needle Height (%f) acquired from the param server.", needle_axis_ht_);
  } else {
    ROS_ERROR("Could not load the Needle Height from the ROS parameter server!");
    return false;
  }

  suture_depth_ = needle_radius_ - needle_axis_ht_;

  dist_entrance_to_exit_ = 2 * sqrt(needle_radius_ * needle_radius_ - needle_axis_ht_ * needle_axis_ht_);

  // Default choice: needle origin in +y half space
  grab_needle_plus_minus_y_ = GRASP_W_NEEDLE_POSITIVE_GRIPPER_Y;
  // Default choice: needle z-axis parallel to gripper z axis
  grab_needle_plus_minus_z_ = GRASP_W_NEEDLE_POSITIVE_GRIPPER_Z;

  needle_origin_wrt_tissue_frame_ << 0.5 * dist_entrance_to_exit_, 0, needle_axis_ht_;
  needle_rotation_mat_wrt_tissue_frame_ << -1,  0,  0,
                                            0,  0, -1,
                                            0, -1,  0;

  initial_needle_affine_wrt_tissue_frame_.linear() = needle_rotation_mat_wrt_tissue_frame_;
  initial_needle_affine_wrt_tissue_frame_.translation() = needle_origin_wrt_tissue_frame_;

  temp_rotation << 1, 0, 0,
                   0, 1, 0,
                   0, 0, 1;
  temp_vec << 0, 0, -grasp_depth_;

  psm_1_grasp_frame_wrt_gripper_frame_.linear() = temp_rotation;
  psm_1_grasp_frame_wrt_gripper_frame_.translation() = temp_vec;

  psm_2_grasp_frame_wrt_gripper_frame_.linear() = temp_rotation;
  psm_2_grasp_frame_wrt_gripper_frame_.translation() = temp_vec;

  return true;

}


void RnNeedleDrivingPlanner::computeDefaultNeedleWrtGripperTransform(){

  Eigen::Vector3d nvec_needle_wrt_grasp_frame, tvec_needle_wrt_grasp_frame, bvec_needle_wrt_grasp_frame;

  /// PSM 1
  // Calculating Needle frame wrt Grasp frame
  needle_origin_wrt_grasp_one_frame_ << 0, grab_needle_plus_minus_y_ * needle_radius_, 0;
  bvec_needle_wrt_grasp_frame << 0, 0, grab_needle_plus_minus_z_; // z
  nvec_needle_wrt_grasp_frame << 0, grab_needle_plus_minus_y_, 0; // x
  tvec_needle_wrt_grasp_frame = bvec_needle_wrt_grasp_frame.cross(nvec_needle_wrt_grasp_frame); // y
  needle_rotation_wrt_grasp_one_frame_.col(0) = nvec_needle_wrt_grasp_frame;
  needle_rotation_wrt_grasp_one_frame_.col(1) = tvec_needle_wrt_grasp_frame;
  needle_rotation_wrt_grasp_one_frame_.col(2) = bvec_needle_wrt_grasp_frame;
  needle_affine_wrt_grasp_one_frame_.linear() = needle_rotation_wrt_grasp_one_frame_;
  needle_affine_wrt_grasp_one_frame_.translation() = needle_origin_wrt_grasp_one_frame_;
  default_needle_affine_wrt_grasp_one_frame_ = needle_affine_wrt_grasp_one_frame_;
  default_preferred_grasp_one_tf_ = default_needle_affine_wrt_grasp_one_frame_;
  // Calculating Needle frame wrt Gripper frame
  needle_affine_wrt_gripper_one_frame_ = psm_1_grasp_frame_wrt_gripper_frame_ * needle_affine_wrt_grasp_one_frame_;
  needle_origin_wrt_gripper_one_frame_ = needle_affine_wrt_gripper_one_frame_.translation();
  needle_rotation_wrt_gripper_one_frame_ = needle_affine_wrt_gripper_one_frame_.linear();

  /// PSM 2
  // TODO Or should it be exactly the same as the PSM1?
  // Calculating Needle frame wrt Grasp frame
  needle_origin_wrt_grasp_two_frame_ << 0, -grab_needle_plus_minus_y_ * needle_radius_, 0;
  bvec_needle_wrt_grasp_frame << 0, 0, -grab_needle_plus_minus_z_; // z
  nvec_needle_wrt_grasp_frame << 0, -grab_needle_plus_minus_y_, 0; // x
  tvec_needle_wrt_grasp_frame = bvec_needle_wrt_grasp_frame.cross(nvec_needle_wrt_grasp_frame); // y
  needle_rotation_wrt_grasp_two_frame_.col(0) = nvec_needle_wrt_grasp_frame;
  needle_rotation_wrt_grasp_two_frame_.col(1) = tvec_needle_wrt_grasp_frame;
  needle_rotation_wrt_grasp_two_frame_.col(2) = bvec_needle_wrt_grasp_frame;
  needle_affine_wrt_grasp_two_frame_.linear() = needle_rotation_wrt_grasp_two_frame_;
  needle_affine_wrt_grasp_two_frame_.translation() = needle_origin_wrt_grasp_two_frame_;
  default_needle_affine_wrt_grasp_two_frame_ = needle_affine_wrt_grasp_two_frame_;
  default_preferred_grasp_two_tf_ = default_needle_affine_wrt_grasp_two_frame_;
  // Calculating Needle frame wrt Gripper frame
  needle_affine_wrt_gripper_two_frame_ = psm_2_grasp_frame_wrt_gripper_frame_ * needle_affine_wrt_grasp_two_frame_;
  needle_origin_wrt_gripper_two_frame_ = needle_affine_wrt_gripper_two_frame_.translation();
  needle_rotation_wrt_gripper_two_frame_ = needle_affine_wrt_gripper_two_frame_.linear();

  ROS_INFO("Needle Gripper transforms acquired.");

}


bool RnNeedleDrivingPlanner::getTransforms(){

  return (getCameraToPSMsTransforms() && getCameraToGripperTransforms());

}


bool RnNeedleDrivingPlanner::getCameraToPSMsTransforms(){

  tf::TransformListener tfListener;
  tf::StampedTransform tfResult_one, tfResult_two;

  bool tf_acquired = false;
  int n_tries = 0;

  ROS_INFO("Attempting to get Camera PSMs transforms");

  while (!tf_acquired){
    if (n_tries > 5) break;
    tf_acquired = true;
    try {
      tfListener.lookupTransform("left_camera_optical_frame",
                                 "one_psm_base_link",
                                 ros::Time(0),
                                 tfResult_one);
      tfListener.lookupTransform("left_camera_optical_frame",
                                 "two_psm_base_link",
                                 ros::Time(0),
                                 tfResult_two);
    } catch (tf::TransformException &exception){
      ROS_WARN("%s", exception.what());
      tf_acquired = false;
      ros::Duration(0.5).sleep();
      ros::spinOnce();
      n_tries++;
    }
  }

  if (!tf_acquired) {
    ROS_ERROR("Cannot acquire Camera PSMs transforms transform. Will NOT use default.");
    return false;
  } else {
    ROS_INFO("Transform Info acquired.");

    psm_one_affine_wrt_lt_camera_ = convertTFToEigen(tfResult_one);
    psm_two_affine_wrt_lt_camera_ = convertTFToEigen(tfResult_two);

    ROS_WARN("Checking the transforms:");
    printEigenAffine(psm_one_affine_wrt_lt_camera_);
    printEigenAffine(psm_two_affine_wrt_lt_camera_);

  }


}


bool RnNeedleDrivingPlanner::getCameraToGripperTransforms(){

  tf::TransformListener tfListener;
  tf::StampedTransform tfResult_one, tfResult_two;

  bool tf_acquired = false;
  int n_tries = 0;

  ROS_INFO("Attempting to get Camera PSMs transforms");

  while (!tf_acquired){
    if (n_tries > 5) break;
    tf_acquired = true;
    try {
      tfListener.lookupTransform("left_camera_optical_frame",
                                 "one_tool_tip_link",
                                 ros::Time(0),
                                 tfResult_one);
      tfListener.lookupTransform("left_camera_optical_frame",
                                 "two_tool_tip_link",
                                 ros::Time(0),
                                 tfResult_two);
    } catch (tf::TransformException &exception){
      ROS_WARN("%s", exception.what());
      tf_acquired = false;
      ros::Duration(0.5).sleep();
      ros::spinOnce();
      n_tries++;
    }
  }

  if (!tf_acquired) {
    ROS_ERROR("Cannot acquire Camera PSMs transforms transform. Will NOT use default.");
    return false;
  } else {
    ROS_INFO("Transform Info acquired.");

    gripper_one_affine_wrt_lt_camera_ = convertTFToEigen(tfResult_one);
    gripper_two_affine_wrt_lt_camera_ = convertTFToEigen(tfResult_two);

    ROS_WARN("Checking the transforms:");
    printEigenAffine(gripper_one_affine_wrt_lt_camera_);
    printEigenAffine(gripper_two_affine_wrt_lt_camera_);

  }


}


void RnNeedleDrivingPlanner::defineTissueFrameWrtLtCamera(Eigen::Vector3d entry_pt,
                                                          Eigen::Vector3d exit_pt,
                                                          Eigen::Vector3d tissue_normal){

  Eigen::Vector3d nvec_tissue_frame_wrt_camera;
  Eigen::Vector3d tvec_tissue_frame_wrt_camera;
  Eigen::Vector3d bvec_tissue_frame_wrt_camera;

  bvec_tissue_frame_wrt_camera = tissue_normal;
  nvec_tissue_frame_wrt_camera = (exit_pt - entry_pt);

  if (nvec_tissue_frame_wrt_camera.norm() < 0.001){
    ROS_WARN("The exit and entry points are too close.");
  }

  nvec_tissue_frame_wrt_camera = nvec_tissue_frame_wrt_camera/nvec_tissue_frame_wrt_camera.norm();
  tvec_tissue_frame_wrt_camera = bvec_tissue_frame_wrt_camera.cross(nvec_tissue_frame_wrt_camera);

  tissue_rotation_wrt_lt_camera_.col(0) = nvec_tissue_frame_wrt_camera;
  tissue_rotation_wrt_lt_camera_.col(0) = tvec_tissue_frame_wrt_camera;
  tissue_rotation_wrt_lt_camera_.col(0) = bvec_tissue_frame_wrt_camera;

  tissue_affine_wrt_lt_camera_.linear() = tissue_rotation_wrt_lt_camera_;
  tissue_affine_wrt_lt_camera_.translation() = entry_pt;

  ROS_INFO("Tissue Frame has been defined with new entry and exit points.");

}


void RnNeedleDrivingPlanner::computeGraspTransform(int arm_index, double phi_x, double phi_y){

  Eigen::Matrix3d new_needle_rotation_wrt_grasp_one, new_needle_rotation_wrt_grasp_two, Rx, Ry;
  Eigen::Vector3d new_needle_origin_wrt_grasp_one, new_needle_origin_wrt_grasp_two;

  Rx = Rotx(phi_x);
  Ry = Roty(phi_y);

  switch (arm_index) {

    case 1:

      new_needle_rotation_wrt_grasp_one = Rx * Ry * needle_rotation_wrt_grasp_one_frame_;
      new_needle_origin_wrt_grasp_one = Rx * needle_origin_wrt_grasp_one_frame_;
      needle_affine_wrt_grasp_one_frame_.linear() = new_needle_rotation_wrt_grasp_one;
      needle_affine_wrt_grasp_one_frame_.translation() = new_needle_origin_wrt_grasp_one;
      needle_affine_wrt_gripper_one_frame_ =
          psm_1_grasp_frame_wrt_gripper_frame_ * needle_affine_wrt_grasp_one_frame_;

      break;

    case 2:

      new_needle_rotation_wrt_grasp_two = Rx * Ry * needle_rotation_wrt_grasp_two_frame_;
      new_needle_origin_wrt_grasp_two = Rx * needle_origin_wrt_grasp_two_frame_;
      needle_affine_wrt_grasp_two_frame_.linear() = new_needle_rotation_wrt_grasp_two;
      needle_affine_wrt_grasp_two_frame_.translation() = new_needle_origin_wrt_grasp_two;
      needle_affine_wrt_gripper_two_frame_ =
          psm_2_grasp_frame_wrt_gripper_frame_ * needle_affine_wrt_grasp_two_frame_;

      break;

  }

}


void RnNeedleDrivingPlanner::computeGraspTransform(int arm_index,
                                                   const geometry_msgs::TransformStamped &grasp_transform) {

  Eigen::Quaterniond q;
  Eigen::Matrix3d R;

  q.x() = grasp_transform.transform.rotation.x;
  q.y() = grasp_transform.transform.rotation.y;
  q.z() = grasp_transform.transform.rotation.z;
  q.w() = grasp_transform.transform.rotation.w;
  R = q.toRotationMatrix();

  switch(arm_index) {

    case 1:

      needle_affine_wrt_grasp_one_frame_.linear() = R;
      needle_affine_wrt_grasp_one_frame_.translation() << grasp_transform.transform.translation.x,
          grasp_transform.transform.translation.y,
          grasp_transform.transform.translation.z;
      needle_affine_wrt_gripper_one_frame_ =
          psm_1_grasp_frame_wrt_gripper_frame_ * needle_affine_wrt_grasp_one_frame_;

      break;

    case 2:

      needle_affine_wrt_grasp_two_frame_.linear() = R;
      needle_affine_wrt_grasp_two_frame_.translation() << grasp_transform.transform.translation.x,
          grasp_transform.transform.translation.y,
          grasp_transform.transform.translation.z;
      needle_affine_wrt_gripper_two_frame_ =
          psm_2_grasp_frame_wrt_gripper_frame_ * needle_affine_wrt_grasp_two_frame_;
      break;

  }

  ROS_INFO("PSM_%d needle - grasp transform updated.", arm_index);

}


double RnNeedleDrivingPlanner::computeNeedleDriveGripperAffines(int arm_index,
                                                                trajectory_msgs::JointTrajectory &needleDriveTraj){

  double fraction;
  /// whichever gripper the user has selected
  Eigen::Affine3d des_gripper_one_wrt_base;
  Eigen::Affine3d des_gripper_two_wrt_base;

  Eigen::Affine3d des_gripper_one_affine_wrt_lt_camera;
  Eigen::Affine3d des_gripper_two_affine_wrt_lt_camera;
  Eigen::Affine3d des_gripper_one_affine_wrt_tissue;
  Eigen::Affine3d des_gripper_two_affine_wrt_tissue;

  std::vector<Eigen::Affine3d> des_gripper_one_affines_wrt_lt_camera;
  std::vector<Eigen::Affine3d> des_gripper_two_affines_wrt_lt_camera;

  Eigen::Vector3d kvec_needle; // needle rotation axis w/rt tissue frame after needle tilting
  Eigen::Matrix3d Rot_needle;

  int nsolns = 0;

  /// phi is the angle rotated about the needle z axis while performing needle drives
  double delta_phi = M_PI/(2*(path_waypoints_-1));
  double phi_insertion = 0;

  /// Needle calculation
  needle_affine_wrt_tissue_frame_ = initial_needle_affine_wrt_tissue_frame_;

  needle_affine_wrt_tissue_frame_.linear() =
      Rotx(psi_needle_axis_tilt_wrt_tissue_) * needle_affine_wrt_tissue_frame_.linear();
  needle_affine_wrt_tissue_frame_.translation() =
      Rotx(psi_needle_axis_tilt_wrt_tissue_) * needle_affine_wrt_tissue_frame_.translation();

  needle_origin_wrt_tissue_frame_ = needle_affine_wrt_tissue_frame_.translation();
  needle_rotation_mat_wrt_tissue_frame_ = needle_affine_wrt_tissue_frame_.linear();

  kvec_needle = needle_affine_wrt_tissue_frame_.linear().col(2);

  switch (arm_index) {

    case 1:

      for (int ipose = 0; ipose < path_waypoints_; ipose++){
        Rot_needle = Rot_k_phi(kvec_needle, phi_insertion);
        needle_affine_wrt_tissue_frame_.linear() = Rot_needle * needle_rotation_mat_wrt_tissue_frame_;

        /// Gripper Calculation
        des_gripper_one_affine_wrt_tissue =
            needle_affine_wrt_tissue_frame_ * needle_affine_wrt_gripper_one_frame_.inverse();
        des_gripper_one_affine_wrt_lt_camera =
            tissue_affine_wrt_lt_camera_ * des_gripper_one_affine_wrt_tissue;
        des_gripper_one_wrt_base =
            psm_one_affine_wrt_lt_camera_.inverse() * des_gripper_one_affine_wrt_lt_camera;


        if (ik_solver_.ik_solve(des_gripper_one_wrt_base)) {
          nsolns++;
          des_gripper_one_affines_wrt_lt_camera.push_back(des_gripper_one_wrt_base);
          ik_ok_array_(ipose) = 1;
        } else {
          ik_ok_array_(ipose) = 0;
        }
        phi_insertion += delta_phi;
      }

      convertAffinesToTrajectoryMsgs(des_gripper_one_affines_wrt_lt_camera, needleDriveTraj);

      break;

    case 2:

      for (int ipose = 0; ipose < path_waypoints_; ipose++){
        Rot_needle = Rot_k_phi(kvec_needle, phi_insertion);
        needle_affine_wrt_tissue_frame_.linear() = Rot_needle * needle_rotation_mat_wrt_tissue_frame_;

        /// Gripper Calculation
        des_gripper_two_affine_wrt_tissue =
            needle_affine_wrt_tissue_frame_ * needle_affine_wrt_gripper_two_frame_.inverse();
        des_gripper_two_affine_wrt_lt_camera =
            tissue_affine_wrt_lt_camera_ * des_gripper_two_affine_wrt_tissue;
        des_gripper_two_wrt_base =
            psm_two_affine_wrt_lt_camera_.inverse() * des_gripper_two_affine_wrt_lt_camera;


        if (ik_solver_.ik_solve(des_gripper_two_wrt_base)) {
          nsolns++;
          des_gripper_two_affines_wrt_lt_camera.push_back(des_gripper_two_wrt_base);
          ik_ok_array_(ipose) = 1;
        } else {
          ik_ok_array_(ipose) = 0;
        }
        phi_insertion += delta_phi;
      }

      convertAffinesToTrajectoryMsgs(des_gripper_two_affines_wrt_lt_camera, needleDriveTraj);

      break;

  }

  std::cout << "ik_ok_point:: " << ik_ok_array_.transpose() << std::endl;

  fraction = nsolns/path_waypoints_;

  if (fraction != 0) {
    ROS_ERROR("Fraction of IK solutions is: %f", fraction);
  }

  return fraction;

}


/*
 * The trajectory is 7x1 and time from start is NOT filled in.
 */
void RnNeedleDrivingPlanner::convertAffinesToTrajectoryMsgs(const std::vector<Eigen::Affine3d> &gripper_affines_wrt_portal,
                                                            trajectory_msgs::JointTrajectory &joint_trajectory) {

  int size;
  Vectorq7x1 q_vec1;
  trajectory_msgs::JointTrajectoryPoint joint_trajectory_point;

  joint_trajectory.points.clear();
  joint_trajectory.joint_names.clear();
  joint_trajectory.header.stamp = ros::Time::now();
  joint_trajectory_point.positions.resize(7);
  // size = gripper_affines_wrt_camera_.size();
  size = gripper_affines_wrt_portal.size();
  for (int n = 0; n < size; n++)
  {
    joint_trajectory_point.positions.clear();
    ik_solver_.ik_solve(gripper_affines_wrt_portal[n]);
    q_vec1 = ik_solver_.get_soln();
    q_vec1(6) = 0;
    for (int i = 0; i < 7; i++)
    {
      joint_trajectory_point.positions[i] = q_vec1(i);
    }
    joint_trajectory_point.time_from_start = ros::Duration(n + 5);
    joint_trajectory.points.push_back(joint_trajectory_point);
  }

  ROS_INFO("Joint Trajectory Generated.");


}


void RnNeedleDrivingPlanner::updateNeedleAndTissueParameters(const geometry_msgs::PointStamped &needle_entry_pt,
                                                             const geometry_msgs::PointStamped &needle_exit_pt) {

  double temp1, temp2, temp3;
  Eigen::Vector3d needle_origin_wrt_tissue_frame;
  Eigen::Quaterniond q;
  Eigen::Matrix3d R;

  needle_entry_point_ = convertPointStampedToEigenVector(needle_entry_pt);
  ROS_INFO("Needle entry point has been updated.");
  needle_exit_point_ = convertPointStampedToEigenVector(needle_exit_pt);
  ROS_INFO("Needle exit point has been updated.");

  // Make sure the exit point is valid, if not, adjust it

  checkExitPoint(needle_entry_point_, needle_exit_point_, dist_entrance_to_exit_);

  defineTissueFrameWrtLtCamera(needle_entry_point_, needle_exit_point_, tissue_normal_);
  ROS_INFO("Tissue Frame definition has been updated.");

  needle_axis_ht_ = sqrt(needle_radius_ * needle_radius_ - (dist_entrance_to_exit_ / 2) * (dist_entrance_to_exit_ / 2));
  suture_depth_ = needle_radius_ - needle_axis_ht_;
  ROS_INFO("Needle axis height/suture depth has been updated.");

  needle_origin_wrt_tissue_frame << 0.5 * dist_entrance_to_exit_, 0, needle_axis_ht_;
  initial_needle_affine_wrt_tissue_frame_.translation() = needle_origin_wrt_tissue_frame;
  ROS_INFO("Needle Tissue Transform has been updated.");

}



/// Interface for Applications

/*
 * Use default grasp transform and attempt a needle driving trajectory
 * with given entry and exit points.
 */
bool RnNeedleDrivingPlanner::requestNeedleDrivingTrajectoryDefaultGrasp(const int arm_index,
                                                                        const geometry_msgs::PointStamped &needle_entry_pt,
                                                                        const geometry_msgs::PointStamped &needle_exit_pt,
                                                                        trajectory_msgs::JointTrajectory &needleDriveTraj) {

  double ik_fraction;

  updateNeedleAndTissueParameters(needle_entry_pt, needle_exit_pt);
  needle_affine_wrt_grasp_one_frame_ = default_needle_affine_wrt_grasp_one_frame_;
  needle_affine_wrt_grasp_two_frame_ = default_needle_affine_wrt_grasp_two_frame_;
  updateNeedleWrtGripperTransforms();
  ik_fraction = computeNeedleDriveGripperAffines(arm_index, needleDriveTraj);

  if (ik_fraction == 1) {
    ROS_INFO("A valid needle drive trajectory has been acquired.");
    return true;
  } else {
    ROS_WARN("Failed to get a valid needle drive trajectory, the ik solution fraction is: %f", ik_fraction);
    return false;
  }

}


/*
 * Use user defined grasp transform to attempt a needle driving trajectory
 * with given entry and exit points.
 */
bool RnNeedleDrivingPlanner::requestOneNeedleDrivingTrajectory(const int arm_index,
                                                               const geometry_msgs::PointStamped &needle_entry_pt,
                                                               const geometry_msgs::PointStamped &needle_exit_pt,
                                                               const geometry_msgs::TransformStamped &grasp_transform,
                                                               trajectory_msgs::JointTrajectory &needleDriveTraj) {

  double ik_fraction;

  updateNeedleAndTissueParameters(needle_entry_pt, needle_exit_pt);

  computeGraspTransform(arm_index, grasp_transform);
  updateNeedleWrtGripperTransforms();

  ik_fraction = computeNeedleDriveGripperAffines(arm_index, needleDriveTraj);

  if (ik_fraction == 1) {
    ROS_INFO("A valid needle drive trajectory has been acquired.");
    return true;
  } else {
    ROS_WARN("Failed to get a valid needle drive trajectory, the ik solution fraction is: %f", ik_fraction);
    return false;
  }

}


/*
 * Find a trajectory which allows a needle driving from the entry to exit requested by the user
 * Auto-search for a grasp transform that works.
 */
bool RnNeedleDrivingPlanner::requestOneNeedleDrivingTrajectory(const int arm_index,
                                                               const geometry_msgs::PointStamped &needle_entry_pt,
                                                               const geometry_msgs::PointStamped &needle_exit_pt,
                                                               trajectory_msgs::JointTrajectory &needleDriveTraj,
                                                               geometry_msgs::TransformStamped &grasp_transform) {

  cwru_davinci_msgs::ListOfTransformStamped potential_transform_list;
  cwru_davinci_msgs::ListOfTransformStamped valid_transform_list;
  Eigen::Affine3d default_preferred_grasp_tf;
  double ik_fraction;
  double grasp_trials_fraction;
  int ngrasps = 0;

  switch (arm_index) {
    case 1:
      default_preferred_grasp_tf = default_preferred_grasp_one_tf_;
      break;

    case 2:
      default_preferred_grasp_tf = default_preferred_grasp_two_tf_;
      break;
  }

  // default_grasp_tf_search_resolution_ is 36;
  generateGraspTransformList(arm_index, default_grasp_tf_search_resolution_, potential_transform_list);

  for (int i = 0; i < default_grasp_tf_search_resolution_; i++) {

    computeGraspTransform(arm_index, potential_transform_list.stamped_transform_list[i]);
    updateNeedleWrtGripperTransforms();
    ik_fraction = computeNeedleDriveGripperAffines(arm_index, needleDriveTraj);
    if (ik_fraction == 1) {
      valid_transform_list.stamped_transform_list.push_back(potential_transform_list.stamped_transform_list[i]);
      ngrasps ++;
    }

  }

  if (ngrasps > 0) {

    ROS_INFO("There are %d valid grasp transforms obtained. Entering selection process.", ngrasps);

    grasp_transform = getBestGraspTransform(default_preferred_grasp_tf, valid_transform_list);

    // Then calculate again with the selected grasp tf
    computeGraspTransform(arm_index, grasp_transform);
    updateNeedleWrtGripperTransforms();

    ik_fraction = computeNeedleDriveGripperAffines(arm_index, needleDriveTraj);

    // Should definitely pass the test, again.
    if (ik_fraction == 1) {
      ROS_INFO("A valid needle drive trajectory has been acquired.");
      return true;
    } else {
      ROS_WARN("Failed to get a valid needle drive trajectory, the ik solution fraction is: %f", ik_fraction);
      return false;
    }

  } else {

    ROS_ERROR("Could not find any grasp transform that suffices the entry & exit pts to do a needle drive.");
    return false;

  }


}



/*
 * Generate a list of grasp transforms (needle w/rt gripper) for further evaluation.
 */
void RnNeedleDrivingPlanner::generateGraspTransformList(const int arm_index,
                                                        const int resolution,
                                                        cwru_davinci_msgs::ListOfTransformStamped &grasp_tf_array) {

  double needle_x, needle_y;
  geometry_msgs::TransformStamped grasp_transform_temp;
  Eigen::Matrix3d R;
  needle_y = 0;
  // for (needle_x = 0; needle_x < 0.7854; needle_x += 0.1)
  // for (needle_x = 0; needle_x < (2 * M_PI); needle_x += 0.1)

  switch (arm_index) {

    case 1:

      for (needle_x = 0; needle_x < (2 * M_PI); needle_x += 0.1) {
        computeGraspTransform(1, needle_x, needle_y);
        grasp_transform_temp.transform.translation.x = needle_affine_wrt_gripper_one_frame_(0, 3);
        grasp_transform_temp.transform.translation.y = needle_affine_wrt_gripper_one_frame_(1, 3);
        grasp_transform_temp.transform.translation.z = needle_affine_wrt_gripper_one_frame_(2, 3);
        R = needle_affine_wrt_gripper_one_frame_.linear();
        Eigen::Quaterniond q1(R);
        grasp_transform_temp.transform.rotation.x = q1.x();
        grasp_transform_temp.transform.rotation.y = q1.y();
        grasp_transform_temp.transform.rotation.z = q1.z();
        grasp_transform_temp.transform.rotation.w = q1.w();
        grasp_transform_temp.child_frame_id = "left_camera_optical_frame";
        grasp_transform_temp.header.stamp = ros::Time::now();
        grasp_tf_array.stamped_transform_list.push_back(grasp_transform_temp);
      }

      break;


    case 2:

      for (needle_x = 0; needle_x < (2 * M_PI); needle_x += 0.1) {
        computeGraspTransform(2, needle_x, needle_y);
        grasp_transform_temp.transform.translation.x = needle_affine_wrt_gripper_two_frame_(0, 3);
        grasp_transform_temp.transform.translation.y = needle_affine_wrt_gripper_two_frame_(1, 3);
        grasp_transform_temp.transform.translation.z = needle_affine_wrt_gripper_two_frame_(2, 3);
        R = needle_affine_wrt_gripper_two_frame_.linear();
        Eigen::Quaterniond q2(R);
        grasp_transform_temp.transform.rotation.x = q2.x();
        grasp_transform_temp.transform.rotation.y = q2.y();
        grasp_transform_temp.transform.rotation.z = q2.z();
        grasp_transform_temp.transform.rotation.w = q2.w();
        grasp_transform_temp.child_frame_id = "left_camera_optical_frame";
        grasp_transform_temp.header.stamp = ros::Time::now();
        grasp_tf_array.stamped_transform_list.push_back(grasp_transform_temp);
      }

      break;

  }


}



/*
 * Generate a list of exit points (w/rt left camera frame) for further evaluation.
 */
void RnNeedleDrivingPlanner::generateExitPoints(const geometry_msgs::PointStamped &needle_entry_pt,
                                                const double suture_depth,
                                                cwru_davinci_msgs::ListOfPointStamped &potential_exit_points_array) {

  ROS_INFO("Generating Potential Exit Points.");

  double kvec_yaw = 0;
  int i = 0;
  Eigen::Vector3d entry_pt;
  Eigen::Vector3d exit_pt;
  Eigen::Vector3d v_entrance_to_exit;
  Eigen::Vector3d v_entrance_to_exit0;
  geometry_msgs::PointStamped exit_pt_stamped_temp;

  v_entrance_to_exit0 << 0, -1, 0;

  entry_pt = convertPointStampedToEigenVector(needle_entry_pt);

  // TODO the step is now fixed. Better make it a variable.
  for (kvec_yaw = 0; kvec_yaw < (M_PI*2); kvec_yaw += 0.1) // before debug was (M_PI/2)
  {
    v_entrance_to_exit = Rotz(kvec_yaw) * v_entrance_to_exit0;
    exit_pt = entry_pt + dist_entrance_to_exit_ * v_entrance_to_exit;
    exit_pt_stamped_temp = ConvertEigenVec3dToGeoMsgsPtStamped(exit_pt);
    potential_exit_points_array.stamped_point_list.push_back(exit_pt_stamped_temp);
  }

}




/*
 * Filter the valid exit points provided in an input array. Use multiple grasp transforms.
 */
double RnNeedleDrivingPlanner::filterValidExitPoints(const int arm_index,
                                                     const geometry_msgs::PointStamped &needle_entry_pt,
                                                     const cwru_davinci_msgs::ListOfPointStamped &exit_points_array,
                                                     cwru_davinci_msgs::ListOfPointStamped &valid_exit_points_array) {

  double valid_exit_fraction;
  int valid_count = 0;
  // They are only containers, not used for passing info.
  geometry_msgs::TransformStamped grasp_transform;
  trajectory_msgs::JointTrajectory needleDriveTraj;

  int n_candidates = exit_points_array.stamped_point_list.size();

  for (int n = 0; n < n_candidates; n++) {

    if (requestOneNeedleDrivingTrajectory(arm_index,
                                          needle_entry_pt,
                                          exit_points_array.stamped_point_list[n],
                                          needleDriveTraj,
                                          grasp_transform)) {

      valid_count ++;
      valid_exit_points_array.stamped_point_list.push_back(exit_points_array.stamped_point_list[n]);

    }

  }

  valid_exit_fraction = valid_count/n_candidates;

  ROS_INFO("Among %d candidate exit points, there are %d valid ones. The fraction is %f.",
           n_candidates, valid_count, valid_exit_fraction);

  return valid_exit_fraction;

}


/*
 * Filter the valid exit points provided in an input array. Use specified grasp transform.
 */
double RnNeedleDrivingPlanner::filterValidExitPoints(const int arm_index,
                                                     const geometry_msgs::PointStamped &needle_entry_pt,
                                                     const geometry_msgs::TransformStamped grasp_tf,
                                                     const cwru_davinci_msgs::ListOfPointStamped &exit_points_array,
                                                     cwru_davinci_msgs::ListOfPointStamped &valid_exit_points_array) {

  double valid_exit_fraction;
  int valid_count = 0;

  // They are only containers, not used for passing info.
  geometry_msgs::TransformStamped grasp_transform;
  trajectory_msgs::JointTrajectory needleDriveTraj;

  int n_candidates = exit_points_array.stamped_point_list.size();

  for (int n = 0; n < n_candidates; n++) {

    if (requestOneNeedleDrivingTrajectory(arm_index,
                                         needle_entry_pt,
                                         exit_points_array.stamped_point_list[n],
                                         grasp_tf,
                                         needleDriveTraj)) {

      valid_count ++;
      valid_exit_points_array.stamped_point_list.push_back(exit_points_array.stamped_point_list[n]);

    }

  }

  valid_exit_fraction = valid_count/n_candidates;

  ROS_INFO("Among %d candidate exit points, there are %d valid ones. The fraction is %f.",
           n_candidates, valid_count, valid_exit_fraction);

  return valid_exit_fraction;

}



/*
 * Filter the valid exit points provided in an input array. Use default grasp transform.
 */
double RnNeedleDrivingPlanner::filterValidExitPointsDefaultGrasp(const int arm_index,
                                                                 const geometry_msgs::PointStamped &needle_entry_pt,
                                                                 const cwru_davinci_msgs::ListOfPointStamped &exit_points_array,
                                                                 cwru_davinci_msgs::ListOfPointStamped &valid_exit_points_array) {

  double valid_exit_fraction;
  int valid_count = 0;
  // They are only containers, not used for passing info.
  geometry_msgs::TransformStamped grasp_transform;
  trajectory_msgs::JointTrajectory needleDriveTraj;

  int n_candidates = exit_points_array.stamped_point_list.size();

  for (int n = 0; n < n_candidates; n++) {

    if (requestNeedleDrivingTrajectoryDefaultGrasp(arm_index,
                                                   needle_entry_pt,
                                                   exit_points_array.stamped_point_list[n],
                                                   needleDriveTraj)) {

      valid_count ++;
      valid_exit_points_array.stamped_point_list.push_back(exit_points_array.stamped_point_list[n]);

    }

  }

  valid_exit_fraction = valid_count/n_candidates;

  ROS_INFO("Among %d candidate exit points, there are %d valid ones. The fraction is %f.",
           n_candidates, valid_count, valid_exit_fraction);

  return valid_exit_fraction;

}



void RnNeedleDrivingPlanner::setTrajectoryVelocity(double velocity,
                                                   trajectory_msgs::JointTrajectory &needleDriveTraj) {

  // Use Linear Estimation to get the time set according to requested velocity.

  double euler_dist;
  Eigen::Affine3d affine_gripper_wrt_base;
  Eigen::Vector3d pt0, pt1;
  double temp1, temp2, temp3;
  Vectorq7x1 q_vec0, q_vec1;
  double time_from_start, time_0, delta_t;

  time_0 = 7;
  needleDriveTraj.points[0].time_from_start = ros::Duration(time_0);

  int waypoints = needleDriveTraj.points.size();

  for (int n = 0; n < (waypoints-1); n++) {

    for (int i = 0; i < 7; i++) {
      q_vec0(i) = needleDriveTraj.points[n].positions[i];
    }

    affine_gripper_wrt_base = fwd_solver_.fwd_kin_solve(q_vec0);
    pt0 = affine_gripper_wrt_base.translation();

    for (int i = 0; i < 7; i++) {
      q_vec1(i) = needleDriveTraj.points[n+1].positions[i];
    }

    affine_gripper_wrt_base = fwd_solver_.fwd_kin_solve(q_vec1);
    pt1 = affine_gripper_wrt_base.translation();

    temp1 = pt0.transpose() * pt0;
    temp2 = pt1.transpose() * pt1;
    temp3 = temp1 - temp2;
    euler_dist = sqrt(abs(temp3));

    delta_t = euler_dist/velocity;

    time_from_start = time_from_start + delta_t;

    needleDriveTraj.points[n+1].time_from_start = ros::Duration(time_from_start);

  }

  ROS_INFO("All points in this trajectory has their time_from_start set according to the given velocity %f",
           velocity);

}



/// Adjustment & Correction

/*
 * Set a preferred grasp tf, compare (by eular angle diff) all the tf options in the provided array, select
 * the one that match best.
 */
geometry_msgs::TransformStamped RnNeedleDrivingPlanner::getBestGraspTransform(Eigen::Affine3d preferred_tf,
                                                                              const cwru_davinci_msgs::ListOfTransformStamped &grasp_tf_array) {

  // ONLY consider rotaional difference

  int array_size = grasp_tf_array.stamped_transform_list.size();
  Eigen::Affine3d candidate_tf;
  Eigen::Matrix3d R1,R2,R_err;

  R1 = preferred_tf.linear();
  double dtheta = 99;
  int winner_index;

  for (int n = 0; n < array_size; n ++) {

    candidate_tf = convertGeoTransformStampedToEigenAffine(grasp_tf_array.stamped_transform_list[n]);

    R2 = candidate_tf.linear();
    R_err = R2*R1.transpose();
    Eigen::AngleAxisd angleAxis(R_err);

    if (dtheta > angleAxis.angle()) {

      dtheta = angleAxis.angle();
      winner_index = n;

    }

  }

  return grasp_tf_array.stamped_transform_list[winner_index];

}



/*
 * To make sure the entry-exit distance is less than the needle diameter. This function is called
 * internally by another member function updateNeedleAndTissueParameters().
 */
void RnNeedleDrivingPlanner::checkExitPoint(const Eigen::Vector3d& entry_pt,
                                            Eigen::Vector3d& exit_pt,
                                            double& dist_entry_exit) {

  double temp1, temp2, temp3;
  Eigen::Vector3d direction;

  direction = exit_pt - entry_pt;
  direction = direction/direction.norm();

  temp1 = exit_pt.transpose() * exit_pt;
  temp2 = entry_pt.transpose() * entry_pt;
  temp3 = temp1 - temp2;

  dist_entry_exit = sqrt(abs(temp3));

  if (dist_entry_exit >= 2*needle_radius_) {
    ROS_WARN("The entry exit distance (%f) is greater than the needle diameter. It will be adjusted!",
             dist_entry_exit);

    // Update exit point and distance
    exit_pt = entry_pt + direction * 1.8 * needle_radius_;
    temp1 = exit_pt.transpose() * exit_pt;
    temp3 = temp1 - temp2;
    dist_entry_exit = sqrt(abs(temp3));

  } else {
    ROS_INFO("Entry and Exit points check passed.");
  }


}



/// Camera Free Needle Drive Interface

void RnNeedleDrivingPlanner::setHardCodedTransforms(Eigen::Affine3d psm_one_affine_wrt_lt_camera,
                                                    Eigen::Affine3d psm_two_affine_wrt_lt_camera) {

  psm_one_affine_wrt_lt_camera_ = psm_one_affine_wrt_lt_camera;
  psm_two_affine_wrt_lt_camera_ = psm_two_affine_wrt_lt_camera;

}


void RnNeedleDrivingPlanner::overlapLeftCamFrameAndPsmBase(const int arm_index) {


  Eigen::Matrix3d R;
  R.setIdentity();

  switch (arm_index) {

    case 1:

      psm_one_affine_wrt_lt_camera_.translation() << 0, 0, 0;
      psm_one_affine_wrt_lt_camera_.linear() = R;

      break;

    case 2:

      psm_two_affine_wrt_lt_camera_.translation() << 0, 0, 0;
      psm_two_affine_wrt_lt_camera_.linear() = R;

      break;

  }

  ROS_WARN("The Left Camera Frame to PSM %d Base Frame are now set to be identical.", arm_index);

}


bool RnNeedleDrivingPlanner::requestNeedleDrivingTrajectoryDefaultGraspInBaseFrame(const int arm_index,
                                                                                   const geometry_msgs::PointStamped &needle_entry_pt_wrt_base,
                                                                                   const geometry_msgs::PointStamped &needle_exit_pt_wrt_base,
                                                                                   trajectory_msgs::JointTrajectory &needleDriveTraj) {
  bool result;

  ROS_WARN("You should be using base frame coordinates.");

  overlapLeftCamFrameAndPsmBase(arm_index);
  // Now whatever you pass should be the same in both psmx base and lt camera frames. Meaning you can use directly the
  // same request functions embedded here. However, you can only use one PSM at a time.

  result = requestNeedleDrivingTrajectoryDefaultGrasp(arm_index,
                                                      needle_entry_pt_wrt_base,
                                                      needle_exit_pt_wrt_base,
                                                      needleDriveTraj);

  if (result != 1) {
    ROS_WARN("Failed");
    return result;
  } else {
    return result;
  }

}


bool RnNeedleDrivingPlanner::requestOneNeedleDrivingTrajectoryInBaseFrame(const int arm_index,
                                                                          const geometry_msgs::PointStamped &needle_entry_pt,
                                                                          const geometry_msgs::PointStamped &needle_exit_pt,
                                                                          const geometry_msgs::TransformStamped &grasp_transform,
                                                                          trajectory_msgs::JointTrajectory &needleDriveTraj) {

  bool result;

  ROS_WARN("You should be using base frame coordinates.");

  overlapLeftCamFrameAndPsmBase(arm_index);

  result = requestOneNeedleDrivingTrajectory(arm_index,
                                             needle_entry_pt,
                                             needle_exit_pt,
                                             grasp_transform,
                                             needleDriveTraj);

  if (result != 1) {
    ROS_WARN("Failed");
    return result;
  } else {
    return result;
  }

}


bool RnNeedleDrivingPlanner::requestOneNeedleDrivingTrajectoryInBaseFrame(const int arm_index,
                                                                          const geometry_msgs::PointStamped &needle_entry_pt,
                                                                          const geometry_msgs::PointStamped &needle_exit_pt,
                                                                          trajectory_msgs::JointTrajectory &needleDriveTraj,
                                                                          geometry_msgs::TransformStamped &grasp_transform) {

  bool result;

  ROS_WARN("You should be using base frame coordinates.");

  overlapLeftCamFrameAndPsmBase(arm_index);

  result = requestOneNeedleDrivingTrajectory(arm_index,
                                             needle_entry_pt,
                                             needle_exit_pt,
                                             needleDriveTraj,
                                             grasp_transform);

  if (result != 1) {
    ROS_WARN("Failed");
    return result;
  } else {
    return result;
  }

}




