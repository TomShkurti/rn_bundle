//
// Created by William on 28/03/18.
//

//#include "../include/rn_skills_as/RnNeedleDrivingPlanner.h"

#include "rn_skills_as/RnNeedleDrivingPlanner.h"


RnNeedleDrivingPlanner::RnNeedleDrivingPlanner(const ros::NodeHandle &nodeHandle):
    nh_(nodeHandle),
    psm_one_(1, nh_),
    psm_two_(2, nh_){

  ROS_INFO("Constructing a Needle Planner");

  tissue_normal_ << 0, 0, -1; // Note that the cam_z points down

  ROS_INFO("Resizing ik_ok_array, there are %d path waypoints.", path_waypoints_);
  ik_ok_array_.resize(path_waypoints_);

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
  bvec_needle_wrt_grasp_frame << 0, 0, -grab_needle_plus_minus_z_; // z
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
  bvec_needle_wrt_grasp_frame << 0, 0, grab_needle_plus_minus_z_; // z
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
  tf::StampedTransform tfResult_one, tfResult_two, tfResult_three;

  bool tf_acquired = false;
  int n_tries = 0;

  ROS_INFO("Attempting to get Camera PSMs transforms");

  while (!tf_acquired){
    if (n_tries > 5) break;
    tf_acquired = true;
    try {
      tfListener.lookupTransform("left_camera_link",
                                 "PSM1psm_base_link",
                                 ros::Time(0),
                                 tfResult_one);
      tfListener.lookupTransform("left_camera_link",
                                 "PSM2psm_base_link",
                                 ros::Time(0),
                                 tfResult_two);

      tfListener.lookupTransform("PSM1psm_base_link",
                                 "PSM2psm_base_link",
                                 ros::Time(0),
                                 tfResult_three);

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
    psm_two_affine_wrt_psm_one_ = convertTFToEigen(tfResult_three);

    ROS_WARN("Checking the transforms(psm_one_affine_wrt_lt_camera_):");
    printEigenAffine(psm_one_affine_wrt_lt_camera_);
    ROS_WARN("Checking the transforms(psm_two_affine_wrt_lt_camera_):");
    printEigenAffine(psm_two_affine_wrt_lt_camera_);
    ROS_WARN("Checking the transforms(psm_two_affine_wrt_psm_one_):");
    printEigenAffine(psm_two_affine_wrt_psm_one_);

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
      tfListener.lookupTransform("left_camera_link",
                                 "PSM1fingertip1",
                                 ros::Time(0),
                                 tfResult_one);
      tfListener.lookupTransform("left_camera_link",
                                 "PSM2fingertip1",
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

    ROS_WARN("Checking the transforms(gripper_one_affine_wrt_lt_camera_):");
    printEigenAffine(gripper_one_affine_wrt_lt_camera_);
    ROS_WARN("Checking the transforms(gripper_two_affine_wrt_lt_camera_):");
    printEigenAffine(gripper_two_affine_wrt_lt_camera_);

  }


}

// used by updateNeedleAndTissueParameters()
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
  tissue_rotation_wrt_lt_camera_.col(1) = tvec_tissue_frame_wrt_camera;
  tissue_rotation_wrt_lt_camera_.col(2) = bvec_tissue_frame_wrt_camera;

  tissue_affine_wrt_lt_camera_.linear() = tissue_rotation_wrt_lt_camera_;
  tissue_affine_wrt_lt_camera_.translation() = entry_pt;

  ROS_INFO("Tissue Frame has been defined with new entry and exit points.");


}


// Will give back the entry_pt wrt cam frame. This is only useful when the tip_pt is below needle centre.
void RnNeedleDrivingPlanner::defineTissueFrameWrtLtCameraViaExitAndTip(const Eigen::Vector3d &exit_pt,
                                                                       const Eigen::Vector3d &tip_pt,
                                                                       const Eigen::Vector3d &tissue_normal,
                                                                       Eigen::Vector3d & entry_pt) {

  Eigen::Vector3d temp_vec;
  Eigen::Affine3d temp_frame;
  Eigen::Matrix3d temp_rot;
  Eigen::Affine3d temp_exit_pt_affine;
  Eigen::Affine3d temp_tip_pt_affine;
  Eigen::Affine3d temp_entry_pt_affine;
  Eigen::Affine3d temp_result;
  Eigen::Matrix3d temp_frame_rot;
  Eigen::Vector3d temp_frame_exit_pt;
  Eigen::Vector3d temp_frame_tip_pt;
  Eigen::Vector3d temp_frame_entry_pt;
  Eigen::Vector3d temp_frame_origin;

  Eigen::Affine3d entry_pt_affine_wrt_cam_frame;

  Eigen::Vector3d nvec_tissue_frame_wrt_camera;
  Eigen::Vector3d tvec_tissue_frame_wrt_camera;
  Eigen::Vector3d bvec_tissue_frame_wrt_camera;


  // 1.1 First get the temp frame rotation mat w/rt lt cam frame
  temp_vec = tip_pt - exit_pt;
  bvec_tissue_frame_wrt_camera = tissue_normal;
  tvec_tissue_frame_wrt_camera = bvec_tissue_frame_wrt_camera.cross(temp_vec);
  tvec_tissue_frame_wrt_camera = tvec_tissue_frame_wrt_camera/tvec_tissue_frame_wrt_camera.norm();
  nvec_tissue_frame_wrt_camera = tvec_tissue_frame_wrt_camera.cross(bvec_tissue_frame_wrt_camera);

  temp_frame_rot.col(0) = nvec_tissue_frame_wrt_camera;
  temp_frame_rot.col(1) = tvec_tissue_frame_wrt_camera;
  temp_frame_rot.col(2) = bvec_tissue_frame_wrt_camera;

  // 1.2 define a temp coordinate frame wrt cam frame as follows
  temp_frame.linear() = temp_frame_rot;
  temp_frame.translation() = exit_pt;

  // 2.1 Then deduce the entry point in the temp frame
  temp_rot.setIdentity();
  temp_exit_pt_affine.linear() = temp_rot;
  temp_exit_pt_affine.translation() = exit_pt;
  temp_tip_pt_affine.linear() = temp_rot;
  temp_tip_pt_affine.translation() = tip_pt;

  temp_result = temp_frame * temp_exit_pt_affine;
  temp_frame_exit_pt = temp_result.translation();
  temp_result = temp_frame * temp_tip_pt_affine;
  temp_frame_tip_pt = temp_result.translation();


  if (temp_frame_tip_pt(1) != 0) {
    ROS_ERROR("Error from defineTissueFrameWrtLtCameraViaExitAndTip() calculation! 001");
  }


  double x1 = temp_frame_exit_pt(0);
  double y1 = temp_frame_exit_pt(2);
  double x2 = temp_frame_tip_pt(0);
  double y2 = temp_frame_tip_pt(2);
  double ori_x1, ori_y1, ori_x2, ori_y2;

  solveBinaryQuadraticCircleEquation(x1, y1, x2, y2, needle_radius_, ori_x1, ori_y1, ori_x2, ori_y2);

  if (ori_y1 > 0 && ori_y2 < 0 && ori_x1 < 0) {
    temp_frame_origin << ori_x1, 0 , ori_y1;
    setNeedleAxisHt(y1);
  } else if (ori_y1 < 0 && ori_y2 > 0 && ori_x2 < 0) {
    temp_frame_origin << ori_x2, 0 , ori_y2;
    setNeedleAxisHt(y2);
  } else {
    ROS_ERROR("Error from defineTissueFrameWrtLtCameraViaExitAndTip() calculation! 002");
  }

  temp_frame_entry_pt << 2*temp_frame_origin(0), 0, 0;
  temp_entry_pt_affine.linear() = temp_rot;
  temp_entry_pt_affine.translation() = temp_frame_entry_pt;

  entry_pt_affine_wrt_cam_frame = temp_frame.inverse() * temp_entry_pt_affine;
  entry_pt = entry_pt_affine_wrt_cam_frame.translation();

  defineTissueFrameWrtLtCamera(entry_pt, exit_pt, tissue_normal); // NOT necessary
  printEigenAffine(tissue_affine_wrt_lt_camera_);


}


// Used by generateGraspTransformList()
void RnNeedleDrivingPlanner::computeGraspTransform(int arm_index, double phi_x, double phi_y){

  Eigen::Matrix3d new_needle_rotation_wrt_grasp_one, new_needle_rotation_wrt_grasp_two, Rx, Ry;
  Eigen::Vector3d new_needle_origin_wrt_grasp_one, new_needle_origin_wrt_grasp_two;

  // Note that currently we never actually apply phi_y. Therefore we only allow the needle to
  // rotate about the gripper x axis (phi_x).
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

  ROS_INFO("Computing a needle drive trajectory/affines.");

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
//  double delta_phi = M_PI/(2*(path_waypoints_-1));
  double delta_phi = M_PI/((path_waypoints_-1));
  double phi_insertion = 0;

  /// Needle calculation
  needle_affine_wrt_tissue_frame_ = initial_needle_affine_wrt_tissue_frame_;

  // Not that  psi_needle_axis_tilt_wrt_tissue_ = 0 as a constant for now.
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

        // Update needle frame rotation wrt tissue each step
        Rot_needle = Rot_k_phi(kvec_needle, phi_insertion);
        needle_affine_wrt_tissue_frame_.linear() = Rot_needle * needle_rotation_mat_wrt_tissue_frame_;


        /// Gripper Calculation
        des_gripper_one_affine_wrt_tissue =
            needle_affine_wrt_tissue_frame_ * needle_affine_wrt_gripper_one_frame_.inverse();
        des_gripper_one_affine_wrt_lt_camera =
            tissue_affine_wrt_lt_camera_ * des_gripper_one_affine_wrt_tissue;
        des_gripper_one_wrt_base =
            psm_one_affine_wrt_lt_camera_.inverse() * des_gripper_one_affine_wrt_lt_camera;

        debug_affine_vessel_ = des_gripper_one_affine_wrt_tissue;

        if (ik_solver_.ik_solve(des_gripper_one_wrt_base)==1) {
          nsolns++;
          des_gripper_one_affines_wrt_lt_camera.push_back(des_gripper_one_wrt_base);

          if ((ipose == 0) || (ipose == (path_waypoints_ - 1)) ) {

            des_gripper_one_affines_wrt_lt_camera.push_back(des_gripper_one_wrt_base);
          }

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

        // Update needle frame rotation wrt tissue each step
        Rot_needle = Rot_k_phi(kvec_needle, phi_insertion);
        needle_affine_wrt_tissue_frame_.linear() = Rot_needle * needle_rotation_mat_wrt_tissue_frame_;

        /// Gripper Calculation
        des_gripper_two_affine_wrt_tissue =
            needle_affine_wrt_tissue_frame_ * needle_affine_wrt_gripper_two_frame_.inverse();
        des_gripper_two_affine_wrt_lt_camera =
            tissue_affine_wrt_lt_camera_ * des_gripper_two_affine_wrt_tissue;
        des_gripper_two_wrt_base =
            psm_two_affine_wrt_lt_camera_.inverse() * des_gripper_two_affine_wrt_lt_camera;

        debug_affine_vessel_ = des_gripper_two_affine_wrt_tissue;


        if (ik_solver_.ik_solve(des_gripper_two_wrt_base)==1) {
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

  std::cout << std::endl;
  std::cout << "ik_ok_point:  " << ik_ok_array_.transpose() << std::endl;
  std::cout << std::endl;




  fraction = double(nsolns/double(path_waypoints_));

  if (fraction != 1) {
    ROS_ERROR("Fraction of IK solutions is: %f, with %d solutions, among %d waypoints.",
              fraction, nsolns, path_waypoints_);
  } else {
    ROS_INFO("Needle driving trajectory (containing %d waypoints) computation succeeded with fraction: %f",
             path_waypoints_,
             fraction);
  }

  return fraction;

}




double RnNeedleDrivingPlanner::computeNeedleDriveGripperAffines(int arm_index,
                                                                double phi_from_initial_0,
                                                                double phi_from_initial_t,
                                                                trajectory_msgs::JointTrajectory &needleDriveTraj) {


  ROS_INFO("Computing a needle drive trajectory/affines.");

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
  double delta_phi = (phi_from_initial_t - phi_from_initial_0)/((path_waypoints_-1));

  double phi_insertion = phi_from_initial_0;

  /// Needle calculation
  needle_affine_wrt_tissue_frame_ = initial_needle_affine_wrt_tissue_frame_;

  // Not that  psi_needle_axis_tilt_wrt_tissue_ = 0 as a constant for now.
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

        // Update needle frame rotation wrt tissue each step
        Rot_needle = Rot_k_phi(kvec_needle, phi_insertion);
        needle_affine_wrt_tissue_frame_.linear() = Rot_needle * needle_rotation_mat_wrt_tissue_frame_;

        /// Gripper Calculation
        des_gripper_one_affine_wrt_tissue =
            needle_affine_wrt_tissue_frame_ * needle_affine_wrt_gripper_one_frame_.inverse();
        des_gripper_one_affine_wrt_lt_camera =
            tissue_affine_wrt_lt_camera_ * des_gripper_one_affine_wrt_tissue;
        des_gripper_one_wrt_base =
            psm_one_affine_wrt_lt_camera_.inverse() * des_gripper_one_affine_wrt_lt_camera;

        debug_affine_vessel_ = des_gripper_one_affine_wrt_tissue;

        if (ik_solver_.ik_solve(des_gripper_one_wrt_base)==1) {
          nsolns++;
          des_gripper_one_affines_wrt_lt_camera.push_back(des_gripper_one_wrt_base);

          if ((ipose == 0) || (ipose == (path_waypoints_ - 1)) ) {

            des_gripper_one_affines_wrt_lt_camera.push_back(des_gripper_one_wrt_base);
          }

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

        // Update needle frame rotation wrt tissue each step
        Rot_needle = Rot_k_phi(kvec_needle, phi_insertion);
        needle_affine_wrt_tissue_frame_.linear() = Rot_needle * needle_rotation_mat_wrt_tissue_frame_;

        /// Gripper Calculation
        des_gripper_two_affine_wrt_tissue =
            needle_affine_wrt_tissue_frame_ * needle_affine_wrt_gripper_two_frame_.inverse();
        des_gripper_two_affine_wrt_lt_camera =
            tissue_affine_wrt_lt_camera_ * des_gripper_two_affine_wrt_tissue;
        des_gripper_two_wrt_base =
            psm_two_affine_wrt_lt_camera_.inverse() * des_gripper_two_affine_wrt_lt_camera;

        debug_affine_vessel_ = des_gripper_two_affine_wrt_tissue;


        if (ik_solver_.ik_solve(des_gripper_two_wrt_base)==1) {
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

  std::cout << std::endl;
  std::cout << "ik_ok_point:  " << ik_ok_array_.transpose() << std::endl;
  std::cout << std::endl;




  fraction = double(nsolns/double(path_waypoints_));

  if (fraction != 1) {
    ROS_ERROR("Fraction of IK solutions is: %f, with %d solutions, among %d waypoints.",
              fraction, nsolns, path_waypoints_);
  } else {
    ROS_INFO("Needle driving trajectory (containing %d waypoints) computation succeeded with fraction: %f",
             path_waypoints_,
             fraction);
  }

  return fraction;


}



bool RnNeedleDrivingPlanner::hasValidNeedleDriveAffine(int arm_index, double phi) {

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

  double phi_insertion = phi;

  /// Needle calculation
  needle_affine_wrt_tissue_frame_ = initial_needle_affine_wrt_tissue_frame_;

  // Not that  psi_needle_axis_tilt_wrt_tissue_ = 0 as a constant for now.
  needle_affine_wrt_tissue_frame_.linear() =
      Rotx(psi_needle_axis_tilt_wrt_tissue_) * needle_affine_wrt_tissue_frame_.linear();
  needle_affine_wrt_tissue_frame_.translation() =
      Rotx(psi_needle_axis_tilt_wrt_tissue_) * needle_affine_wrt_tissue_frame_.translation();

  needle_origin_wrt_tissue_frame_ = needle_affine_wrt_tissue_frame_.translation();
  needle_rotation_mat_wrt_tissue_frame_ = needle_affine_wrt_tissue_frame_.linear();

  kvec_needle = needle_affine_wrt_tissue_frame_.linear().col(2);


  switch (arm_index) {

    case 1:

      // Calculate only once.
      Rot_needle = Rot_k_phi(kvec_needle, phi_insertion);
      needle_affine_wrt_tissue_frame_.linear() = Rot_needle * needle_rotation_mat_wrt_tissue_frame_;

      /// Gripper Calculation
      des_gripper_one_affine_wrt_tissue =
          needle_affine_wrt_tissue_frame_ * needle_affine_wrt_gripper_one_frame_.inverse();
      des_gripper_one_affine_wrt_lt_camera =
          tissue_affine_wrt_lt_camera_ * des_gripper_one_affine_wrt_tissue;
      des_gripper_one_wrt_base =
          psm_one_affine_wrt_lt_camera_.inverse() * des_gripper_one_affine_wrt_lt_camera;

      if (ik_solver_.ik_solve(des_gripper_one_wrt_base)==1) {

        return true;

      } else {

        return false;

      }

      break;

    case 2:

      // Calculate only once.
      Rot_needle = Rot_k_phi(kvec_needle, phi_insertion);
      needle_affine_wrt_tissue_frame_.linear() = Rot_needle * needle_rotation_mat_wrt_tissue_frame_;

      /// Gripper Calculation
      des_gripper_two_affine_wrt_tissue =
          needle_affine_wrt_tissue_frame_ * needle_affine_wrt_gripper_two_frame_.inverse();
      des_gripper_two_affine_wrt_lt_camera =
          tissue_affine_wrt_lt_camera_ * des_gripper_two_affine_wrt_tissue;
      des_gripper_two_wrt_base =
          psm_two_affine_wrt_lt_camera_.inverse() * des_gripper_two_affine_wrt_lt_camera;

      if (ik_solver_.ik_solve(des_gripper_two_wrt_base)==1) {

        return true;

      } else {

        return false;

      }

      break;

  }

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

  size = gripper_affines_wrt_portal.size();

  for (int n = 0; n < size; n++)
  {
    Vectorq7x1 q_vec1;
    joint_trajectory_point.positions.clear();

    if (ik_solver_.ik_solve(gripper_affines_wrt_portal[n])==1) {

      q_vec1 = ik_solver_.get_soln();
      q_vec1(6) = 0;


    } else {
      ROS_ERROR("Failed to solve IK while generating a trajectory from affines");
    }

    for (int i = 0; i < 7; i++)
    {
      joint_trajectory_point.positions.push_back(q_vec1(i));
    }
    joint_trajectory_point.time_from_start = ros::Duration(n + 5);
    joint_trajectory.points.push_back(joint_trajectory_point);

  }

  ROS_INFO("Joint Trajectory Generated.");

}


void RnNeedleDrivingPlanner::updateNeedleAndTissueParameters(const geometry_msgs::PointStamped &needle_entry_pt,
                                                             const geometry_msgs::PointStamped &needle_exit_pt) {

  Eigen::Vector3d needle_origin_wrt_tissue_frame;
  Eigen::Quaterniond q;
  Eigen::Matrix3d R;

  needle_entry_point_ = convertPointStampedToEigenVector(needle_entry_pt);
  ROS_INFO("Needle entry point has been updated.");
  needle_exit_point_ = convertPointStampedToEigenVector(needle_exit_pt);
  ROS_INFO("Needle exit point has been updated.");

  // Make sure the exit point is valid, if not, adjust it

  checkExitPoint(needle_entry_point_, needle_exit_point_, dist_entrance_to_exit_);
  ROS_INFO("Dist entrance to exit (%f) has been updated.", dist_entrance_to_exit_);

  defineTissueFrameWrtLtCamera(needle_entry_point_, needle_exit_point_, tissue_normal_);
  ROS_INFO("Tissue Frame definition has been updated.");

  needle_axis_ht_ = sqrt(needle_radius_ * needle_radius_ - (dist_entrance_to_exit_ / 2) * (dist_entrance_to_exit_ / 2));
  suture_depth_ = needle_radius_ - needle_axis_ht_;
  ROS_INFO("Needle axis height (%f) and suture depth (%f) has been updated.", needle_axis_ht_, suture_depth_);

  needle_origin_wrt_tissue_frame << 0.5 * dist_entrance_to_exit_, 0, needle_axis_ht_;
  initial_needle_affine_wrt_tissue_frame_.translation() = needle_origin_wrt_tissue_frame;
  ROS_INFO("Needle Tissue Transform has been updated.");


  phi_needle_initial_ = 0;
  phi_needle_penetration_ = atan(needle_axis_ht_/(0.5*dist_entrance_to_exit_));
//  phi_needle_emergence_ = M_PI - 2*phi_needle_penetration_;
  phi_needle_emergence_ = M_PI - 1*phi_needle_penetration_;

  // This set gets updated every time the system receives a new pair of needle entry and exit.
  needle_phis_.phi_initial = 0;
  needle_phis_.phi_exit_pt = - atan(needle_axis_ht_/(0.5*dist_entrance_to_exit_));
  needle_phis_.phi_entry_pt = M_PI - needle_phis_.phi_exit_pt;
  needle_phis_.phi_penetration = - needle_phis_.phi_exit_pt; // = atan(needle_axis_ht_/(0.5*dist_entrance_to_exit_))
  needle_phis_.phi_emergence = M_PI - needle_phis_.phi_penetration;
  needle_phis_.phi_initial_insertion_lower_reference = needle_phis_.phi_penetration - (1.0/36.0)*M_PI;
  needle_phis_.phi_initial_insertion_upper_reference = needle_phis_.phi_penetration + (5.0/36.0)*M_PI;
  needle_phis_.phi_final_insertion_lower_reference = needle_phis_.phi_emergence - (5.0/36.0)*M_PI;
  needle_phis_.phi_final_insertion_upper_reference = needle_phis_.phi_emergence + (1.0/36.0)*M_PI;



  ROS_INFO("Needle Penetration (%f) and Emergence Phi (%f) have been updated.", phi_needle_penetration_, phi_needle_emergence_);
}


// This is only useful when the tip_pt is below needle centre.
void RnNeedleDrivingPlanner::updateNeedleAndTissueParametersWithExitAndTip(const geometry_msgs::PointStamped &needle_exit_pt,
                                                                           const geometry_msgs::PointStamped &needle_tip_pt) {

  Eigen::Vector3d tip_pt;
  Eigen::Vector3d needle_origin_wrt_tissue_frame;

  tip_pt = convertPointStampedToEigenVector(needle_tip_pt);

  needle_exit_point_ = convertPointStampedToEigenVector(needle_exit_pt);
  ROS_INFO("Needle exit point has been updated.");

  // needle_entry_point_ will be set by the following function
  defineTissueFrameWrtLtCameraViaExitAndTip(needle_exit_point_, tip_pt, tissue_normal_, needle_entry_point_);
  ROS_INFO("Needle entry point has been updated.");
  ROS_INFO("Tissue Frame definition has been updated.");

  checkExitPoint(needle_entry_point_, needle_exit_point_, dist_entrance_to_exit_);

  needle_axis_ht_ = sqrt(needle_radius_ * needle_radius_ - (dist_entrance_to_exit_ / 2) * (dist_entrance_to_exit_ / 2));
  suture_depth_ = needle_radius_ - needle_axis_ht_;
  ROS_INFO("Needle axis height/suture depth has been updated.");

  needle_origin_wrt_tissue_frame << 0.5 * dist_entrance_to_exit_, 0, needle_axis_ht_;
  initial_needle_affine_wrt_tissue_frame_.translation() = needle_origin_wrt_tissue_frame;
  ROS_INFO("Needle Tissue Transform has been updated.");

  phi_needle_initial_ = 0;
  phi_needle_penetration_ = atan(needle_axis_ht_/(0.5*dist_entrance_to_exit_));
//  phi_needle_emergence_ = M_PI - 2*phi_needle_penetration_;
  phi_needle_emergence_ = M_PI - 1*phi_needle_penetration_;

  // This set gets updated every time the system receives a new pair of needle entry and exit.
  needle_phis_.phi_initial = 0;
  needle_phis_.phi_exit_pt = - atan(needle_axis_ht_/(0.5*dist_entrance_to_exit_));
  needle_phis_.phi_entry_pt = M_PI - needle_phis_.phi_exit_pt;
  needle_phis_.phi_penetration = - needle_phis_.phi_exit_pt; // = atan(needle_axis_ht_/(0.5*dist_entrance_to_exit_))
  needle_phis_.phi_emergence = M_PI - needle_phis_.phi_penetration;
  needle_phis_.phi_initial_insertion_lower_reference = needle_phis_.phi_penetration - (1.0/36.0)*M_PI;
  needle_phis_.phi_initial_insertion_upper_reference = needle_phis_.phi_penetration + (5.0/36.0)*M_PI;
  needle_phis_.phi_final_insertion_lower_reference = needle_phis_.phi_emergence - (5.0/36.0)*M_PI;
  needle_phis_.phi_final_insertion_upper_reference = needle_phis_.phi_emergence + (1.0/36.0)*M_PI;

  ROS_INFO("Needle Penetration and Emergence Phis have been u"
           "pdated.");

}



/// Interface for Applications

/*
 * Use default grasp transform and attempt a needle driving trajectory
 * with given entry and exit points.
 */
bool RnNeedleDrivingPlanner::requestNeedleDrivingTrajectoryDefaultGrasp(const int &arm_index,
                                                                        const geometry_msgs::PointStamped &needle_entry_pt,
                                                                        const geometry_msgs::PointStamped &needle_exit_pt,
                                                                        trajectory_msgs::JointTrajectory &needleDriveTraj) {

  double ik_fraction;

  updateNeedleAndTissueParameters(needle_entry_pt, needle_exit_pt);
  needle_affine_wrt_grasp_one_frame_ = default_needle_affine_wrt_grasp_one_frame_;
  needle_affine_wrt_grasp_two_frame_ = default_needle_affine_wrt_grasp_two_frame_;
  updateNeedleWrtGripperTransforms(); // to get needle_affine_wrt_gripper_one_frame_ & needle_affine_wrt_gripper_two_frame_
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
bool RnNeedleDrivingPlanner::requestOneNeedleDrivingTrajectoryUserGrasp(const int &arm_index,
                                                               const geometry_msgs::PointStamped &needle_entry_pt,
                                                               const geometry_msgs::PointStamped &needle_exit_pt,
                                                               const geometry_msgs::TransformStamped &grasp_transform,
                                                               trajectory_msgs::JointTrajectory &needleDriveTraj) {

  double ik_fraction;

  updateNeedleAndTissueParameters(needle_entry_pt, needle_exit_pt);

  computeGraspTransform(arm_index, grasp_transform);
  updateNeedleWrtGripperTransforms(); // TODO not needed perhaps

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
bool RnNeedleDrivingPlanner::requestOneNeedleDrivingTrajectoryGeneratedGrasp(const int &arm_index,
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
  bool has_solution;

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

    has_solution = requestOneNeedleDrivingTrajectoryUserGrasp(arm_index,
                                                     needle_entry_pt,
                                                     needle_exit_pt,
                                                     potential_transform_list.stamped_transform_list[i],
                                                     needleDriveTraj);


//    computeGraspTransform(arm_index, potential_transform_list.stamped_transform_list[i]);
//    updateNeedleWrtGripperTransforms();
//    ik_fraction = computeNeedleDriveGripperAffines(arm_index, needleDriveTraj);


    if (has_solution == true) {
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

    ROS_ERROR("\nCould not find any grasp transform that suffices the entry & exit pts to do a needle drive.\n");
    return false;

  }


}



bool RnNeedleDrivingPlanner::requestNeedleDrivingTrajectoryDefaultGrasp(const int &arm_index,
                                                                        const geometry_msgs::PointStamped &needle_entry_pt,
                                                                        const geometry_msgs::PointStamped &needle_exit_pt,
                                                                        const double phi_0,
                                                                        const double phi_t,
                                                                        trajectory_msgs::JointTrajectory &needleDriveTraj) {

  double ik_fraction;

  updateNeedleAndTissueParameters(needle_entry_pt, needle_exit_pt);
  needle_affine_wrt_grasp_one_frame_ = default_needle_affine_wrt_grasp_one_frame_;
  needle_affine_wrt_grasp_two_frame_ = default_needle_affine_wrt_grasp_two_frame_;
  updateNeedleWrtGripperTransforms(); // to get needle_affine_wrt_gripper_one_frame_ & needle_affine_wrt_gripper_two_frame_

  updatePsmKinematicAvailability(arm_index);

  ik_fraction = computeNeedleDriveGripperAffines(arm_index, phi_0, phi_t, needleDriveTraj);

  if (ik_fraction == 1) {
    ROS_INFO("A valid needle drive trajectory has been acquired.");

    // update needle upper and lower kinematically possible angle.
    updateNeedleDriveKinematicBoundary(arm_index);

    return true;
  } else {
    ROS_WARN("Failed to get a valid needle drive trajectory, the ik solution fraction is: %f", ik_fraction);
    return false;
  }

}




bool RnNeedleDrivingPlanner::requestOneNeedleDrivingTrajectoryUserGrasp(const int &arm_index,
                                                                        const geometry_msgs::PointStamped &needle_entry_pt,
                                                                        const geometry_msgs::PointStamped &needle_exit_pt,
                                                                        const geometry_msgs::TransformStamped &grasp_transform,
                                                                        const double phi_0,
                                                                        const double phi_t,
                                                                        trajectory_msgs::JointTrajectory &needleDriveTraj) {

  double ik_fraction;

  updateNeedleAndTissueParameters(needle_entry_pt, needle_exit_pt);

  computeGraspTransform(arm_index, grasp_transform);
  updateNeedleWrtGripperTransforms(); // TODO not needed perhaps

  updatePsmKinematicAvailability(arm_index);

  ik_fraction = computeNeedleDriveGripperAffines(arm_index, phi_0, phi_t, needleDriveTraj);

  if (ik_fraction == 1) {
    ROS_INFO("A valid needle drive trajectory has been acquired.");

    // update needle upper and lower kinematically possible angle.
    updateNeedleDriveKinematicBoundary(arm_index);

    return true;
  } else {
    ROS_WARN("Failed to get a valid needle drive trajectory, the ik solution fraction is: %f", ik_fraction);
    return false;
  }

}



bool RnNeedleDrivingPlanner::requestOneNeedleDrivingTrajectoryGeneratedGrasp(const int &arm_index,
                                                                             const geometry_msgs::PointStamped &needle_entry_pt,
                                                                             const geometry_msgs::PointStamped &needle_exit_pt,
                                                                             const double phi_0,
                                                                             const double phi_t,
                                                                             trajectory_msgs::JointTrajectory &needleDriveTraj,
                                                                             geometry_msgs::TransformStamped &grasp_transform) {

    cwru_davinci_msgs::ListOfTransformStamped potential_transform_list;
    cwru_davinci_msgs::ListOfTransformStamped valid_transform_list;
    Eigen::Affine3d default_preferred_grasp_tf;
    double ik_fraction;
    double grasp_trials_fraction;
    int ngrasps = 0;
    bool has_solution;

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

      has_solution = requestOneNeedleDrivingTrajectoryUserGrasp(arm_index,
                                                                needle_entry_pt,
                                                                needle_exit_pt,
                                                                potential_transform_list.stamped_transform_list[i],
                                                                phi_0,
                                                                phi_t,
                                                                needleDriveTraj);

      if (has_solution == true) {
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

      ROS_ERROR("\nCould not find any grasp transform that suffices the entry & exit pts to do a needle drive.\n");
      return false;

    }

  }



bool RnNeedleDrivingPlanner::requestOneNeedleDrivingTrajectoryUserGripperNeedleTransform(const int &arm_index,
                                                                 const geometry_msgs::PointStamped &needle_entry_pt,
                                                                 const geometry_msgs::PointStamped &needle_exit_pt,
                                                                 const geometry_msgs::TransformStamped &grasp_transform,
                                                                 const double phi_0,
                                                                 const double phi_t,
                                                                 trajectory_msgs::JointTrajectory &needleDriveTraj) {



}



// TODO retire this one
void RnNeedleDrivingPlanner::updateNeedleDriveKinematicBoundary(const int &arm_index) {

  double upper_phi, lower_phi;
  int count = 0;
  bool upper_limit_found = false;
  bool lower_limit_found = false;

  double delta_phi = M_PI/180;

  upper_phi = phi_needle_penetration_ + phi_needle_emergence_;
  lower_phi = phi_needle_penetration_;

  while ((!upper_limit_found) && (upper_phi < (phi_needle_penetration_ + M_PI) )) {


    if (hasValidNeedleDriveAffine(arm_index, upper_phi)) {

      count ++;
      upper_phi = upper_phi + delta_phi;

    } else {

      switch (arm_index) {
        case 1:
          phi_needle_kinematic_upper_angle_psm_1_ = upper_phi - delta_phi;
          break;
        case 2:
          phi_needle_kinematic_upper_angle_psm_2_ = upper_phi - delta_phi;
          break;
      }

      upper_limit_found = true;
    }

  }

  if (upper_limit_found) {
    ROS_INFO("phi_needle_kinematic_upper_angle (%f) has been updated for PSM %d", upper_phi - delta_phi, arm_index);
  } else {
    ROS_WARN("phi_needle_kinematic_upper_angle has NOT been updated for PSM %d", arm_index);
  }

  count = 0;

  while ((!lower_limit_found) && (lower_limit_found > -phi_needle_penetration_)) {



    if (hasValidNeedleDriveAffine(arm_index, lower_phi)) {

      count ++;
      lower_phi = lower_phi - delta_phi;



    } else {

      switch (arm_index) {
        case 1:
          phi_needle_kinematic_lower_angle_psm_1_ = lower_phi + delta_phi;
          break;
        case 2:
          phi_needle_kinematic_lower_angle_psm_2_ = lower_phi + delta_phi;
          break;
      }

      lower_limit_found = true;
    }

  }

  if (lower_limit_found) {
    ROS_INFO("phi_needle_kinematic_lower_angle (%f) has been updated for PSM %d", lower_phi + delta_phi, arm_index);
  } else {
    ROS_WARN("phi_needle_kinematic_lower_angle has NOT been updated for PSM %d", arm_index);
  }

}



void RnNeedleDrivingPlanner::updatePsmKinematicAvailability(const int &arm_index) {

  int n_step;
  double angle_step;
  double angle_test;
  bool previous_ik_ok = false;

  int n_section = 0;

  angle_step = (double) M_PI/36.0; // 5 degrees

  n_step = (int)fabs((needle_phis_.phi_entry_pt - needle_phis_.phi_exit_pt)/angle_step);
  angle_test = needle_phis_.phi_exit_pt;

  // Comment out after debugging
//  needle_phis_.showParameters();

  switch (arm_index) {

    case 1:

      psm_1_kinematic_availability_.section_lower_limits.clear();
      psm_1_kinematic_availability_.section_upper_limits.clear();

      psm_1_kinematic_availability_.arm_index = 1;
      psm_1_kinematic_availability_.num_continuous_kinematic_ok_sections = 0;


      for (int n = 0; n < n_step; n++) {

        if (hasValidNeedleDriveAffine(arm_index, angle_test)) {

          if (!previous_ik_ok) { // a section's lower boundary found

            psm_1_kinematic_availability_.section_lower_limits.push_back(angle_test);

            n_section ++;
            psm_1_kinematic_availability_.num_continuous_kinematic_ok_sections = n_section;

          } else {
            // do nothing
          }

          previous_ik_ok = true;

        } else {

          if (previous_ik_ok) { // a section's upper boundary found

            double v_2 = angle_test - angle_step;
            psm_1_kinematic_availability_.section_upper_limits.push_back(v_2); // need to step back

          }

          previous_ik_ok = false;

        }

        angle_test = angle_test + angle_step;

      }

      ROS_WARN("There are %d kinematically availible sections for PSM 1.", psm_1_kinematic_availability_.num_continuous_kinematic_ok_sections);
      std::cout << "_____________________________________________" << std::endl;
      std::cout << " Section # | Lower boundary | Upper Boundary" << std::endl
                << "---------------------------------------------" << std::endl;
      for (int i = 0; i < psm_1_kinematic_availability_.num_continuous_kinematic_ok_sections; i++) {

        std::cout << "    "<< i << "         "
                  << psm_1_kinematic_availability_.section_lower_limits[i] << "        "
                  << psm_1_kinematic_availability_.section_upper_limits[i] << std::endl;
      }
      std::cout << "---------------------------------------------" << std::endl;

      break;

    case 2:

      psm_2_kinematic_availability_.section_lower_limits.clear();
      psm_2_kinematic_availability_.section_upper_limits.clear();

      psm_2_kinematic_availability_.arm_index = 2;
      psm_2_kinematic_availability_.num_continuous_kinematic_ok_sections = 0;


      for (int n = 0; n < n_step; n++) {

        if (hasValidNeedleDriveAffine(arm_index, angle_test)) {

          if (!previous_ik_ok) { // a section's lower boundary found

            psm_2_kinematic_availability_.section_lower_limits.push_back(angle_test);

            n_section ++;
            psm_2_kinematic_availability_.num_continuous_kinematic_ok_sections = n_section;

          } else {
            // do nothing
          }

          previous_ik_ok = true;

        } else {

          if (previous_ik_ok) { // a section's upper boundary found

            double v_2 = angle_test - angle_step;
            psm_2_kinematic_availability_.section_upper_limits.push_back(v_2); // need to step back

          }

          previous_ik_ok = false;

        }

        angle_test = angle_test + angle_step;

      }

      ROS_WARN("There are %d kinematically availible sections for PSM 2.", psm_2_kinematic_availability_.num_continuous_kinematic_ok_sections);
      std::cout << "_____________________________________________" << std::endl;
      std::cout << " Section # | Lower boundary | Upper Boundary" << std::endl
                << "---------------------------------------------" << std::endl;
      for (int i = 0; i < psm_2_kinematic_availability_.num_continuous_kinematic_ok_sections; i++) {

        std::cout << "    "<< i << "         "
                  << psm_2_kinematic_availability_.section_lower_limits[i] << "        "
                  << psm_2_kinematic_availability_.section_upper_limits[i] << std::endl;
      }
      std::cout << "---------------------------------------------" << std::endl;

      break;


  }










}





/*
 * Generate a list of grasp transforms (needle w/rt gripper) for further evaluation.
 */
void RnNeedleDrivingPlanner::generateGraspTransformList(const int &arm_index,
                                                        const int resolution,
                                                        cwru_davinci_msgs::ListOfTransformStamped &grasp_tf_array) {

  double needle_x, needle_y;
  geometry_msgs::TransformStamped grasp_transform_temp;
  Eigen::Matrix3d R;
  needle_y = 0;
  // for (needle_x = 0; needle_x < 0.7854; needle_x += 0.1)
  // for (needle_x = 0; needle_x < (2 * M_PI); needle_x += 0.1)

  double needle_x_increment;

  needle_x_increment = (2 * M_PI)/resolution;

  switch (arm_index) {

    case 1:

      for (needle_x = 0; needle_x < (2 * M_PI); needle_x += needle_x_increment) {
        computeGraspTransform(1, needle_x, needle_y);
        grasp_transform_temp.transform.translation.x = needle_affine_wrt_grasp_one_frame_(0, 3);
        grasp_transform_temp.transform.translation.y = needle_affine_wrt_grasp_one_frame_(1, 3);
        grasp_transform_temp.transform.translation.z = needle_affine_wrt_grasp_one_frame_(2, 3);
        R = needle_affine_wrt_grasp_one_frame_.linear();
        Eigen::Quaterniond q1(R);
        grasp_transform_temp.transform.rotation.x = q1.x();
        grasp_transform_temp.transform.rotation.y = q1.y();
        grasp_transform_temp.transform.rotation.z = q1.z();
        grasp_transform_temp.transform.rotation.w = q1.w();
        grasp_transform_temp.child_frame_id = "left_camera_link";
        grasp_transform_temp.header.stamp = ros::Time::now();
        grasp_tf_array.stamped_transform_list.push_back(grasp_transform_temp);
      }

      break;


    case 2:

      for (needle_x = 0; needle_x < (2 * M_PI); needle_x += needle_x_increment) {
        computeGraspTransform(2, needle_x, needle_y);
        grasp_transform_temp.transform.translation.x = needle_affine_wrt_grasp_two_frame_(0, 3);
        grasp_transform_temp.transform.translation.y = needle_affine_wrt_grasp_two_frame_(1, 3);
        grasp_transform_temp.transform.translation.z = needle_affine_wrt_grasp_two_frame_(2, 3);
        R = needle_affine_wrt_grasp_two_frame_.linear();
        Eigen::Quaterniond q2(R);
        grasp_transform_temp.transform.rotation.x = q2.x();
        grasp_transform_temp.transform.rotation.y = q2.y();
        grasp_transform_temp.transform.rotation.z = q2.z();
        grasp_transform_temp.transform.rotation.w = q2.w();
        grasp_transform_temp.child_frame_id = "left_camera_link";
        grasp_transform_temp.header.stamp = ros::Time::now();
        grasp_tf_array.stamped_transform_list.push_back(grasp_transform_temp);
      }

      break;

  }

  int size_of_output_list = grasp_tf_array.stamped_transform_list.size();

  ROS_INFO("Grasp transform list generated with %d point, step length: %f. There are %d items in the list",
           resolution,
           needle_x_increment,
           size_of_output_list);


}




/*
 * Generate a list of exit points (w/rt left camera frame) for further evaluation.
 */
void RnNeedleDrivingPlanner::generateExitPoints(const geometry_msgs::PointStamped &needle_entry_pt,
                                                const double &suture_depth,
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
double RnNeedleDrivingPlanner::filterValidExitPoints(const int &arm_index,
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

    if (requestOneNeedleDrivingTrajectoryGeneratedGrasp(arm_index,
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
double RnNeedleDrivingPlanner::filterValidExitPoints(const int &arm_index,
                                                     const geometry_msgs::PointStamped &needle_entry_pt,
                                                     const geometry_msgs::TransformStamped &grasp_tf,
                                                     const cwru_davinci_msgs::ListOfPointStamped &exit_points_array,
                                                     cwru_davinci_msgs::ListOfPointStamped &valid_exit_points_array) {

  double valid_exit_fraction;
  int valid_count = 0;

  // They are only containers, not used for passing info.
  geometry_msgs::TransformStamped grasp_transform;
  trajectory_msgs::JointTrajectory needleDriveTraj;

  int n_candidates = exit_points_array.stamped_point_list.size();

  for (int n = 0; n < n_candidates; n++) {

    if (requestOneNeedleDrivingTrajectoryUserGrasp(arm_index,
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
double RnNeedleDrivingPlanner::filterValidExitPointsDefaultGrasp(const int &arm_index,
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
  double total_dist;

  double max_actual_velocity;

  double acceleration_time, dcceleration_start_time, time_count, temp_t;

  double current_vel, next_vel, average_vel;
  Eigen::Affine3d affine_gripper_wrt_base;
  Eigen::Vector3d pt0, pt1;
  Eigen::Vector3d temp_vec;
  double temp;
  Vectorq7x1 q_vec0, q_vec1;
  double time_from_start, time_0, delta_t;

  time_0 = 7;
  current_vel = 0;
  time_count = 0;
  total_dist = approximateTrajectoryDist(needleDriveTraj);
  temp_t = velocity/acceleration_;

  if (temp_t*velocity < total_dist) {
    acceleration_time = temp_t;
    temp_t = (total_dist/velocity) - acceleration_time;
    dcceleration_start_time = acceleration_time + temp_t;
  } else {
    acceleration_time = sqrt(total_dist/acceleration_);
    dcceleration_start_time = acceleration_time;
    max_actual_velocity = dcceleration_start_time*acceleration_;
    ROS_WARN("Unable to reach the requested speed due to the short distance. The actual maximum speed will be: %f",
             max_actual_velocity);
  }

  needleDriveTraj.points[0].time_from_start = ros::Duration(time_0);
  time_from_start = time_0;

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

    temp_vec = pt0 - pt1;
    temp = temp_vec.transpose() * temp_vec;
    euler_dist = sqrt(fabs(temp));

    /// Consier the acceleration period

    if (euler_dist == 0) {

      // If the euler_dist is 0, meaning the point is the meant to be a stop-and-check point
      // therefore a 2-second halt is added for that point.
      delta_t = 4;

    } else {

      // Acceleration process
      if (time_count < acceleration_time) {


        // calculate the velocity at the end of the movement
        next_vel = sqrt(current_vel*current_vel + 2*euler_dist*acceleration_);

        if (next_vel > velocity) {
          next_vel = velocity;
        }

        average_vel = 0.5*(next_vel + current_vel);

        // update the current vel for the next loop
        current_vel = next_vel;


      } else if (time_count > dcceleration_start_time) {


        // calculate the velocity at the end of the movement
        if (current_vel*current_vel - 2*euler_dist*acceleration_ > 0) {
          next_vel = sqrt(current_vel*current_vel - 2*euler_dist*acceleration_);
        } else {
          next_vel = 0;
        }

        average_vel = 0.5*(next_vel + current_vel);

        // update the current vel for the next loop
        current_vel = next_vel;


      } else {
          average_vel = velocity;
        }

      delta_t = euler_dist/average_vel;
      time_count = time_count + delta_t; // Should not be the same as time_from_start due to possible halt periods.

    }

    time_from_start = time_from_start + delta_t;

    needleDriveTraj.points[n+1].time_from_start = ros::Duration(time_from_start);

// TODO delete
    std::cout  << pt0.transpose() << std::endl;

  }


  ROS_INFO("All points in this trajectory has their time_from_start set according to the given velocity %f",
           velocity);

}


double RnNeedleDrivingPlanner::approximateTrajectoryDist(trajectory_msgs::JointTrajectory &joint_trajectory) {


  double euler_dist;
  double total_dist;

  Eigen::Affine3d affine_gripper_wrt_base;
  Eigen::Vector3d pt0, pt1;
  Eigen::Vector3d temp_vec;
  double temp;
  Vectorq7x1 q_vec0, q_vec1;

  total_dist = 0;

  int waypoints = joint_trajectory.points.size();

  for (int n = 0; n < (waypoints-1); n++) {

    for (int i = 0; i < 7; i++) {
      q_vec0(i) = joint_trajectory.points[n].positions[i];
    }

    affine_gripper_wrt_base = fwd_solver_.fwd_kin_solve(q_vec0);

    pt0 = affine_gripper_wrt_base.translation();

    for (int i = 0; i < 7; i++) {
      q_vec1(i) = joint_trajectory.points[n+1].positions[i];
    }

    affine_gripper_wrt_base = fwd_solver_.fwd_kin_solve(q_vec1);
    pt1 = affine_gripper_wrt_base.translation();

    temp_vec = pt0 - pt1;
    temp = temp_vec.transpose() * temp_vec;
    euler_dist = sqrt(fabs(temp));

    total_dist = total_dist + euler_dist;

  }

  return total_dist;
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


// TODO appears to have bugs
/*
 * To make sure the entry-exit distance is less than the needle diameter. This function is called
 * internally by another member function updateNeedleAndTissueParameters().
 */
void RnNeedleDrivingPlanner::checkExitPoint(const Eigen::Vector3d& entry_pt,
                                            Eigen::Vector3d& exit_pt,
                                            double& dist_entry_exit) {

  double temp1, temp2, temp3;
  Eigen::Vector3d temp_vec;
  Eigen::Vector3d direction;

  direction = exit_pt - entry_pt;
  direction = direction/direction.norm();

  temp_vec = exit_pt - entry_pt;
  temp2 = temp_vec.transpose() * temp_vec;


  dist_entry_exit = sqrt(fabs(temp2));

  if (dist_entry_exit >= 2*needle_radius_) {
    ROS_WARN("The entry exit distance (%f) is greater than the needle diameter. It will be adjusted!",
             dist_entry_exit);

    // Update exit point and distance
    exit_pt = entry_pt + direction * 1.8 * needle_radius_;

    std::cout << "entry_pt: " << entry_pt.transpose() << std::endl;
    std::cout << "updated exit_pt: " << exit_pt.transpose() << std::endl;

    temp_vec = exit_pt - entry_pt;
    temp2 = temp_vec.transpose() * temp_vec;
    dist_entry_exit = sqrt(fabs(temp2));

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


void RnNeedleDrivingPlanner::overlapLeftCamFrameAndPsmBase(const int &arm_index) {

  /// Fisrt thing should be setting the default tissue norm w/rt base frame
  tissue_normal_ << 0, 0, 1;

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


bool RnNeedleDrivingPlanner::requestNeedleDrivingTrajectoryDefaultGraspInBaseFrame(const int &arm_index,
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


bool RnNeedleDrivingPlanner::requestOneNeedleDrivingTrajectoryUserGraspInBaseFrame(const int &arm_index,
                                                                          const geometry_msgs::PointStamped &needle_entry_pt,
                                                                          const geometry_msgs::PointStamped &needle_exit_pt,
                                                                          const geometry_msgs::TransformStamped &grasp_transform,
                                                                          trajectory_msgs::JointTrajectory &needleDriveTraj) {

  bool result;

  ROS_WARN("You should be using base frame coordinates.");

  overlapLeftCamFrameAndPsmBase(arm_index);

  result = requestOneNeedleDrivingTrajectoryUserGrasp(arm_index,
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


bool RnNeedleDrivingPlanner::requestOneNeedleDrivingTrajectoryInBaseFrame(const int &arm_index,
                                                                          const geometry_msgs::PointStamped &needle_entry_pt,
                                                                          const geometry_msgs::PointStamped &needle_exit_pt,
                                                                          trajectory_msgs::JointTrajectory &needleDriveTraj,
                                                                          geometry_msgs::TransformStamped &grasp_transform) {

  bool result;

  ROS_WARN("You should be using base frame coordinates.");

  overlapLeftCamFrameAndPsmBase(arm_index);

  result = requestOneNeedleDrivingTrajectoryGeneratedGrasp(arm_index,
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


/// Dual Arm Related Functions

void RnNeedleDrivingPlanner::generateDualPsmOpBoundaryVertices() {

//  std::vector<Eigen::Vector3d> dual_ok_pts_in_pms1;
  davinci_kinematics::Vectorq7x1 q_vec;
  Eigen::Affine3d psm1_test_affine_in_psm2, psm2_test_affine_in_psm1, self_psm_affine;
  Eigen::Matrix3d psm1_tip_rotation, psm2_tip_rotation;
  Eigen::Vector3d x_vec_psm_1, y_vec_psm_1, z_vec_psm_1;
  Eigen::Vector3d x_vec_psm_2, y_vec_psm_2, z_vec_psm_2;
  Eigen::Vector3d psm1_test_pt_in_psm1, psm2_test_pt_in_psm2, psm1_test_pt_in_psm2, psm2_test_pt_in_psm1;
  Eigen::Vector3d temp_vec;
  int count = 0;

  double delta_angle = M_PI/36.0;

  dual_op_boundary_pts_in_psm_one_.clear();
  dual_op_zone_geo_centre_in_psm_one_ << 0, 0, 0;

  // TODO delete or clean up
//  joint_range_upper_limit:
//  1.5994 0.94249 0.24001 3.0485 3.0528 3.0376 3.0399
//  joint_range_lower_limit:
//  -1.605 -0.93556 -0.002444 -3.0456 -3.0414 -3.0481 -3.0498

  // TODO delete
  std::cout << "POINTS" << std::endl << std::endl;

  // q_upper_limits & q_lower_limits defined in namespace davinci_kinematics
  // <cwru_davinci_kinematics/davinci_kinematic_definitions.h>
  for (double q0 = -1.605; q0 < 1.5994; q0 = q0 + delta_angle) {

    for (double q1 = -0.93556; q1 < 0.94249; q1 = q1 + delta_angle) {

      q_vec(0) = q0;
      q_vec(1) = q1;
      q_vec(2) = 0.24001;
      q_vec(3) = 0;
      q_vec(4) = 0;
      q_vec(5) = 0;
      q_vec(6) = 0;

      // A PSM1 test point in PSM1 base frame OR a PSM2 test point in PSM2 base frame
      self_psm_affine = fwd_solver_.fwd_kin_solve(q_vec); // A point in a PSM1 base frame

      psm1_test_pt_in_psm1 = self_psm_affine.translation();
      psm2_test_pt_in_psm2 = psm1_test_pt_in_psm1;

      x_vec_psm_1 << 0, 0, 1;
      z_vec_psm_1 = psm1_test_pt_in_psm1/psm1_test_pt_in_psm1.norm();
      y_vec_psm_1 = z_vec_psm_1.cross(x_vec_psm_1);
      psm1_tip_rotation.col(0) = x_vec_psm_1;
      psm1_tip_rotation.col(1) = z_vec_psm_1;
      psm1_tip_rotation.col(2) = y_vec_psm_1;

      self_psm_affine.linear() = psm1_tip_rotation;

      // A PMS1 test point in PSM2 base frame
      psm1_test_affine_in_psm2 = psm_two_affine_wrt_psm_one_.inverse() * self_psm_affine;
      // A PSM2 test point in PSM1 base frame
      psm2_test_affine_in_psm1 = psm_two_affine_wrt_psm_one_ * self_psm_affine;

      psm1_test_pt_in_psm2 = psm1_test_affine_in_psm2.translation();
      psm2_test_pt_in_psm1 = psm2_test_affine_in_psm1.translation();

//      std::cout << psm1_test_pt_in_psm1.transpose() << std::endl;
//      std::cout << psm2_test_pt_in_psm1.transpose() << std::endl;


      // Check if the PSM1 test point passes PSM2's ik test
      if (ik_solver_.ik_solve(psm1_test_affine_in_psm2)==1) {
        dual_op_boundary_pts_in_psm_one_.push_back(psm1_test_pt_in_psm1);
//        std::cout << psm1_test_pt_in_psm1.transpose() << std::endl;
        temp_vec = temp_vec + psm1_test_pt_in_psm1;
        count++;

      }

      // Check if the PSM2 test point passes PSM1's ik test
      if (ik_solver_.ik_solve(psm2_test_affine_in_psm1)==1) {
        dual_op_boundary_pts_in_psm_one_.push_back(psm2_test_pt_in_psm1);
//        std::cout << psm2_test_pt_in_psm1.transpose() << std::endl;
        temp_vec = temp_vec + psm2_test_pt_in_psm1;
        count++;
      }

    }

  }

  if (count != 0) {
    dual_op_zone_geo_centre_in_psm_one_(0) = temp_vec(0)/count;
    dual_op_zone_geo_centre_in_psm_one_(1) = temp_vec(1)/count;
    dual_op_zone_geo_centre_in_psm_one_(2) = temp_vec(2)/count;
  }

  std::cout << "dual_op_boundary_pts_in_psm_one_: " << std::endl;
  std::cout << dual_op_zone_geo_centre_in_psm_one_.transpose() << std::endl;



}



/// PSM controllers

void RnNeedleDrivingPlanner::executeTrajectory(psm_controller &psm,
                                               const trajectory_msgs::JointTrajectory &needle_drive_traj) {

  ROS_WARN("Reviewing Plan");

  trajectory_msgs::JointTrajectory trajectory;
  trajectory = needle_drive_traj;

  int size = needle_drive_traj.points.size();

  ros::Duration duration = needle_drive_traj.points[size-1].time_from_start;
  double secs = duration.toSec();

  ROS_INFO("secs: %f", secs);

  ROS_WARN("Executing Plan!");

  psm.move_psm(trajectory);

  duration.sleep();

  ROS_WARN("Execution Complete!");

}


void RnNeedleDrivingPlanner::executeTrajectory(psm_controller &psm,
                                               const cwru_davinci_msgs::ListOfJointTrajectory &list_of_joint_traj) {

  ROS_WARN("Receiving a Trajectory List");

  int list_size;

  list_size =  list_of_joint_traj.joint_trajectory_list.size();

  for (int n = 0; n < list_size; n++) {

    executeTrajectory(psm, list_of_joint_traj.joint_trajectory_list[n]);

  }

  ROS_WARN("A total of %d trajectories have been exectued!", list_size);

}




void RnNeedleDrivingPlanner::goToLocationPointingDownFaceVector(psm_controller &psm,
                                                      const double &x,
                                                      const double &y,
                                                      const double &z) {

  davinci_kinematics::Vectorq7x1 q_vec;

  Eigen::Vector3d tip_origin;
  Eigen::Vector3d x_vec, y_vec, z_vec;
  Eigen::Matrix3d tip_rotation;
  Eigen::Affine3d des_affine;
  double norm;

  double time = 7;

  trajectory_msgs::JointTrajectoryPoint trajPoint;
  trajectory_msgs::JointTrajectory traj;
  trajPoint.positions.resize(7);

  // Deduce the gripper rotation w/rt base frame first

  if (x==0 && y==0) {
    x_vec << 0, 1, 0;
  } else {
    norm = sqrt(x*x + y*y);
    x_vec << x/norm, y/norm, 0;
  }
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
  ik_solver_.ik_solve(des_affine);
  q_vec = ik_solver_.get_soln();

  // TODO delete
  std::cout << "q_vec: " << q_vec.transpose() << std::endl;

  // Fill in the traj msgs
  for (int i = 0; i < 7; i++) {
    trajPoint.positions[i] = q_vec[i];
  }

  trajPoint.time_from_start = ros::Duration(time);

  traj.points.clear();
  traj.joint_names.clear();
  traj.header.stamp = ros::Time::now();

  traj.points.push_back(trajPoint);


  // Order the PSM to move
  ROS_INFO("Going to (%f, %f, %f)", x, y, z);
  psm.move_psm(traj);
  ros::Duration(time).sleep(); // TODO is this necessary?
  ROS_INFO("Done");

}



void RnNeedleDrivingPlanner::goToLocationPointingDownFaceForward(psm_controller &psm,
                                                                 const double &x,
                                                                 const double &y,
                                                                 const double &z) {

  davinci_kinematics::Vectorq7x1 q_vec;

  Eigen::Vector3d tip_origin;
  Eigen::Vector3d x_vec, y_vec, z_vec;
  Eigen::Matrix3d tip_rotation;
  Eigen::Affine3d des_affine;
  double norm;

  double time = 7;

  trajectory_msgs::JointTrajectoryPoint trajPoint;
  trajectory_msgs::JointTrajectory traj;
  trajPoint.positions.resize(7);


  // Gripper x faces forward
  z_vec << 0, 0, -1;
  x_vec << 0, 1, 0;
  y_vec = z_vec.cross(x_vec);
  tip_rotation.col(0) = x_vec;
  tip_rotation.col(1) = y_vec;
  tip_rotation.col(2) = z_vec;

  // Fill in the affine
  tip_origin << x, y, z;
  des_affine.linear() = tip_rotation;
  des_affine.translation() = tip_origin;

  // Sent to the IK solver
  ik_solver_.ik_solve(des_affine);
  q_vec = ik_solver_.get_soln();


  // Fill in the traj msgs
  for (int i = 0; i < 7; i++) {
    trajPoint.positions[i] = q_vec[i];
  }

  trajPoint.time_from_start = ros::Duration(time);

  traj.points.clear();
  traj.joint_names.clear();
  traj.header.stamp = ros::Time::now();

  traj.points.push_back(trajPoint);

  // Order the PSM to move
  ROS_INFO("Going to (%f, %f, %f)", x, y, z);
  psm.move_psm(traj);
  ros::Duration(time).sleep(); // TODO is this necessary?
  ROS_INFO("Done");

}



void RnNeedleDrivingPlanner::goToLocationPointingPsmLeft(psm_controller &psm,
                                                      const double &x,
                                                      const double &y,
                                                      const double &z) {

  davinci_kinematics::Vectorq7x1 q_vec;

  Eigen::Vector3d tip_origin;
  Eigen::Vector3d x_vec, y_vec, z_vec;
  Eigen::Matrix3d tip_rotation;
  Eigen::Affine3d des_affine;
  Eigen::Affine3d debug_affine;
  double norm;

  double time = 7;

  trajectory_msgs::JointTrajectoryPoint trajPoint;
  trajectory_msgs::JointTrajectory traj;
  trajPoint.positions.resize(7);


  // Gripper x faces Up
  z_vec << -1, 0, 0;
  x_vec << 0, 0, 1;
  y_vec = z_vec.cross(x_vec);
  tip_rotation.col(0) = x_vec;
  tip_rotation.col(1) = y_vec;
  tip_rotation.col(2) = z_vec;

  // Fill in the affine
  tip_origin << x, y, z;
  des_affine.linear() = tip_rotation;
  des_affine.translation() = tip_origin;

  // Sent to the IK solver
  ik_solver_.ik_solve(des_affine);
  q_vec = ik_solver_.get_soln();
  debug_affine = fwd_solver_.fwd_kin_solve(q_vec);

  // Fill in the traj msgs
  for (int i = 0; i < 7; i++) {
    trajPoint.positions[i] = q_vec[i];
  }

  trajPoint.time_from_start = ros::Duration(time);

  traj.points.clear();
  traj.joint_names.clear();
  traj.header.stamp = ros::Time::now();

  traj.points.push_back(trajPoint);

  // Order the PSM to move
  ROS_INFO("Going to (%f, %f, %f)", x, y, z);
  std::cout << "q_vec: " << q_vec.transpose();
  std::cout << "debug_affine:" << std::endl;
  printEigenAffine(debug_affine);
  psm.move_psm(traj);
  ros::Duration(time).sleep(); // TODO is this necessary?
  ROS_INFO("Done");

}

void RnNeedleDrivingPlanner::goToLocationPointingPsmRight(psm_controller &psm,
                                                      const double &x,
                                                      const double &y,
                                                      const double &z) {

  davinci_kinematics::Vectorq7x1 q_vec;

  Eigen::Vector3d tip_origin;
  Eigen::Vector3d x_vec, y_vec, z_vec;
  Eigen::Matrix3d tip_rotation;
  Eigen::Affine3d des_affine;
  Eigen::Affine3d debug_affine;
  double norm;

  double time = 7;

  trajectory_msgs::JointTrajectoryPoint trajPoint;
  trajectory_msgs::JointTrajectory traj;
  trajPoint.positions.resize(7);


  // Gripper x faces Down
  z_vec << 1, 0, 0;
  x_vec << 0, 0, -1;
  y_vec = z_vec.cross(x_vec);
  tip_rotation.col(0) = x_vec;
  tip_rotation.col(1) = y_vec;
  tip_rotation.col(2) = z_vec;

  // Fill in the affine
  tip_origin << x, y, z;
  des_affine.linear() = tip_rotation;
  des_affine.translation() = tip_origin;

  // Sent to the IK solver
  ik_solver_.ik_solve(des_affine);
  q_vec = ik_solver_.get_soln();
  debug_affine = fwd_solver_.fwd_kin_solve(q_vec);


  // Fill in the traj msgs
  for (int i = 0; i < 7; i++) {
    trajPoint.positions[i] = q_vec[i];
  }

  trajPoint.time_from_start = ros::Duration(time);

  traj.points.clear();
  traj.joint_names.clear();
  traj.header.stamp = ros::Time::now();

  traj.points.push_back(trajPoint);

  // Order the PSM to move
  ROS_INFO("Going to (%f, %f, %f)", x, y, z);
  std::cout << "q_vec: " << q_vec.transpose();
  std::cout << "debug_affine:" << std::endl;
  printEigenAffine(debug_affine);
  psm.move_psm(traj);
  ros::Duration(time).sleep(); // TODO is this necessary?
  ROS_INFO("Done");

}


// TODO complete
void RnNeedleDrivingPlanner::openGripper(psm_controller &psm, const double &angle) {

  davinci_kinematics::Vectorq7x1 q_vec;


}





/// Debugging Functions

Eigen::Vector3d RnNeedleDrivingPlanner::transformPointFromBaseToLtCamFrame(const int & arm_index,
                                                                           const Eigen::Vector3d &point) {

  Eigen::Matrix3d point_rotation;
  Eigen::Affine3d point_affine;
  point_rotation.setIdentity();
  point_affine.translation() = point;
  point_affine.linear() = point_rotation;

  Eigen::Affine3d result_affine;
  Eigen::Vector3d result_point;

  switch (arm_index) {

    case 1:

      // base wrt cam affine
      result_affine = psm_one_affine_wrt_lt_camera_*point_affine;
      result_point = result_affine.translation();

      break;

    case 2:
      // base wrt cam affine
      result_affine = psm_two_affine_wrt_lt_camera_*point_affine;
      result_point = result_affine.translation();

      break;

  }

  std::cout << "transformPointFromBaseToLtCamFrame: " << std::endl <<
           "Point in base frame: " << point.transpose() << std::endl <<
           "Point in cam frame: " << result_point.transpose() << std::endl;

  return result_point;

}


Eigen::Vector3d RnNeedleDrivingPlanner::transformPointFromLtCamFrameToBase(const int & arm_index,
                                                                           const Eigen::Vector3d &point) {

  Eigen::Matrix3d point_rotation;
  Eigen::Affine3d point_affine;
  point_rotation.setIdentity();
  point_affine.translation() = point;
  point_affine.linear() = point_rotation;

  Eigen::Affine3d result_affine;
  Eigen::Vector3d result_point;

  switch (arm_index) {

    case 1:

      // base wrt cam affine
      result_affine = psm_one_affine_wrt_lt_camera_.inverse()*point_affine;
      result_point = result_affine.translation();

      break;

    case 2:
      // base wrt cam affine
      result_affine = psm_two_affine_wrt_lt_camera_.inverse()*point_affine;
      result_point = result_affine.translation();

      break;

  }

  std::cout << "transformPointFromBaseToLtCamFrame: " << std::endl <<
            "Point in cam frame: " << point.transpose() << std::endl <<
            "Point in base frame: " << result_point.transpose() << std::endl;

  return result_point;

}



/// Auxiliary Functions


void RnNeedleDrivingPlanner::solveBinaryQuadraticCircleEquation(double x1,
                                                                double y1,
                                                                double x2,
                                                                double y2,
                                                                double r,
                                                                double &origin_x1,
                                                                double &origin_y1,
                                                                double &origin_x2,
                                                                double &origin_y2) {

  double k = (x1 - x2)/(y2 - y1);
  double j = (-x1*x1 + x2*x2 - y1*y1 + y2*y2)/2*(y2 - y1);
  double a = (1 + k*k);
  double b = 2*k*j;
  double c = (j*j - r*r);

  origin_x1 = (-b + sqrt(b*b - 4*a*c))/(2*a);
  origin_x2 = (-b - sqrt(b*b - 4*a*c))/(2*a);

  origin_y1 = ((x1 - x2)/(y2 - y1))*(origin_x1 - (x1 + x2)/2) + (y1 + y2)/2;
  origin_y2 = ((x1 - x2)/(y2 - y1))*(origin_x2 - (x1 + x2)/2) + (y1 + y2)/2;

  // TODO delete
  {
    std::cout << "origin_x1: " << origin_x1 << std::endl;
    std::cout << "origin_y1: " << origin_y1 << std::endl;
    std::cout << "origin_x2: " << origin_x2 << std::endl;
    std::cout << "origin_y2: " << origin_y2 << std::endl;
  }


}
