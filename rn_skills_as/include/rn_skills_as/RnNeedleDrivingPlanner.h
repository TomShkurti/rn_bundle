//
// Created by William on 28/03/18.
//

#ifndef RN_SKILLS_AS_RNNEEDLEDRIVINGPLANNER_H
#define RN_SKILLS_AS_RNNEEDLEDRIVINGPLANNER_H

#include "rn_skills_as/rn_davinci_skills_base.h"



class RnNeedleDrivingPlanner {
 public:

  explicit RnNeedleDrivingPlanner(const ros::NodeHandle &nodeHandle);
  // ~RnNeedleDrivingPlanner();

  /// Initialisations, normally only get called once.
  bool getNeedleParams();
  void computeDefaultNeedleWrtGripperTransform();
  bool getTransforms();
  bool getCameraToPSMsTransforms();
  bool getCameraToGripperTransforms();


  /// Needle Driving Related Functions
  void defineTissueFrameWrtLtCamera(Eigen::Vector3d entry_pt,
                                    Eigen::Vector3d exit_pt,
                                    Eigen::Vector3d tissue_normal);

  void computeGraspTransform(int arm_index, double phi_x, double phi_y);

  void computeGraspTransform(int arm_index,
                             const geometry_msgs::TransformStamped &grasp_transform);

  double computeNeedleDriveGripperAffines(int arm_index,
                                          trajectory_msgs::JointTrajectory &needleDriveTraj);

  void updateNeedleAndTissueParameters(const geometry_msgs::PointStamped &needle_entry_pt,
                                       const geometry_msgs::PointStamped &needle_exit_pt);

  inline void updateNeedleWrtGripperTransforms(){
    needle_affine_wrt_gripper_one_frame_ =
        psm_1_grasp_frame_wrt_gripper_frame_ * needle_affine_wrt_grasp_one_frame_;
    needle_affine_wrt_gripper_two_frame_ =
        psm_2_grasp_frame_wrt_gripper_frame_ * needle_affine_wrt_grasp_two_frame_;
  }


  /// Adjustment & Correction

  /**
   * Set a preferred grasp tf, compare all the tf options in the provided array, select
   * the one that match best.
   * @param preferred_tf
   * @param grasp_tf_array
   * @return the best matched tf from the given list.
   */
  geometry_msgs::TransformStamped getBestGraspTransform(Eigen::Affine3d preferred_tf,
                                                        const cwru_davinci_msgs::ListOfTransformStamped &grasp_tf_array);


  /**
   * To make sure the entry-exit distance is less than the needle diameter. This function is called
   * internally by another member function updateNeedleAndTissueParameters()
   * @param entry_pt
   * @param exit_pt will be modified if adjustment is needed
   * @param dist_entry_exit
   */
  void checkExitPoint(const Eigen::Vector3d& entry_pt,
                      Eigen::Vector3d& exit_pt,
                      double& dist_entry_exit);


  /// Interface for Applications

  /**
   * Use default grasp transform and attempt a needle driving trajectory
   * with given entry and exit points.
   * @param arm_index: 1 or 2
   * @param needle_entry_pt: in left camera frame
   * @param needle_exit_pt: in left camera frame
   * @param needleDriveTraj
   * @return true only when the fraction of valid waypoints is 1.
   */
  bool requestNeedleDrivingTrajectoryDefaultGrasp(const int &arm_index,
                                                  const geometry_msgs::PointStamped &needle_entry_pt,
                                                  const geometry_msgs::PointStamped &needle_exit_pt,
                                                  trajectory_msgs::JointTrajectory &needleDriveTraj);

  /**
   * Use user defined grasp transform to attempt a needle driving trajectory
   * with given entry and exit points.
   * @param arm_index: 1 or 2
   * @param needle_entry_pt: in left camera frame
   * @param needle_exit_pt: in left camera frame
   * @param needleDriveTraj
   * @return true only when the fraction of valid waypoints is 1.
   * @return
   */
  bool requestOneNeedleDrivingTrajectory(const int &arm_index,
                                         const geometry_msgs::PointStamped &needle_entry_pt,
                                         const geometry_msgs::PointStamped &needle_exit_pt,
                                         const geometry_msgs::TransformStamped &grasp_transform,
                                         trajectory_msgs::JointTrajectory &needleDriveTraj);


  /**
   * Find a trajectory which allows a needle driving from the entry to exit requested by the user.
   * Auto-search for a grasp transform that works.
   * @param arm_index
   * @param needle_entry_pt
   * @param needle_exit_pt
   * @param needleDriveTraj
   * @param grasp_transform
   * @return
   */
  bool requestOneNeedleDrivingTrajectory(const int &arm_index,
                                         const geometry_msgs::PointStamped &needle_entry_pt,
                                         const geometry_msgs::PointStamped &needle_exit_pt,
                                         trajectory_msgs::JointTrajectory &needleDriveTraj,
                                         geometry_msgs::TransformStamped &grasp_transform);


  /**
   * Generate a list of grasp transforms (needle w/rt gripper) for further evaluation.
   * @param arm_index
   * @param resolution: This many points that you want to insert in 2 PI.
   * @param grasp_tf_array: results are stored in it.
   */
  void generateGraspTransformList(const int &arm_index,
                                  const int resolution,
                                  cwru_davinci_msgs::ListOfTransformStamped &grasp_tf_array);

  /**
   * Generate a list of exit points (w/rt left camera frame) for further evaluation.
   * @param needle_exit_pt
   * @param suture_depth
   * @param potential_exit_points_array
   */
  void generateExitPoints(const geometry_msgs::PointStamped &needle_entry_pt,
                          const double &suture_depth,
                          cwru_davinci_msgs::ListOfPointStamped &potential_exit_points_array);

  /**
   * Filter the valid exit points provided in an input array. Use multiple grasp transforms.
   * @param arm_index
   * @param needle_entry_pt
   * @param exit_points_array
   * @param valid_exit_points_array
   * @return the fraction of valid exit points
   */
  double filterValidExitPoints(const int &arm_index,
                               const geometry_msgs::PointStamped &needle_entry_pt,
                               const cwru_davinci_msgs::ListOfPointStamped &exit_points_array,
                               cwru_davinci_msgs::ListOfPointStamped &valid_exit_points_array);

  /**
   * Filter the valid exit points provided in an input array. Use specified grasp transform.
   * @param arm_index
   * @param needle_entry_pt
   * @param grasp_tf
   * @param exit_points_array
   * @param valid_exit_points_array
   * @return the fraction of valid exit points
   */
  double filterValidExitPoints(const int &arm_index,
                               const geometry_msgs::PointStamped &needle_entry_pt,
                               const geometry_msgs::TransformStamped &grasp_tf,
                               const cwru_davinci_msgs::ListOfPointStamped &exit_points_array,
                               cwru_davinci_msgs::ListOfPointStamped &valid_exit_points_array);

  /**
   * Filter the valid exit points provided in an input array. Use default grasp transform.
   * @param needle_entry_pt
   * @param exit_points_array
   * @param valid_exit_points_array
   * @return the fraction of valid exit points
   */
  double filterValidExitPointsDefaultGrasp(const int &arm_index,
                                           const geometry_msgs::PointStamped &needle_entry_pt,
                                           const cwru_davinci_msgs::ListOfPointStamped &exit_points_array,
                                           cwru_davinci_msgs::ListOfPointStamped &valid_exit_points_array);


  // Will use fwd kin to solve pts and then estimate the distance
  // with the velocity provided te "time from start" field can be filled
  void setTrajectoryVelocity(double velocity, trajectory_msgs::JointTrajectory &needleDriveTraj);



  /// Auxiliary Utility Functions

  void convertAffinesToTrajectoryMsgs(const std::vector<Eigen::Affine3d> &gripper_affines_wrt_portal,
                                      trajectory_msgs::JointTrajectory &joint_trajectory);

  inline Eigen::Vector3d convertPointStampedToEigenVector(const geometry_msgs::PointStamped &pt)
  {
    Eigen::Vector3d vec;
    vec << pt.point.x, pt.point.y, pt.point.z;
    return vec;
  }

  inline Eigen::Affine3d convertTFToEigen(const tf::Transform &t)
  {
    Eigen::Affine3d e;
    for (int i = 0; i < 3; i++)
    {
      e.matrix()(i, 3) = t.getOrigin()[i];
      for (int j = 0; j < 3; j++)
      {
        e.matrix()(i, j) = t.getBasis()[i][j];
      }
    }
    // Fill in identity in last row
    for (int col = 0; col < 3; col++)
      e.matrix()(3, col) = 0;
    e.matrix()(3, 3) = 1;
    return e;
  }

  inline Eigen::Affine3d convertGeoTransformStampedToEigenAffine(const geometry_msgs::TransformStamped &geoTf) {

    Eigen::Affine3d e;
    Eigen::Vector3d origin;
    Eigen::Matrix3d rotation;
    Eigen::Quaterniond quaternion;

    origin << geoTf.transform.translation.x,
        geoTf.transform.translation.y,
        geoTf.transform.translation.z;

    quaternion.x() = geoTf.transform.rotation.x;
    quaternion.y() = geoTf.transform.rotation.y;
    quaternion.z() = geoTf.transform.rotation.z;
    quaternion.w() = geoTf.transform.rotation.w;

    rotation = quaternion.toRotationMatrix();

    e.linear() = rotation;
    e.translation() = origin;

    return e;
  }

  inline geometry_msgs::TransformStamped convertEigenAffineToGeoTransform(Eigen::Affine3d affine) {

    geometry_msgs::TransformStamped geo_tf;

    Eigen::Quaterniond quaternion;
    Eigen::Matrix3d rotation;
    Eigen::Vector3d origin;

    rotation = affine.linear();
    quaternion = rotation;

    origin = affine.translation();

    geo_tf.transform.rotation.x = quaternion.x();
    geo_tf.transform.rotation.y = quaternion.y();
    geo_tf.transform.rotation.z = quaternion.z();
    geo_tf.transform.rotation.w = quaternion.w();

    geo_tf.transform.translation.x = origin[0];
    geo_tf.transform.translation.y = origin[1];
    geo_tf.transform.translation.z = origin[2];

    return geo_tf;

  }

  inline void printEigenAffine(Eigen::Affine3d affine)
  {
    std::cout << "Rotation: " << std::endl;
    std::cout << affine.linear() << std::endl;
    std::cout << "origin: " << affine.translation().transpose() << std::endl;
  }

  inline Eigen::Matrix3d Rotx(double phi)
  {
    Eigen::Matrix3d Rx;
    Rx(0, 0) = 1.0;
    Rx(0, 1) = 0.0;
    Rx(0, 2) = 0.0;
    Rx(1, 0) = 0.0;
    Rx(1, 1) = cos(phi);
    Rx(1, 2) = -sin(phi);
    Rx(2, 0) = 0.0;
    Rx(2, 1) = sin(phi);
    Rx(2, 2) = cos(phi);
    return Rx;
  }

  inline Eigen::Matrix3d Rotz(double phi)
  {
    Eigen::Matrix3d Rz;
    Rz(0, 0) = cos(phi);
    Rz(0, 1) = -sin(phi);
    Rz(0, 2) = 0.0;
    Rz(1, 0) = sin(phi);
    Rz(1, 1) = cos(phi);
    Rz(1, 2) = 0.0;
    Rz(2, 0) = 0.0;
    Rz(2, 1) = 0.0;
    Rz(2, 2) = 1.0;
    return Rz;
  }

  inline Eigen::Matrix3d Roty(double phi)
  {
    Eigen::Matrix3d Roty;
    Roty(0, 0) = cos(phi);
    Roty(0, 1) = 0;
    Roty(0, 2) = sin(phi);
    Roty(1, 0) = 0;
    Roty(1, 1) = 1.0;
    Roty(1, 2) = 0.0;
    Roty(2, 0) = -sin(phi);
    Roty(2, 1) = 0.0;
    Roty(2, 2) = cos(phi);
    return Roty;
  }

  inline Eigen::Matrix3d Rot_k_phi(Eigen::Vector3d k_vec, double phi)
  {
    Eigen::Matrix3d R_k_phi;
    double kx = k_vec(0);
    double ky = k_vec(1);
    double kz = k_vec(2);
    Eigen::Matrix3d K;
    K(0, 0) = 0.0;
    K(0, 1) = -kz;
    K(0, 2) = ky;
    K(1, 0) = kz;
    K(1, 1) = 0.0;
    K(1, 2) = -kx;
    K(2, 0) = -ky;
    K(2, 1) = kx;
    K(2, 2) = 0;
    Eigen::Matrix3d I = Eigen::MatrixXd::Identity(3, 3);
    R_k_phi = I + sin(phi) * K + (1 - cos(phi)) * K * K;
    return R_k_phi;
  }

  inline geometry_msgs::PointStamped ConvertEigenVec3dToGeoMsgsPtStamped(Eigen::Vector3d vec)
  {
    geometry_msgs::PointStamped pt;
    pt.header.stamp = ros::Time::now();
    pt.header.frame_id = "/left_camera_optical_frame";
    pt.point.x = vec[0];
    pt.point.y = vec[1];
    pt.point.z = vec[2];
    return pt;
  }


  /// set() functions

  inline void setNeedleRaidus(double new_needle_radius) {
    needle_radius_ = new_needle_radius;
    nh_.setParam("/needle/radius", needle_radius_);
    ROS_INFO("Needle radius has been updated locally and in the parameter server.");
  }

  inline void setNeedleGraspDepth(double new_grasp_depth) {
    grasp_depth_ = new_grasp_depth;
    nh_.setParam("/needle/grasp_depth", grasp_depth_);
    ROS_INFO("Needle grasp depth has been updated locally and in the parameter server.");
  }

  inline void setNeedleAxisHt(double new_axis_ht) {
    needle_axis_ht_ = new_axis_ht;
    nh_.setParam("/needle/height", needle_axis_ht_);
    ROS_INFO("Needle axis height has been updated locally and in the parameter server.");
  }

  inline void setDefaultGraspTfSearchResolution(int resolution) {
    default_grasp_tf_search_resolution_ = resolution;
    ROS_INFO("default_grasp_tf_search_resolution_ has been updated.");
  }


  /// get() functions

  inline Eigen::Affine3d getDefaultNeedleAffineWrtGraspOneFrame() {
    return default_needle_affine_wrt_grasp_one_frame_;
  }




  /// Camera Free Needle Drive Interface

  /**
   * Set hard coded transform of camera psm
   * @param psm_one_affine_wrt_lt_camera: use this to replace the member variable
   * @param psm_two_affine_wrt_lt_camera: use this to replace the member varibale
   */
  void setHardCodedTransforms(Eigen::Affine3d psm_one_affine_wrt_lt_camera,
                              Eigen::Affine3d psm_two_affine_wrt_lt_camera);

  void overlapLeftCamFrameAndPsmBase(const int &arm_index);

  /// may not be of much use now
  Eigen::Affine3d convertPsmBaseFrameCoordinateToCameraFrame(int arm_index,
                                                             Eigen::Affine3d coord_base_frame);

  // TODO Seems that we set the tf from cam to psm base to be identity matrix will do the job
  // because if the cam_frame is the base_frame, whatever you order in the cam_frame is equivlent of
  // having it ordered in the base frame in the first place. BUT, we have 2 pams.. So we have to
  // do it once a time.

  bool requestNeedleDrivingTrajectoryDefaultGraspInBaseFrame(const int &arm_index,
                                                             const geometry_msgs::PointStamped &needle_entry_pt_wrt_base,
                                                             const geometry_msgs::PointStamped &needle_exit_pt_wrt_base,
                                                             trajectory_msgs::JointTrajectory &needleDriveTraj);


  bool requestOneNeedleDrivingTrajectoryInBaseFrame(const int &arm_index,
                                                    const geometry_msgs::PointStamped &needle_entry_pt,
                                                    const geometry_msgs::PointStamped &needle_exit_pt,
                                                    const geometry_msgs::TransformStamped &grasp_transform,
                                                    trajectory_msgs::JointTrajectory &needleDriveTraj);


  // TODO caution! This may not be that straight forward. Make sure the artificial cam frame is not going to affect it.

  bool requestOneNeedleDrivingTrajectoryInBaseFrame(const int &arm_index,
                                                    const geometry_msgs::PointStamped &needle_entry_pt,
                                                    const geometry_msgs::PointStamped &needle_exit_pt,
                                                    trajectory_msgs::JointTrajectory &needleDriveTraj,
                                                    geometry_msgs::TransformStamped &grasp_transform);



  /// Debugging functions

  Eigen::Vector3d transformPointFromBaseToLtCamFrame(const int & arm_index,
                                                     const Eigen::Vector3d &point);



  /// TODO finish
  void outputPspPlayfile(std::string path);



 private:
  /// ROS
  ros::NodeHandle nh_;
  ros::Publisher exit_pt_publisher_;
  ros::Publisher exit_pt_score_publisher_;
  ros::Publisher exit_pt_array_publisher_;

  /// Needle & Grasp related variables and matrices.
  double needle_radius_;
  double grasp_depth_;
  double needle_axis_ht_;
  double suture_depth_;
  double dist_entrance_to_exit_;
  int grab_needle_plus_minus_y_;
  int grab_needle_plus_minus_z_;
  double psi_needle_axis_tilt_wrt_tissue_; // Not implemented

  Eigen::Vector3d needle_origin_wrt_tissue_frame_;
  Eigen::Matrix3d needle_rotation_mat_wrt_tissue_frame_;
  Eigen::Affine3d needle_affine_wrt_tissue_frame_;
  Eigen::Affine3d initial_needle_affine_wrt_tissue_frame_; // as a save

  Eigen::Affine3d psm_1_grasp_frame_wrt_gripper_frame_;
  Eigen::Affine3d psm_2_grasp_frame_wrt_gripper_frame_;

  Eigen::Vector3d needle_origin_wrt_grasp_one_frame_;
  Eigen::Matrix3d needle_rotation_wrt_grasp_one_frame_;
  Eigen::Affine3d needle_affine_wrt_grasp_one_frame_;
  Eigen::Vector3d needle_origin_wrt_grasp_two_frame_;
  Eigen::Matrix3d needle_rotation_wrt_grasp_two_frame_;
  Eigen::Affine3d needle_affine_wrt_grasp_two_frame_;
  Eigen::Affine3d default_needle_affine_wrt_grasp_one_frame_;
  Eigen::Affine3d default_needle_affine_wrt_grasp_two_frame_;

  Eigen::Vector3d needle_origin_wrt_gripper_one_frame_;
  Eigen::Matrix3d needle_rotation_wrt_gripper_one_frame_;
  Eigen::Affine3d needle_affine_wrt_gripper_one_frame_;
  Eigen::Vector3d needle_origin_wrt_gripper_two_frame_;
  Eigen::Matrix3d needle_rotation_wrt_gripper_two_frame_;
  Eigen::Affine3d needle_affine_wrt_gripper_two_frame_;

  /// Needle grasping constants
  const int GRASP_W_NEEDLE_POSITIVE_GRIPPER_Y = 1;
  const int GRASP_W_NEEDLE_NEGATIVE_GRIPPER_Y = -1;
  const int GRASP_W_NEEDLE_POSITIVE_GRIPPER_Z = 1;
  const int GRASP_W_NEEDLE_NEGATIVE_GRIPPER_Z = -1;


  /// PSM related variables
  Eigen::Affine3d psm_one_affine_wrt_lt_camera_; // TODO or the other way round?
  Eigen::Affine3d psm_two_affine_wrt_lt_camera_;
  Eigen::Affine3d gripper_one_affine_wrt_lt_camera_;
  Eigen::Affine3d gripper_two_affine_wrt_lt_camera_;


  /// Planning
  int path_waypoints_ = 21; // TODO set function
  int default_grasp_tf_search_resolution_ = 36;

  Eigen::Affine3d default_preferred_grasp_one_tf_;
  Eigen::Affine3d default_preferred_grasp_two_tf_;

  geometry_msgs::PoseStamped valid_exit_points_list_[] ;
  geometry_msgs::PoseStamped candidate_exit_pts_list_[];

  Eigen::Matrix3d tissue_rotation_wrt_lt_camera_;
  Eigen::Affine3d tissue_affine_wrt_lt_camera_;

  // solution storage
  // TODO delete if never used
  std::vector<Eigen::Affine3d> gripper_one_affines_wrt_camera_;
  std::vector<Eigen::Affine3d> gripper_one_affines_wrt_portal_;
  std::vector<Eigen::Affine3d> gripper_two_affines_wrt_camera_;
  std::vector<Eigen::Affine3d> gripper_two_affines_wrt_portal_;

  Eigen::Vector3d needle_entry_point_;
  Eigen::Vector3d needle_exit_point_;
  // Eigen::Vector3d v_entrance_to_exit_;
  // Eigen::Vector3d v_entrance_to_exit0_;
  Eigen::Vector3d tissue_normal_;

  Eigen::VectorXi ik_ok_array_;


  /// Kinematics Solvers
  davinci_kinematics::Forward fwd_solver_;
  davinci_kinematics::Inverse ik_solver_;

};

#endif //RN_SKILLS_AS_RNNEEDLEDRIVINGPLANNER_H

