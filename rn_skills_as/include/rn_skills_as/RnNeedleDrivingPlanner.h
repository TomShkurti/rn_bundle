//
// Created by William on 28/03/18.
//

#ifndef RN_SKILLS_AS_RNNEEDLEDRIVINGPLANNER_H
#define RN_SKILLS_AS_RNNEEDLEDRIVINGPLANNER_H

#include "rn_skills_as/rn_davinci_skills_base.h"

using namespace rn_davinci_skills;

using namespace davinci_kinematics;

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

  /// TODO add & finish (to accommodate Orhan's usage)
  void computeGraspTransfrom(int arm_index,
                             const geometry_msgs::TransformStamped &gripper_to_needle_transform);


  double computeNeedleDriveGripperAffines(int arm_index,
                                          trajectory_msgs::JointTrajectory &needleDriveTraj);

  /**
   * This function will get you a trajectory that start from phi_from_initial_0 and end at phi_from_initial_t.
   * It will take only one drive.
   * @param arm_index
   * @param phi_from_initial_0: Initial is 0. Phi is the so called needle insertion angle. 0 indiates it is where the drive starts.
   * @param phi_from_initial_t: destination.
   * @param needleDriveTraj
   * @return
   */
  double computeNeedleDriveGripperAffines(int arm_index,
                                          double phi_from_initial_0,
                                          double phi_from_initial_t,
                                          trajectory_msgs::JointTrajectory &needleDriveTraj);

  bool hasValidNeedleDriveAffine(int arm_index, double phi);



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
  bool requestOneNeedleDrivingTrajectoryUserGrasp(const int &arm_index,
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
  bool requestOneNeedleDrivingTrajectoryGeneratedGrasp(const int &arm_index,
                                         const geometry_msgs::PointStamped &needle_entry_pt,
                                         const geometry_msgs::PointStamped &needle_exit_pt,
                                         trajectory_msgs::JointTrajectory &needleDriveTraj,
                                         geometry_msgs::TransformStamped &grasp_transform);



  bool requestNeedleDrivingTrajectoryDefaultGrasp(const int &arm_index,
                                                  const geometry_msgs::PointStamped &needle_entry_pt,
                                                  const geometry_msgs::PointStamped &needle_exit_pt,
                                                  const double phi_0,
                                                  const double phi_t,
                                                  trajectory_msgs::JointTrajectory &needleDriveTraj);


  bool requestOneNeedleDrivingTrajectoryUserGrasp(const int &arm_index,
                                                  const geometry_msgs::PointStamped &needle_entry_pt,
                                                  const geometry_msgs::PointStamped &needle_exit_pt,
                                                  const geometry_msgs::TransformStamped &grasp_transform,
                                                  const double phi_0,
                                                  const double phi_t,
                                                  trajectory_msgs::JointTrajectory &needleDriveTraj);


  bool requestOneNeedleDrivingTrajectoryGeneratedGrasp(const int &arm_index,
                                                       const geometry_msgs::PointStamped &needle_entry_pt,
                                                       const geometry_msgs::PointStamped &needle_exit_pt,
                                                       const double phi_0,
                                                       const double phi_t,
                                                       trajectory_msgs::JointTrajectory &needleDriveTraj,
                                                       geometry_msgs::TransformStamped &grasp_transform);


  void updateNeedleDriveKinematicBoundary(const int &arm_index);

  // To be used only after you update the grasp transform(s).
  void updatePsmKinematicAvailability(const int &arm_index);






  // TODO finish

  /// Needle Extraction related functions

  // This is only useful when the tip_pt is below needle centre.
  void updateNeedleAndTissueParametersWithExitAndTip(const geometry_msgs::PointStamped &needle_exit_pt,
                                                     const geometry_msgs::PointStamped &needle_tip_pt);

// This is only useful when the tip_pt is below needle centre.
  void defineTissueFrameWrtLtCameraViaExitAndTip(const Eigen::Vector3d &exit_pt,
                                                 const Eigen::Vector3d &tip_pt,
                                                 const Eigen::Vector3d &tissue_normal,
                                                 Eigen::Vector3d & entry_pt);


  bool getNeedleExtractionTrajectory(const int &arm_index);



  // should be using needle wrt cam.
  bool getNeedlePickingTrajectory();



  // should generate 2 psms' trajectories in a synchronised manner.
  bool getNeedleHandoverTrajectory();


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


  // TODO finish

  /**
   * Try to find a grasp transform with an allowed perturbation from the user specified grasp transform, which allows
   * a valid trajectory for needle driving.
   * @param arm_index
   * @param needle_entry_pt
   * @param needle_exit_pt
   * @param user_grasp_transform
   * @param perturbation: Ranging from 0 to 1
   * @param needleDriveTraj
   * @param final_grasp_transform
   * @return
   */
  bool requestOneNeedleDrivingTrajectory(const int &arm_index,
                                         const geometry_msgs::PointStamped &needle_entry_pt,
                                         const geometry_msgs::PointStamped &needle_exit_pt,
                                         const geometry_msgs::TransformStamped &user_grasp_transform,
                                         const double &perturbation,
                                         trajectory_msgs::JointTrajectory &needleDriveTraj,
                                         geometry_msgs::TransformStamped &final_grasp_transform);

  void generateGraspTransformListWithPerturbation (const int &arm_index,
                                                   const geometry_msgs::TransformStamped &user_grasp_transform,
                                                   const double &perturbation,
                                                   cwru_davinci_msgs::ListOfTransformStamped &grasp_tf_array);


  /// Auxiliary Utility Functions

  double approximateTrajectoryDist(trajectory_msgs::JointTrajectory &joint_trajectory);

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

  /**
   * Solves circle equations with known 2 pts and radius.
   * @param x1
   * @param y1
   * @param x2
   * @param y2
   * @param r
   * @param origin_x1
   * @param origin_y1
   * @param origin_x2
   * @param origin_y2
   */
  void solveBinaryQuadraticCircleEquation(double x1,
                                          double y1,
                                          double x2,
                                          double y2,
                                          double r,
                                          double &origin_x1,
                                          double &origin_y1,
                                          double &origin_x2,
                                          double &origin_y2);


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

  inline void setPathWaypoints (int number) {
    path_waypoints_ = number;
    ik_ok_array_.resize(path_waypoints_);
    ROS_INFO("path_waypoints_ has been updated.");
  }

  inline void setAccelerationRate (double acc) {
    acceleration_ = acc; // m/s^2
    ROS_INFO("acceleration_ has been updated.");
  }


  /// get() functions

  inline Eigen::Affine3d getDefaultNeedleAffineWrtGraspOneFrame() {
    return default_needle_affine_wrt_grasp_one_frame_;
  }


  inline double getPhiNeedleInitial() {
    return phi_needle_initial_;
  }

  inline double getPhiNeedlePenetration() {
    return phi_needle_penetration_;
  }

  inline double getPhiNeedleEmergence() {
    return phi_needle_emergence_;
  }


  inline double getPhiNeedleKinematicLowerAnglePsm1() {
    return phi_needle_kinematic_lower_angle_psm_1_;
  }

  inline double getPhiNeedleKinematicLowerAnglePsm2() {
    return phi_needle_kinematic_lower_angle_psm_2_;
  }

  inline double getPhiNeedleKinematicUpperAnglePsm1() {
    return phi_needle_kinematic_upper_angle_psm_1_;
  }

  inline double getPhiNeedleKinematicUpperAnglePsm2() {
    return phi_needle_kinematic_upper_angle_psm_2_;
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


  bool requestOneNeedleDrivingTrajectoryUserGraspInBaseFrame(const int &arm_index,
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


  /// Dual arm related functions

  void generateDualPsmOpBoundaryVertices();



  /// Debugging functions

  Eigen::Vector3d transformPointFromBaseToLtCamFrame(const int & arm_index,
                                                     const Eigen::Vector3d &point);


  Eigen::Vector3d transformPointFromLtCamFrameToBase(const int & arm_index,
                                                     const Eigen::Vector3d &point);

  inline void printPsmBaseToCamTransfomrs() {

    ROS_WARN("DEBUG -- printing psm_one_affine_wrt_lt_camera_: ");
    printEigenAffine(psm_one_affine_wrt_lt_camera_);
    ROS_WARN("DEBUG -- printing psm_two_affine_wrt_lt_camera_: ");
    printEigenAffine(psm_two_affine_wrt_lt_camera_);

  }

  // psi_needle_axis_tilt_wrt_tissue_
  inline void printPsiNeedleAxisTiltWrtTissue () {

    std::cout << "psi_needle_axis_tilt_wrt_tissue_: " << psi_needle_axis_tilt_wrt_tissue_ << std::endl;

  }


  // needle_affine_wrt_gripper_one_frame_
  inline void printNeedleAffineWrtGripperFrames() {

    std::cout << "needle_affine_wrt_gripper_one_frame_: " << std::endl;
    printEigenAffine(needle_affine_wrt_gripper_one_frame_);

    std::cout << "needle_affine_wrt_gripper_two_frame_: " << std::endl;
    printEigenAffine(needle_affine_wrt_gripper_two_frame_);

  }

  inline void printDebugAffineVessel() {

    std::cout << "debug_affine_vessel_: " << std::endl;
    printEigenAffine(debug_affine_vessel_);

  }

  // TODO delete
  inline void printGraspDepth () {
    std::cout << "grasp_depth_: " << grasp_depth_ << std::endl;
  }

  ///



  /// Temporary PSM Controller & Execution functions
  /**
   * An execution function.
   * @param psm
   * @param needleDriveTraj
   */
  void executeTrajectory(psm_controller &psm,
                         const trajectory_msgs::JointTrajectory &needleDriveTraj);

  void executeTrajectory(psm_controller &psm,
                         const cwru_davinci_msgs::ListOfJointTrajectory &list_of_joint_traj);



  void goToLocationPointingDownFaceVector(psm_controller &psm,
                                          const double &x,
                                          const double &y,
                                          const double &z);

  void goToLocationPointingDownFaceForward(psm_controller &psm,
                                          const double &x,
                                          const double &y,
                                          const double &z);

  void goToLocationPointingPsmLeft(psm_controller &psm,
                                           const double &x,
                                           const double &y,
                                           const double &z);

  void goToLocationPointingPsmRight(psm_controller &psm,
                                   const double &x,
                                   const double &y,
                                   const double &z);

  // TODO complete
  void openGripper(psm_controller &psm, const double &angle);



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
  double psi_needle_axis_tilt_wrt_tissue_ = 0; // Not implemented, set to be 0

  // When the needle is rotated for @phi_needle_penetration_ from @phi_needle_initial_ its tip will touch the
  // tissue surface at the entry point. When the needle is rotated for @phi_needle_emergence_ from @phi_needle_penetration_ its tip
  // will emerge again from the exit point.
  double phi_needle_initial_;
  double phi_needle_penetration_;
  double phi_needle_emergence_;
  // When tissue frame (entry and exit pts) is set, and the grasp tf is set, one can calculate the lower and upper boundary
  // of the needle rotation. The results can be additions to the phi_needle_penetration_ and phi_needle_emergence_.
  // This info can be helpful to addressing the multi-drive and extraction issues.
  double phi_needle_kinematic_lower_angle_psm_1_;
  double phi_needle_kinematic_lower_angle_psm_2_;
  double phi_needle_kinematic_upper_angle_psm_1_;
  double phi_needle_kinematic_upper_angle_psm_2_;

  // structs defined in <davinci_skills_base.h>
  NeedleDrivingPhis needle_phis_;
  PmsKinematicAvailability psm_1_kinematic_availability_;
  PmsKinematicAvailability psm_2_kinematic_availability_;

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
  Eigen::Affine3d psm_two_affine_wrt_psm_one_;

  // Dual PSM related
  std::vector<Eigen::Vector3d> dual_op_boundary_pts_in_psm_one_;
  Eigen::Vector3d dual_op_zone_geo_centre_in_psm_one_;


  /// Planning
  int path_waypoints_ = 21; // TODO set function
  int default_grasp_tf_search_resolution_ = 36;

  Eigen::Affine3d default_preferred_grasp_one_tf_;
  Eigen::Affine3d default_preferred_grasp_two_tf_;

  geometry_msgs::PoseStamped valid_exit_points_list_[] ;
  geometry_msgs::PoseStamped candidate_exit_pts_list_[];

  Eigen::Matrix3d tissue_rotation_wrt_lt_camera_;
  Eigen::Affine3d tissue_affine_wrt_lt_camera_;

  // Speed and acceleration
  double acceleration_ = 0.0008; // m/s^2


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
  Eigen::Vector3d tissue_normal_; // w/rt CAMERA frame!

  Eigen::VectorXi ik_ok_array_;

  // Trajectory storage
  cwru_davinci_msgs::ListOfJointTrajectory list_of_trajectory_;
  trajectory_msgs::JointTrajectory trajectory_;


  /// Kinematics Solvers
  davinci_kinematics::Forward fwd_solver_;
  davinci_kinematics::Inverse ik_solver_;


  /// Temporary PSM controllers
  psm_controller psm_one_;
  psm_controller psm_two_;


  /// Debug Auxiliary
  Eigen::Affine3d debug_affine_vessel_;

};

#endif //RN_SKILLS_AS_RNNEEDLEDRIVINGPLANNER_H

