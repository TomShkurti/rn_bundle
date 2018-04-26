//
// Created by william on 26/04/18.
// @ HMS Prince of Wales
//

#include "rn_skills_as/RnNeedleDrivingPlanner.h"

using std::cout;
using std::cin;
using std::endl;
using std::string;

void showWelcomePage();

int main(int argc, char **argv) {

  /// ROS
  ros::init(argc, argv, "demo_needle_planner_stand_alone");
  ros::NodeHandle node;

  /// Needle Planner
  RnNeedleDrivingPlanner rnNeedleDrivingPlanner(node);

  /// Introduction
  string user_key;
  showWelcomePage();
  cout << endl
       << "Please input any key to proceed." << endl;
  cin  >> user_key;

  while (1>0) {

    /// Scenario 1
    bool test;
    double user_x, user_y, user_z;
    Eigen::Vector3d pt_entry, pt_exit;
    geometry_msgs::PointStamped needle_entry_pt;
    geometry_msgs::PointStamped needle_exit_pt;
    trajectory_msgs::JointTrajectory needleDriveTraj; // OUTPUT
    geometry_msgs::TransformStamped grasp_transform;
    int arm = 1;

    cout << " You are going to specify everything in \e[1mPSM one Base Frame\e[0m." << endl;
    cout << " * Use Default Grasp Transform" << endl;
    cout << " Type in your \e[1mentry point\e[0mcooridinate w/rt Psm 1 frame." << endl;
    cout << " x: ";
    cin >> user_x;
    cout << " y: ";
    cin >> user_y;
    cout << " z: ";
    cin >> user_z;
    pt_entry << user_x, user_y, user_z;
    cout << "\n Type in your \e[1mexit point\e[0mcooridinate w/rt Psm 1 frame." << endl;
    cout << " x: ";
    cin >> user_x;
    cout << " y: ";
    cin >> user_y;
    cout << " z: ";
    cin >> user_z;
    pt_exit << user_x, user_y, user_z;

    needle_entry_pt = rnNeedleDrivingPlanner.ConvertEigenVec3dToGeoMsgsPtStamped(pt_entry);
    needle_exit_pt = rnNeedleDrivingPlanner.ConvertEigenVec3dToGeoMsgsPtStamped(pt_exit);

    test = rnNeedleDrivingPlanner.requestNeedleDrivingTrajectoryDefaultGraspInBaseFrame(arm,
                                                                                        needle_entry_pt,
                                                                                        needle_exit_pt,
                                                                                        needleDriveTraj);

    cout << "If you have failed to find a trajectory for designated entry and exit points, try to engage the auto-search mode -- the planner will try to find a grasp tf for you that can accomplish the task." << endl;

    cout << "Type in any key to proceed:";
    cin >> user_key;

    test = rnNeedleDrivingPlanner.requestOneNeedleDrivingTrajectoryInBaseFrame(arm,
                                                                               needle_entry_pt,
                                                                               needle_exit_pt,
                                                                               needleDriveTraj,
                                                                               grasp_transform);


  }


return 0;

}



void showWelcomePage() {

  cout << endl;
  cout << "\e[1m--------- DEMO -----------" << endl
       << "NEELDLE PLANNER INTERFACES \e[0m" << endl;
  const time_t ctt = time(0);
  cout << asctime(localtime(&ctt)) << endl;

  cout << "  This demo shows you how to use some basic interfaces provided by the Needle Driving Planner Class to request a da Vinci robot trajectory for execution." << endl;

  cout << "  You can either use the PSM base or the Left Camera Optical frame to specify your entry and exit points." << endl;

  cout << endl;
  cout <<   "                                * base_z" << endl;
  for (int i = 0; i < 6; i++) {
    cout << "                                *" << endl;
  }
  cout <<   "                                * base_origin" << endl;
  cout <<   "     PSM SIDE                   * * * * * * * * *              HUMAN SIDE" << endl;
  cout <<   "                              *               base_y" << endl;
  for (int i = 0; i < 3; i ++) {

    int n = 28 - 2*i;
    for (int j = 0; j < n; j ++) {
      cout << " ";
    }
    cout << "*" << endl;
  }
  cout << "              base_x  *" << endl;

  for (int i = 0; i < 3; i ++) {
    cout << endl;
  }

  cout <<    "                            _________" << endl;
  for (int i = 0; i < 4; i++) {
    cout <<  "                            |       |" << endl;
  }
  cout <<    "                            --------- WORK SPACE" << endl << endl;

}
