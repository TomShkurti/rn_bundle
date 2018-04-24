//
// Created by william on 24/04/18.
//

#ifndef RN_SKILLS_AS_DAVINCISKILLSACTIONSERVER_H
#define RN_SKILLS_AS_DAVINCISKILLSACTIONSERVER_H

#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <vector>
#include <string>
#include <unordered_map>

#include <actionlib/server/simple_action_server.h>
#include <rn_skills_as/NeedleDriveLiteAction.h>

#include <ctime>
#include <iostream>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <cwru_davinci_control/psm_controller.h>

#include "rn_skills_as/rn_davinci_skills_base.h"
#include "rn_skills_as/RnNeedleDrivingPlanner.h"

using std::string;

class DavinciSkillsActionServer {
 public:

  DavinciSkillsActionServer(ros::NodeHandle nodeHandle, string name);

  ~DavinciSkillsActionServer(void){};

  void executeCB(const rn_skills_as::NeedleDriveLiteGoalConstPtr &goal);

  void movePSM(psm_controller &psm);

// rn_skills_as


 private:

  ros::NodeHandle nh_;

  /// AS & msgs
  actionlib::SimpleActionServer<rn_skills_as::NeedleDriveLiteAction> as_;
  rn_skills_as::NeedleDriveLiteGoal goal_;
  rn_skills_as::NeedleDriveLiteFeedback feedback_;
  rn_skills_as::NeedleDriveLiteResult result_;

  /// Needle drive planner
  RnNeedleDrivingPlanner rnNeedleDrivingPlanner;

  void go(
      psm_controller & psm,
      double x,
      double y,
      double z,
      double a,
      double g,
      double t
  );

  /// Controller
  void executeTrajectory(psm_controller & psm,
                         trajectory_msgs::JointTrajectory &des_trajectory);




};

#endif //RN_SKILLS_AS_DAVINCISKILLSACTIONSERVER_H
