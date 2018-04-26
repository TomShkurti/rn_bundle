# Da Vinci Skills Action Server

## INTRODUCTION

This action server contains all the Skills that are currently available. You can either use a ROS action client as an interface to request actions through a ROS action server, which manages all the skills behind the scene, or you can instantiate the skill objects into your own code so you can use the internal interfaces directly in your code.

## COMPONENT LIST

### ACTION SERVER

1. Master Skills Action Server & Client (Not complete)

### Skill Libraries

1. Needle Driving Planner
2. Needle Extraction Planner (Not complete)
3. Suture Knot Tying Planner (Not complete)
4. Suture Pulling Planner (Not Complete)

## How to use

### Needle Driving Planner

Make sure your da Vinci robot is loaded (either in a Gazebo simulation or a real one). If you want to use da Vinci's perception for needle planning (a.k.a. you are going to specify your entry and exit points in the da Vinci Left Camera Frame), please check you can see the following links:
```
left_camera_optical_frame
one_psm_base_link
```
However, you can ignore this if you are going to use PSM base frames to specify the entry and exit points. You don't even need to run a da Vinci in this case. 

#### Code Tips


1. First of all, create an object of Class RnNeedleDrivingPlanner
```
RnNeedleDrivingPlanner rnNeedleDrivingPlanner(node);
```

2. To specify the **entry** and **exit point**, use **default grasp transform**:
```
rnNeedleDrivingPlanner.requestNeedleDrivingTrajectoryDefaultGrasp(arm, needle_entry_pt, needle_exit_pt, needleDriveTraj);
```
The output trajectory is written into ```needleDriveTraj```.

3.  To specify the **entry** and **exit point**, use **custom grasp transform**:
```
rnNeedleDrivingPlanner.requestOneNeedleDrivingTrajectory(arm, needle_entry_pt, needle_exit_pt, custom_grasp_transform, needleDriveTraj);
```
The output trajectory is written into ```needleDriveTraj```.

4. To specify only the **entry** and **exit point**, you can also use the Needle Planner to automatically search for a valid grasp transfrom that can do the task. There are occasions that you find your designated grasp transform cannot work, then you would need this.
```
rnNeedleDrivingPlanner.requestOneNeedleDrivingTrajectory(arm, needle_entry_pt, needle_exit_pt, needleDriveTraj, grasp_transform);
```
Note that in this case, the ```grasp_transform``` is the grasp transform acquired automatically by the function. The output trajectory is written into ```needleDriveTraj```.

5. After you get a trajectory of type ```trajectory_msgs::JointTrajectory &joint_trajectory```, you can use this function to change to velocity of your trajectory. 
```
void convertAffinesToTrajectoryMsgs(const std::vector<Eigen::Affine3d> &gripper_affines_wrt_portal, trajectory_msgs::JointTrajectory &joint_trajectory);
```                                     

6. If you do **not** wish to use the left camera frame and want to specify points in the PSM base frame. You can use these functions to do exactly the same job:
```
rnNeedleDrivingPlanner.requestNeedleDrivingTrajectoryDefaultGraspInBaseFrame(arm, needle_entry_pt, needle_exit_pt, needleDriveTraj);
```
```
rnNeedleDrivingPlanner.requestOneNeedleDrivingTrajectoryInBaseFrame(arm, needle_entry_pt, needle_exit_pt, custom_grasp_transform, needleDriveTraj);
```
```
rnNeedleDrivingPlanner.requestOneNeedleDrivingTrajectoryInBaseFrame(arm, needle_entry_pt, needle_exit_pt, needleDriveTraj, grasp_transform);
```
Note that they only add a suffix -InBaseFrame to the Camera frame version of functions.

7. There are many other utility functions that can be helpful in your programming. 

* To convert an Eigen::Affine3d to a trajectory_msgs::JointTrajectory messgae
```   
   convertAffinesToTrajectoryMsgs
```
* To convert a geometry_msgs::PointStamped to an Eigen::Vector3d
```
   convertPointStampedToEigenVector
```
* To convert a tf::Transform to an Eigen::Affine3d
```
   convertTFToEigen
```
* To convert a geometry_msgs::TransformStamped to an Eigen::Affine3d
```
convertGeoTransformStampedToEigenAffine
```
* To convert an Eigen::Affine to a geometry_msgs::TransformStamped
```
   convertEigenAffineToGeoTransform
```

8. There are a bunch of set() function for changing some default values.

| Subject                            | Function           | 
| -------------                      |:-------------:| 
| Needle Radius                      | setNeedleRaidus(double new_needle_radius) | 
| Needle Grasp Depth                 | setNeedleGraspDepth(double new_grasp_depth)     | 
| Needle Axis Height                 | setNeedleAxisHt(double new_axis_ht)     |  
| Default Grasp Tf search resolution | setDefaultGraspTfSearchResolution(int resolution)      |  



#### Run Steps

1. Launch the config file
```
roslaunch rn_skills_as test_params.launch 
```
2. Run your main code.