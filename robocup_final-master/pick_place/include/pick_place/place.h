#ifndef PLACE_H
#define PLACE_H

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_msgs/Bool.h>

class Place
{
  private:
    moveit::planning_interface::MoveGroupInterface arm_torso_group;
    moveit::planning_interface::MoveGroupInterface arm_group;
    moveit::planning_interface::MoveGroupInterface gripper_group;

  public:
    Place():arm_torso_group("arm_torso"), arm_group("arm"), gripper_group("gripper") {};

  public:
    bool init();
    void update();

  /* Robot Motion */
  public:
    void place();
  
  private:
    void lowerTorso();
    void prePlaceApproach();
    void toPlacePose();
    void openGripper();
    void postPlaceRetreat();
    void closeGripper();
    void homing();
    void higherTorso();

  /* ROS Communication */
  public:
    ros::NodeHandle nh_;

  private:
    ros::Subscriber place_target_sub_;   // where is the target object, store this in a local variable

    ros::Publisher torso_pub_;
    ros::Publisher gripper_pub_;
    ros::Publisher move_base_pub_;
    ros::Publisher place_done_pub_;

  public:
    std::string place_comm_topic_;
    std::string place_target_topic_;

    std::string place_done_topic_;

  private:
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

  /* Local Variables */
  private:
    // for ROS
    bool command_;
    bool lower_torso_;
    std_msgs::Bool place_done_;

    int counter_;         // After 3 place processes, higher the torso for the pick on the high table

    // some assistance vars
    std::string ref_frame_;
    Eigen::Vector3d target_position_;
    std::vector<double> arm_home_value_;
    // std::vector<double> gripper_open_value_;
    std::vector<double> gripper_close_value_;
    geometry_msgs::PoseStamped pre_approach_pose_;
    geometry_msgs::Pose place_pose_;
    geometry_msgs::Pose retreat_pose_;

    trajectory_msgs::JointTrajectory gripper_open_value_;

  /* MoveIt */
  private:
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan_;
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan_;
    moveit::planning_interface::PlanningSceneInterface PSI_;
};

#endif