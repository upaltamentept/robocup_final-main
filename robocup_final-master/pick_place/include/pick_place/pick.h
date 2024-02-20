#ifndef PICK_H
#define PICK_H

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Bool.h>
#include <project_msgs/LabeledCentroid.h>
#include <moveit_msgs/CollisionObject.h>

class Pick
{
  private:
    moveit::planning_interface::MoveGroupInterface arm_torso_group;
    moveit::planning_interface::MoveGroupInterface gripper_group;

  public:
    Pick():arm_torso_group("arm_torso"), gripper_group("gripper") {};

  public:
    bool init();
    void update();

  /* Robot Motion */
  public:
    void pick();
  
  private:
    void lowerTorso();
    void prePickApproach();
    void openGripper();
    void toPickPose();
    void closeGripper();
    void higherTorso();
    void postPickRetreat();
    void toTransportPose();

  private:
    void setWorkspace();

  /* ROS Communication */
  public:
    ros::NodeHandle nh_;

  private:
    ros::Subscriber pick_target_sub_;   // where is the target object, store this in a local variable

    ros::Publisher gripper_pub_;
    ros::Publisher torso_pub_;
    ros::Publisher pick_done_pub_;
    ros::Publisher label_pick_pub_;

  private:
    std::string pick_comm_topic_;
    std::string pick_target_topic_;

    std::string pick_done_topic_;

    trajectory_msgs::JointTrajectory gripper_close_value_;

  private:
    void poseCallback(const project_msgs::LabeledCentroid::ConstPtr &msg);
    int object_label_;

  /* Local Variables */
  private:
    // for ROS
    bool command_;
    std_msgs::Bool pick_done_;

    project_msgs::LabeledCentroid labeled_centroid_;

    // some assistance vars
    std::string ref_frame_;
    Eigen::Vector3d target_position_;
    std::vector<double> transport_value_;
    geometry_msgs::PoseStamped pre_approach_pose_;
    geometry_msgs::Pose pick_pose_;
    geometry_msgs::Pose retreat_pose_;
    geometry_msgs::Pose transport_pose_;

  /* MoveIt */
  private:
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan_;
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan_;
    moveit::planning_interface::PlanningSceneInterface PSI_;
    std::vector<std::string> object_names_;

};

#endif