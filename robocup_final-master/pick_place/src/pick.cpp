#include <pick_place/pick.h>

bool Pick::init()
{
  // Topic definitions
  pick_target_topic_ =  "/labeled_objects";
  pick_done_topic_ = "/pick_done";

  // Subscribers and publishers
  pick_target_sub_ = nh_.subscribe(pick_target_topic_, 1, &Pick::poseCallback, this);

  gripper_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/gripper_controller/command", 1);
  torso_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/torso_controller/command", 1);
  pick_done_pub_ = nh_.advertise<std_msgs::Bool>(pick_done_topic_, 1);
  label_pick_pub_ = nh_.advertise<project_msgs::LabeledCentroid>("/label_pick", 1);

  // Initialize flags
  command_ = false;
  pick_done_.data = true;
  
  // Pre-defined poses
  transport_value_ = {0.35, 0.1, -0.3, -0.3, 2.269, -1.57, 0.45, 0.2};
  
  gripper_close_value_.joint_names.resize(2);
  gripper_close_value_.joint_names[0] = "gripper_left_finger_joint";
  gripper_close_value_.joint_names[1] = "gripper_right_finger_joint";

  gripper_close_value_.points.resize(1);
  gripper_close_value_.points[0].positions.resize(2);
  gripper_close_value_.points[0].positions[0] = 0;
  gripper_close_value_.points[0].positions[1] = 0;
  gripper_close_value_.points[0].time_from_start = ros::Duration(1.0);
  
  // Set values for MoveIt
  ref_frame_ = "base_footprint";

  arm_torso_group.setPlannerId("RRTConnectkConfigDefault");
  arm_torso_group.setPlanningTime(30.0);
  arm_torso_group.setMaxAccelerationScalingFactor(0.3);
  arm_torso_group.setMaxVelocityScalingFactor(0.3);
  arm_torso_group.setEndEffectorLink("arm_tool_link");

  // Initialize TIAGo
  openGripper();

  return true;
}

void Pick::update()
{
  if (command_)
  {
    // Wait for 2 sec to update the planning scene
    ros::Duration(2.0).sleep();

    // reset the joint value
    
    // Perform pick
    pick();

    // Publish msgs that are needed
    pick_done_pub_.publish(pick_done_);
    label_pick_pub_.publish(labeled_centroid_);

    // Clear all collision objects in the planning scene
    object_names_ = PSI_.getKnownObjectNames();

    ROS_INFO_STREAM("Number of collision objects in the scene: " << object_names_.size());
    
    PSI_.removeCollisionObjects(object_names_);
    object_names_.clear();

    command_ = false;
  }
}

void Pick::pick()
{
  lowerTorso();
  openGripper();
  prePickApproach();
  toPickPose();
  closeGripper();
  higherTorso();
  postPickRetreat();
  toTransportPose();
}

void Pick::lowerTorso()
{
  trajectory_msgs::JointTrajectory torso_lower_value;

  torso_lower_value.joint_names.resize(1);
  torso_lower_value.joint_names[0] = "torso_lift_joint";

  torso_lower_value.points.resize(1);
  torso_lower_value.points[0].positions.resize(1);
  torso_lower_value.points[0].positions[0] = 0.1;
  torso_lower_value.points[0].time_from_start = ros::Duration(1.0);

  torso_pub_.publish(torso_lower_value);
  ros::Duration(5.0).sleep();
}

void Pick::prePickApproach()
{
  std::vector<double> ready_value;
  ready_value = {0.15, 0.175, 0.349, -3.14, 1.047, 1.57, 0, 0};

  arm_torso_group.setPoseTarget(pre_approach_pose_);
  bool succ = (arm_torso_group.plan(arm_plan_) == moveit_msgs::MoveItErrorCodes::SUCCESS);

  if (!succ)
  {
    ROS_INFO_STREAM("Planning failed");
  }

  arm_torso_group.move();
  ROS_INFO_STREAM("Pre-pick goal reached");
}

void Pick::openGripper()
{
  std::vector<double> open_value = {0.044, 0.044};
  
  gripper_group.setJointValueTarget(open_value);
  gripper_group.plan(gripper_plan_);
  gripper_group.move();
  ROS_INFO_STREAM("Gripper opened");
}

void Pick::toPickPose()
{
  pick_pose_ = pre_approach_pose_.pose;
  pick_pose_.position.x += 0.26;
  // pick_pose_.position.z -= 0.025;

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(pick_pose_);

  moveit_msgs::RobotTrajectory trajectory;

  double eef_step = 0.01;  // Resolution of the Cartesian path
  double jump_threshold = 0.0;  // No jump threshold

  double fraction = arm_torso_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  int trajectory_points = trajectory.joint_trajectory.points.size();
  ROS_INFO_STREAM("Number of points in the trajectory = " << trajectory_points);

  arm_torso_group.execute(trajectory);

  geometry_msgs::PoseStamped pick_pose;
  pick_pose = arm_torso_group.getCurrentPose("arm_tool_link");
  ROS_INFO_STREAM("Arrived at: " << pick_pose.pose.position);
}

void Pick::closeGripper()
{
  gripper_pub_.publish(gripper_close_value_);
  ros::Duration(1.0).sleep();
}

void Pick::higherTorso()
{
  // higher the torso for safty reason
  trajectory_msgs::JointTrajectory torso_higher_value;

  torso_higher_value.joint_names.resize(1);
  torso_higher_value.joint_names[0] = "torso_lift_joint";

  torso_higher_value.points.resize(1);
  torso_higher_value.points[0].positions.resize(1);
  torso_higher_value.points[0].positions[0] = 0.32;
  torso_higher_value.points[0].time_from_start = ros::Duration(1.0);

  torso_pub_.publish(torso_higher_value);
  ros::Duration(5.0).sleep();
}

void Pick::postPickRetreat()
{
  retreat_pose_ = pick_pose_;
  retreat_pose_.position.z += 0.05;

  geometry_msgs::Pose retreat_pose_1;
  retreat_pose_1 = retreat_pose_;
  retreat_pose_1.position.x -= 0.28;

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(retreat_pose_);
  waypoints.push_back(retreat_pose_1);

  moveit_msgs::RobotTrajectory trajectory;

  double eef_step = 0.01;  // Resolution of the Cartesian path
  double jump_threshold = 0.0;  // No jump threshold

  double fraction = arm_torso_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  arm_torso_group.execute(trajectory);

  ROS_INFO_STREAM("Retreated");
}

void Pick::toTransportPose()
{
  arm_torso_group.setJointValueTarget(transport_value_);
  arm_torso_group.plan(arm_plan_);
  arm_torso_group.move();

  ROS_INFO_STREAM("Ready to go");
}

void Pick::poseCallback(const project_msgs::LabeledCentroid::ConstPtr &msg)
{
  labeled_centroid_ = *msg;

  command_ = true;

  object_label_ = msg->label;

  target_position_ << msg->pose.pose.position.x,
                      msg->pose.pose.position.y,
                      msg->pose.pose.position.z;

  pre_approach_pose_.header.frame_id = ref_frame_;
  pre_approach_pose_.pose.position = msg->pose.pose.position;

  pre_approach_pose_.pose.position.x -= 0.415;    // Length of the gripper = 0.215
  pre_approach_pose_.pose.position.z += 0.02;

  tf2::Quaternion quaternion;
  quaternion.setRPY(1.57, 0.0, 0.0);

  pre_approach_pose_.pose.orientation = tf2::toMsg(quaternion);
  
  ROS_INFO_STREAM("Command received, position: " << target_position_.transpose());
}