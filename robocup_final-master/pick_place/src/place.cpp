#include <pick_place/place.h>

bool Place::init()
{
  place_target_topic_ =  "/place_pose";
  place_done_topic_ = "/place_done";

  place_target_sub_ = nh_.subscribe(place_target_topic_, 1, &Place::poseCallback, this);

  torso_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/torso_controller/command", 1);
  place_done_pub_ = nh_.advertise<std_msgs::Bool>(place_done_topic_, 1);
  gripper_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/gripper_controller/command", 1);
  move_base_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 1);

  command_ = false;
  place_done_.data = true;

  arm_home_value_ = {0.15, 0.19, -1.34, -0.19, 1.94, -1.57, 1.36, 0};

  gripper_open_value_.joint_names.resize(2);
  gripper_open_value_.joint_names[0] = "gripper_left_finger_joint";
  gripper_open_value_.joint_names[1] = "gripper_right_finger_joint";

  gripper_open_value_.points.resize(1);
  gripper_open_value_.points[0].positions.resize(2);
  gripper_open_value_.points[0].positions[0] = 1;
  gripper_open_value_.points[0].positions[1] = 1;
  gripper_open_value_.points[0].time_from_start = ros::Duration(1.0);

  ref_frame_ = "base_footprint";

  arm_torso_group.setPlannerId("RRTConnectkConfigDefault");
  arm_torso_group.setPlanningTime(30.0);
  arm_torso_group.setMaxAccelerationScalingFactor(0.2);
  arm_torso_group.setMaxVelocityScalingFactor(0.2);
  arm_torso_group.setEndEffectorLink("arm_tool_link");

  counter_ = 0;

  return true;
}

void Place::update()
{
  if (command_)
  {
    ros::Duration(2.0).sleep();
    place();
    
    place_done_pub_.publish(place_done_);

    // Clear all collision objects in the planning scene
    std::vector<std::string> object_names;
    object_names = PSI_.getKnownObjectNames();
    ROS_INFO_STREAM("Number of collision objects in the scene: " << object_names.size());
    PSI_.removeCollisionObjects(object_names);
    object_names.clear();

    command_ = false;
  }
}

void Place::place()
{
  if (lower_torso_)
  {
    lowerTorso();
  }
  prePlaceApproach();
  toPlacePose();
  openGripper();
  postPlaceRetreat();
  homing();
  if (counter_ >= 3)
  {
    higherTorso();
  }
}

void Place::lowerTorso()
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

void Place::prePlaceApproach()
{
  arm_torso_group.setPoseTarget(pre_approach_pose_);
  bool succ = (arm_torso_group.plan(arm_plan_) == moveit_msgs::MoveItErrorCodes::SUCCESS);

  if (!succ)
  {
    ROS_INFO_STREAM("Planning failed");
  }

  arm_torso_group.move();

  ROS_INFO_STREAM("Pre-place goal reached");
}

void Place::toPlacePose()
{
  place_pose_ = pre_approach_pose_.pose;
  place_pose_.position.x += 0.28;

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(place_pose_);

  moveit_msgs::RobotTrajectory trajectory;

  double eef_step = 0.01;  // Resolution of the Cartesian path
  double jump_threshold = 0.0;  // No jump threshold

  double fraction = arm_torso_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  arm_torso_group.execute(trajectory);
}

void Place::openGripper()
{
  gripper_pub_.publish(gripper_open_value_);
  ros::Duration(1.0).sleep();
}

void Place::postPlaceRetreat()
{
  retreat_pose_ = place_pose_;
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
}

void Place::homing()
{
  // For safty when tucking the arm, retreat the base
  ros::Time start_time = ros::Time::now();
  ros::Duration timeout(2.0); // Timeout of 2 seconds
  while (ros::Time::now() - start_time < timeout)
  {
    geometry_msgs::Twist move_base_retreat;

    // Here you build your twist message
    move_base_retreat.linear.x = -0.2;
    move_base_retreat.linear.y = 0.0;
    move_base_retreat.linear.z = 0.0;

    move_base_retreat.angular.x = 0.0;
    move_base_retreat.angular.y = 0.0;
    move_base_retreat.angular.z = 0.3;

    move_base_pub_.publish(move_base_retreat);
  }

  // Tuck the arm
  std::system("rosrun tiago_gazebo tuck_arm.py");
}

void Place::higherTorso()
{
  // higher the torso for a better view at kitchen
  trajectory_msgs::JointTrajectory torso_higher_value;

  torso_higher_value.joint_names.resize(1);
  torso_higher_value.joint_names[0] = "torso_lift_joint";

  torso_higher_value.points.resize(1);
  torso_higher_value.points[0].positions.resize(1);
  torso_higher_value.points[0].positions[0] = 0.25;
  torso_higher_value.points[0].time_from_start = ros::Duration(1.0);

  torso_pub_.publish(torso_higher_value);
  ros::Duration(5.0).sleep();
}

void Place::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  command_ = true;

  target_position_ << msg->pose.position.x,
               msg->pose.position.y,
               msg->pose.position.z;

  pre_approach_pose_.header.frame_id = ref_frame_;
  pre_approach_pose_.pose.position = msg->pose.position;

  pre_approach_pose_.pose.position.x -= 0.415;
  pre_approach_pose_.pose.position.z += 0.02;

  if (msg -> pose.position.z < 0.8)
  {
    lower_torso_ = true;
  }

  tf2::Quaternion quaternion;
  quaternion.setRPY(1.57, 0.0, 0.0);

  pre_approach_pose_.pose.orientation = tf2::toMsg(quaternion);
  
  ROS_INFO_STREAM("Command received, position: " << target_position_.transpose());

  // After 3 place, higher the torso
  counter_ += 1;
}