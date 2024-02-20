#include <obstacle_converter/obstacle_converter.h>

void ObstacleConverter::init()
{
  // Init args
  command_ = false;
  ready_for_place_ = false;
  ref_frame_ = "base_footprint";

  // Init subscribers
  obstacle_geometry_sub_ = nh_.subscribe("/remainder_objects", 1, &ObstacleConverter::obstacleCallback, this);
  pick_done_sub_ = nh_.subscribe("/gripper_succeed", 1, &ObstacleConverter::pickCallback, this);

  // Init publisher
  torso_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/torso_controller/command", 1);
}

void ObstacleConverter::update()
{
  if (command_)
  {
    addCollisionObjects();

    // For next construction, delet all info for this interation
    centroids_.clear();
    primitives_.clear();

    command_ = false;
  }
}

void ObstacleConverter::addCollisionObjects()
{
  double table_width, table_centroid;
  
  std::vector<moveit_msgs::CollisionObject> collision_objects_;
  
  collision_objects_.resize(num_collision_objects_);

  if (!ready_for_place_)
  {
    for (int i = 0; i < num_collision_objects_; ++i)
    {
      // Header frame
      collision_objects_[i].header.frame_id = ref_frame_;

      // Poses
      geometry_msgs::Pose box_pose;
      box_pose.orientation.w = 1.0;
      box_pose.position.x = centroids_[i].x;
      box_pose.position.y = centroids_[i].y;
      box_pose.position.z = (centroids_[i].z + primitives_[i].z) / 2;

      // Primitives
      shape_msgs::SolidPrimitive primitive;
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[primitive.BOX_X] = primitives_[i].x + 0.03;
      primitive.dimensions[primitive.BOX_Y] = primitives_[i].y;
      primitive.dimensions[primitive.BOX_Z] = box_pose.position.z * 2;

      // Names
      if (i == num_collision_objects_-1)
      {
        // If the object is the table, we need to adjust some parameters
        collision_objects_[i].id = "table";

        table_width = primitives_[i].y;
        table_centroid = centroids_[i].y;

        primitive.dimensions[primitive.BOX_X] += 0.07;
        primitive.dimensions[primitive.BOX_Y] = 5.0;
      }
      else
      {
        std::string object_name = "collision_object_" + std::to_string(i);
        collision_objects_[i].id = object_name;
      }

      // Add the object
      collision_objects_[i].primitives.push_back(primitive);
      collision_objects_[i].primitive_poses.push_back(box_pose);

      collision_objects_[i].operation = collision_objects_[i].ADD;
    }
  }
  else if (ready_for_place_)
  {
    collision_objects_.resize(4);

    //
    // Cube for lower shelf plane
    collision_objects_[0].header.frame_id = ref_frame_;
 
    collision_objects_[0].primitives.resize(1);
    collision_objects_[0].primitive_poses.resize(1);

    collision_objects_[0].primitives[0].type = collision_objects_[0].primitives[0].BOX;
  
    collision_objects_[0].primitive_poses[0].orientation.w = 1.0;
    collision_objects_[0].primitive_poses[0].position.x = centroids_[0].x;
    collision_objects_[0].primitive_poses[0].position.y = centroids_[0].y;
    collision_objects_[0].primitive_poses[0].position.z = (centroids_[0].z + (primitives_[0].z / 2)) / 2;

    collision_objects_[0].primitives[0].dimensions.resize(3);
    collision_objects_[0].primitives[0].dimensions[0] = primitives_[0].x + 0.1;
    collision_objects_[0].primitives[0].dimensions[1] = 5.0;
    collision_objects_[0].primitives[0].dimensions[2] = collision_objects_[0].primitive_poses[0].position.z * 2;

    collision_objects_[0].id = "shelf_plane_lower";

    collision_objects_[0].operation = collision_objects_[0].ADD;

    // Cube for upper shelf plane

    collision_objects_[1].header.frame_id = ref_frame_;

    collision_objects_[1].primitives.resize(1);
    collision_objects_[1].primitive_poses.resize(1);
    
    collision_objects_[1].primitives[0].type = collision_objects_[1].primitives[0].BOX;

    collision_objects_[1].primitive_poses[0].orientation.w = 1.0;
    collision_objects_[1].primitive_poses[0].position.x = centroids_[0].x;
    collision_objects_[1].primitive_poses[0].position.y = centroids_[0].y;
    collision_objects_[1].primitive_poses[0].position.z = collision_objects_[0].primitives[0].dimensions[2] + 0.72;

    collision_objects_[1].primitives[0].dimensions.resize(3);
    collision_objects_[1].primitives[0].dimensions[0] = primitives_[0].x + 0.1;
    collision_objects_[1].primitives[0].dimensions[1] = 5.0;
    collision_objects_[1].primitives[0].dimensions[2] = 0.72;

    collision_objects_[1].id = "shelf_plane_upper";

    collision_objects_[1].operation = collision_objects_[0].ADD;
    
    // Left & right wall for the shelf frames
    for (int i = 2; i < 4; ++i)
    {
      // Header frame
      collision_objects_[i].header.frame_id = ref_frame_;

      // Poses
      geometry_msgs::Pose wall_pose;
      wall_pose.orientation.w = 1.0;
      wall_pose.position.x = centroids_[0].x;
      wall_pose.position.y = centroids_[0].y - (primitives_[0].y / 2) + (i-2) * (primitives_[0].y);
      wall_pose.position.z = collision_objects_[0].primitive_poses[0].position.z;

      // Primitives
      shape_msgs::SolidPrimitive primitive;
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[primitive.BOX_X] = primitives_[0].x + 0.1;
      primitive.dimensions[primitive.BOX_Y] = 0.01;
      primitive.dimensions[primitive.BOX_Z] = 5;
      std::string wall_name = "wall_" + std::to_string(i - 2);
      collision_objects_[i].id = wall_name;

      collision_objects_[i].primitives.push_back(primitive);
      collision_objects_[i].primitive_poses.push_back(wall_pose);
      collision_objects_[i].operation = collision_objects_[i].ADD;
    }
  }
  else
  {
    ROS_WARN_STREAM("Message Conflict");
  }

  PSI.applyCollisionObjects(collision_objects_);

  ready_for_place_ = false;
}

void ObstacleConverter::obstacleCallback(const project_msgs::CollisionGeometry::ConstPtr& msg)
{
  // How many objects should there be
  num_collision_objects_ = msg->x.size();
  
  centroids_.resize(num_collision_objects_);
  primitives_.resize(num_collision_objects_);
  
  for (int i = 0; i < num_collision_objects_; ++i)
  {
    centroids_[i].x = msg->x[i];
    centroids_[i].y = msg->y[i];
    centroids_[i].z = msg->z[i];

    primitives_[i].x = msg->length[i];
    primitives_[i].y = msg->width[i];
    primitives_[i].z = msg->height[i];
  }

  command_ = true;
}

void ObstacleConverter::pickCallback(const std_msgs::Bool::ConstPtr& msg)
{
  ready_for_place_ = msg->data;

  // If pick failed, lower the torso for better view for YOLO
  if (ready_for_place_)
  {  
    ROS_INFO_STREAM("Collision object is going to be construct for PLACE");
  }
  else
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
}