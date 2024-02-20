#include <place_pose/place_pose.h>

bool PlacePose::initalize(ros::NodeHandle& nh_)
{
  
  object_labeled_sub_ = nh_.subscribe
                     ("/labeled_objects", 1, &PlacePose::ObjectPoseCallback, this); //subscribe the target object
  place_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/place_pose", 1); // publish the place pose 
  shelf_box_pub_ = nh_.advertise<project_msgs::CollisionGeometry>("/remainder_objects", 1); //publish the obstacle to generate object in planning scene

  table_cloud_sub_ = nh_.subscribe("/table_plane_pointcloud", 1, &PlacePose::TablePoseCallback, this);//subscibe table plane

  execute_sub_ = nh_.subscribe("/find_place", 1, &PlacePose::ExecuteCallback, this); // execute once when the message comes
  table_point_cloud_.reset(new PointCloud);    // holds table point cloud
  num_catrgory_1 = 0; //the number of objects on the first layer
  num_catrgory_2 = 0; // the number of objects on the second layer
  num_known = 0; //the number of unknown object
  ROS_INFO_STREAM("Initial Done");
  return true;
}


void PlacePose::ObjectPoseCallback(const project_msgs::LabeledCentroid::ConstPtr& msg)//messsage type)
{
  object_label = msg->label; //
}

void PlacePose::TablePoseCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  pcl::fromROSMsg(*msg, *table_point_cloud_);
  PointT minPt, maxPt;
  pcl::getMinMax3D(*table_point_cloud_, minPt, maxPt);

  //calculate the centroid and size of table
  min_x = minPt.x;
  len_y = (maxPt.y - minPt.y)/4;
  mid_y = (maxPt.y + minPt.y)/2;
  max_z = maxPt.z;
  collision_shelf.x.clear();
  collision_shelf.y.clear();
  collision_shelf.z.clear();
  collision_shelf.length.clear();
  collision_shelf.width.clear();
  collision_shelf.height.clear();
  collision_shelf.x.push_back((maxPt.x + minPt.x)/2);
  collision_shelf.y.push_back((maxPt.y + minPt.y)/2);
  collision_shelf.z.push_back((maxPt.z + minPt.z)/2);
  collision_shelf.length.push_back((maxPt.x - minPt.x));
  collision_shelf.width.push_back((maxPt.y - minPt.y));
  collision_shelf.height.push_back((maxPt.z - minPt.z));

}

void PlacePose::ExecuteCallback(const std_msgs::Bool::ConstPtr& msg)
{
  //the place pose for catgory1
  if(object_label == 1)
  {
    place.pose.position.x = min_x + 0.05;
    place.pose.position.y = mid_y + pow(-1,num_catrgory_1+1) * len_y * ceil(num_catrgory_1/2) ;
    place.pose.position.z = max_z+ 0.1;
    num_catrgory_1 += 1;
    ROS_INFO_STREAM("num_cat_1:"<<num_catrgory_1);
  }
  //the place pose for catgory2
  else if(object_label == 2)
  {
    place.pose.position.x = min_x + 0.05;
    place.pose.position.y = mid_y +pow(-1,num_catrgory_2+1) * len_y * ceil(num_catrgory_2/2);
    place.pose.position.z = max_z+ 0.1;
    num_catrgory_2 += 1;
    ROS_INFO_STREAM("num_cat_2:"<<num_catrgory_2);
  }
  //the place pose for unknown object
  else
  {
    place.pose.position.x = 0.25;
    place.pose.position.y = 0.1;
    place.pose.position.z = 0.5;
  }
  place_pose_pub_.publish(place);
  shelf_box_pub_.publish(collision_shelf);
}