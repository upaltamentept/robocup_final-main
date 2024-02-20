#ifndef OBJECT_LABELING_H
#define OBJECT_LABELING_H

#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <project_msgs/LabeledCentroid.h>
#include <project_msgs/CollisionGeometry.h>
#include <cmath>

/*********************************************************************
* darknet
********************************************************************/
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

/*********************************************************************
* STD
********************************************************************/
#include <iostream>
#include <fstream>
#include <pthread.h>
#include <map>
#include <std_msgs/Bool.h>

/*********************************************************************
* PCL and Opencv
********************************************************************/
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/impl/transforms.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class PlacePose
{
public:
  // pcl pointcloud types (only color RGB)
  typedef pcl::PointXYZRGB PointT;
  typedef pcl::PointCloud<PointT> PointCloud;
  typedef PointCloud::Ptr CloudPtr;

public:
  bool initalize(ros::NodeHandle& nh_);
  //bool update(const ros::Time& time);
private:
  void ObjectPoseCallback(const project_msgs::LabeledCentroid::ConstPtr& msg);
  void TablePoseCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
  void ExecuteCallback(const std_msgs::Bool::ConstPtr& msg);
  ros::Subscriber object_labeled_sub_;   //!< sub detections form detector
  ros::Subscriber table_cloud_sub_;   //!< sub point cloud of table
  ros::Subscriber execute_sub_;  //execute the code once
  ros::Publisher place_pose_pub_; //!< publisher for place pose
  ros::Publisher shelf_box_pub_; //!< publisher for labeled pointcloud

  // outputs

  CloudPtr table_point_cloud_;                             //!< table point cloud
  int object_label;
  int num_catrgory_1;
  int num_catrgory_2;
  int num_known;
  float min_x;
  float len_y;
  float max_z;
  float mid_y;
  geometry_msgs::PoseStamped place;
  project_msgs::CollisionGeometry collision_shelf;

};

#endif
