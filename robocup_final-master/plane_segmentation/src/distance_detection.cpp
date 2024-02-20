#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <std_msgs/Float32.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
ros::Publisher pub;
//subscribe the table pointcloud and calculate the distance to table 
void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);
    std_msgs::Float32 distance;
    distance.data = minPt.x;
    pub.publish(distance);
    // Example: Print the number of points in the cloud
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "distance_detection");
    ros::NodeHandle nh;

    // Create a subscriber to the "pointcloud_topic" topic
    ros::Subscriber sub = nh.subscribe("/table_plane_pointcloud", 10, pointCloudCallback);
    pub = nh.advertise<std_msgs::Float32>("distance_to_table", 10);
    // Spin the node and process callback functions
    ros::spin();

    return 0;
}