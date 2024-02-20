#ifndef OBSTACLE_CONVERTER
#define OBSTACLE_CONVERTER

#include <project_msgs/CollisionGeometry.h>
#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_msgs/Bool.h>


class ObstacleConverter
{
  private:
    //
    ros::NodeHandle nh_;

    ros::Subscriber obstacle_geometry_sub_;
    ros::Subscriber pick_done_sub_;

    ros::Publisher torso_pub_;

    moveit::planning_interface::PlanningSceneInterface PSI;
  public:
    //
    void init();
    void update();

  private:
    void obstacleCallback(const project_msgs::CollisionGeometry::ConstPtr& msg);
    void pickCallback(const std_msgs::Bool::ConstPtr& msg);

  private:
   void addCollisionObjects();

  private:
    int num_collision_objects_;
    std::vector<geometry_msgs::Vector3> centroids_, primitives_;

  private:
    std::string ref_frame_;
    bool command_;

    std::vector<moveit_msgs::CollisionObject> collision_objects_;

    bool ready_for_place_;

};

#endif
