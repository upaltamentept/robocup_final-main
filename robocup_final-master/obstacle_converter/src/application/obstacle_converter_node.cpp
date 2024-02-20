#include <obstacle_converter/obstacle_converter.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle_converter_node");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ObstacleConverter oc;

  oc.init();

  ros::Rate loop_rate(10);
  loop_rate.sleep();

  while (ros::ok())
  { 
    oc.update();
    loop_rate.sleep();
  }

  ros::shutdown;
  return 0;
}