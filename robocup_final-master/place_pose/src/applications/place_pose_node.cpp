#include <place_pose/place_pose.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "place_pose");
  ros::NodeHandle nh;

  PlacePose placepose;

  // Init
  if(!placepose.initalize(nh))
  {
    return -1;
  }
  ros::spin();
  // Run
  //ros::Rate rate(30);
  //while(ros::ok())
  //{
    
    //placepose.update(ros::Time::now());
    // ros::spin();
    //rate.sleep();
  //}

  return 0;
}
