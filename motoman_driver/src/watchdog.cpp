#include "ros/ros.h"
#include "motoman_msgs/Effort.h"
//#include "motoman_driver/io_relay.h"

void watchDog_CB(const motoman_msgs::Effort::ConstPtr& msg)
{
  float seuil = 60.0; //N
  if(msg->ForceTotaleTCP > seuil){
    ROS_WARN("La force est superieur au seuil (%f N): [%f]", seuil ,msg->ForceTotaleTCP);
  }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "watchdog");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("joint_effort", 1000, watchDog_CB);

  ros::spin();

  return 0;
}
