#include "PowerPassFilter.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "human_detection_node");

  ros::NodeHandle nh;
  double frequency = 250.0;


  // Parameters
  //inputs
  std::string topic_external_wrench;

  //outputs
  std::string topic_human_wrench;


  if (!nh.getParam("topic_external_wrench", topic_external_wrench))   {
    ROS_ERROR("Couldn't retrieve the topic name for the external wrench. ");
    return -1;
  }

  if (!nh.getParam("topic_human_wrench", topic_human_wrench))   {
    ROS_ERROR("Couldn't retrieve the topic name for the human wrench. ");
    return -1;
  }


  PowerPassFilter human_detector(nh,
                                 frequency,
                                 topic_external_wrench,
                                 topic_human_wrench);

  human_detector.Run();


  return 0;
}