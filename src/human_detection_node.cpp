#include "PowerPassFilter.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "human_detection_node");

  ros::NodeHandle nh;
  double frequency = 250.0;


  // Parameters
  //inputs
  std::string topic_external_force;

  //outputs
  std::string topic_human_force;


  if (!nh.getParam("topic_external_force", topic_external_force))   {
    ROS_ERROR("Couldn't retrieve the topic name for the external force. ");
    return -1;
  }

  if (!nh.getParam("topic_human_force", topic_human_force))   {
    ROS_ERROR("Couldn't retrieve the topic name for the human force. ");
    return -1;
  }


  PowerPassFilter human_detector(nh,
                                 frequency,
                                 topic_external_force,
                                 topic_human_force);

  human_detector.Run();


  return 0;
}