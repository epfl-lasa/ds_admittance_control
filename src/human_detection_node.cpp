#include "PowerPassFilter.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "human_detection_node");

  ros::NodeHandle nh;
  double frequency = 250.0;


  // Parameters
  std::string topic_external_wrench;
  std::string topic_tank_state;
  std::string topic_admittance_ratio;

  std::vector<double> M_a;
  std::vector<double> D_a;


  if (!nh.getParam("topic_external_wrench", topic_external_wrench))   {
    ROS_ERROR("Couldn't retrieve the topic name for the external wrench. ");
    return -1;
  }

  if (!nh.getParam("topic_tank_state", topic_tank_state))   {
    ROS_ERROR("Couldn't retrieve the topic name for the state of the tank. ");
    return -1;
  }

  if (!nh.getParam("topic_admittance_ratio", topic_admittance_ratio))   {
    ROS_ERROR("Couldn't retrieve the topic name for the admittance ratio. ");
    return -1;
  }

  if (!nh.getParam("mass", M_a)) {
    ROS_ERROR("Couldn't retrieve the admittance mass.");
    return -1;
  }

  if (!nh.getParam("damping", D_a)) {
    ROS_ERROR("Couldn't retrieve the admittance damping.");
    return -1;
  }



  PowerPassFilter human_detector(nh,
                                 frequency,
                                 topic_external_wrench,
                                 topic_tank_state,
                                 topic_admittance_ratio,
                                 M_a,
                                 D_a);

  human_detector.Run();


  return 0;
}