#include "PowerPassFilter.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "human_detection_node");

  ros::NodeHandle nh;
  double frequency = 250.0;


  // Parameters
  std::string topic_external_wrench;
  std::string topic_external_wrench_filtered;
  std::string topic_human_wrench;
  std::string topic_desired_velocity;
  std::string topic_tank_state;

  std::vector<double> M_a;
  std::vector<double> D_a;
  std::vector<double> ft_rotation;


  if (!nh.getParam("topic_external_wrench", topic_external_wrench))   {
    ROS_ERROR("Couldn't retrieve the topic name for the external wrench. ");
    return -1;
  }

  if (!nh.getParam("topic_external_wrench_filtered", topic_external_wrench_filtered))   {
    ROS_ERROR("Couldn't retrieve the topic name for the external wrench. ");
    return -1;
  }

  if (!nh.getParam("topic_human_wrench", topic_human_wrench))   {
    ROS_ERROR("Couldn't retrieve the topic name for the human wrench. ");
    return -1;
  }

  if (!nh.getParam("topic_desired_velocity", topic_desired_velocity))   {
    ROS_ERROR("Couldn't retrieve the topic name for the desired velocity. ");
    return -1;
  }

  if (!nh.getParam("topic_tank_state", topic_tank_state))   {
    ROS_ERROR("Couldn't retrieve the topic name for the state of the tank. ");
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

  if (!nh.getParam("ft_sensor_rotation", ft_rotation)) {
    ROS_ERROR("Couldn't retrieve the rotation matrix for ft-sensor.");
    return -1;
  }


  PowerPassFilter human_detector(nh,
                                 frequency,
                                 topic_external_wrench,
                                 topic_external_wrench_filtered,
                                 topic_human_wrench,
                                 topic_desired_velocity,
                                 topic_tank_state,
                                 M_a,
                                 D_a,
                                 ft_rotation);

  human_detector.Run();


  return 0;
}