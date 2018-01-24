#ifndef __POWER_PASS_FILTER_H__
#define __POWER_PASS_FILTER_H__


#include "geometry_msgs/Wrench.h"

// #include "geometry_msgs/Pose2D.h"
// #include "std_msgs/Int32.h"


#include "ros/ros.h"
#include <vector>

// #include <dynamic_reconfigure/server.h>
// #include <adaptive_polishing/polishing_paramsConfig.h>

class PowerPassFilter {

private:

	// ROS
	ros::NodeHandle nh_;
	ros::Rate loop_rate_;


	ros::Subscriber sub_external_force_;
	ros::Publisher  pub_human_force_;



public:
	PowerPassFilter(
	    ros::NodeHandle &n,
	    double frequency,
	    std::string topic_external_force,
	    std::string topic_human_force
	);

	void Run();


private:

	void UpdateExternalForce(const geometry_msgs::Wrench::ConstPtr& msg);





};

#endif //__POWER_PASS_FILTER_H__