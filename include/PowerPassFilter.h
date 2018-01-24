#ifndef __POWER_PASS_FILTER_H__
#define __POWER_PASS_FILTER_H__


#include "geometry_msgs/Wrench.h"

// #include "geometry_msgs/Pose2D.h"
// #include "std_msgs/Int32.h"


#include "ros/ros.h"
#include <vector>

// #include <dynamic_reconfigure/server.h>
// #include <adaptive_polishing/polishing_paramsConfig.h>


#include "eigen3/Eigen/Core"
using namespace Eigen;

typedef Matrix<double, 6, 1> Vector6d;


class PowerPassFilter {

private:

	// ROS
	ros::NodeHandle nh_;
	ros::Rate loop_rate_;


	ros::Subscriber sub_input_wrench_;
	ros::Publisher  pub_output_wrench_;


	Vector6d wrench_input_;
	Vector6d wrench_output_;


public:
	PowerPassFilter(
	    ros::NodeHandle &n,
	    double frequency,
	    std::string topic_input_wrench,
	    std::string topic_output_wrench
	);

	void Run();


private:

	void UpdateInputWrench(const geometry_msgs::Wrench::ConstPtr& msg);
	void PublishOutputWrench();




};

#endif //__POWER_PASS_FILTER_H__