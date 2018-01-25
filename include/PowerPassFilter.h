#ifndef __POWER_PASS_FILTER_H__
#define __POWER_PASS_FILTER_H__


#include "geometry_msgs/Wrench.h"

// #include "geometry_msgs/Pose2D.h"
// #include "std_msgs/Int32.h"


#include "ros/ros.h"
#include <vector>

#include <dynamic_reconfigure/server.h>
#include <ds_admittance_control/PowerPassFilterConfig.h>


#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"
using namespace Eigen;

typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;

class PowerPassFilter {

private:

	// ROS
	ros::NodeHandle nh_;
	ros::Rate loop_rate_;
	ros::Duration real_loop_rate_;


	ros::Subscriber sub_input_wrench_;
	ros::Publisher  pub_output_wrench_;


	Vector6d wrench_input_;
	Vector6d wrench_output_;

	Vector6d simulated_velocity_;

	double input_power_;
	double output_power_;
	double tank_energy_;



	// params
	Matrix6d M_f_, D_f_;


	//dynamic reconfig settig
	dynamic_reconfigure::Server<ds_admittance_control::PowerPassFilterConfig> dyn_rec_srv_;
	dynamic_reconfigure::Server<ds_admittance_control::PowerPassFilterConfig>::CallbackType dyn_rec_f_;

	// reconf parameters 
	double tank_size_;
	double energy_trigger_;
	double dissipation_rate_;
	double force_dead_zone_;
	double torque_dead_zone_;
	double filter_rate_;





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
	void SimulateVelocity();
	void UpdateEnergyTank();
	void ComputeFilteredWrench();
	void DynRecCallback(ds_admittance_control::PowerPassFilterConfig &config, uint32_t level);




};

#endif //__POWER_PASS_FILTER_H__