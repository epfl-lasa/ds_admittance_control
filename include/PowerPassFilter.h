#ifndef __POWER_PASS_FILTER_H__
#define __POWER_PASS_FILTER_H__


#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"

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
typedef Matrix<double, 3, 3> Matrix3d;

class PowerPassFilter {

private:

	// ROS
	ros::NodeHandle nh_;
	ros::Rate loop_rate_;
	ros::Duration real_loop_rate_;


	ros::Subscriber sub_input_wrench_;

	ros::Publisher  pub_input_wrench_filtered_;
	ros::Publisher  pub_output_wrench_;
	ros::Publisher  pub_desired_velocity_;
	ros::Publisher  pub_tank_state_;


	Vector6d wrench_input_;
	Vector6d wrench_output_;

	Vector6d simulated_velocity_;
	Vector6d desired_velocity_;

	double input_power_;
	double output_power_;
	double dissipate_power_;
	double tank_energy_;



	// params
	Matrix6d M_a_, D_a_;
	Matrix3d ft_rotation_;


	//dynamic reconfig settig
	dynamic_reconfigure::Server<ds_admittance_control::PowerPassFilterConfig> dyn_rec_srv_;
	dynamic_reconfigure::Server<ds_admittance_control::PowerPassFilterConfig>::CallbackType dyn_rec_f_;

	// reconf parameters
	double tank_size_;
	double energy_trigger_;
	double dissipation_rate_;
	double force_dead_zone_;
	double torque_dead_zone_;
	double force_filter_rate_;
	double vel_filter_rate_;

	double acc_linear_max_;
	double acc_angular_max_;
	double vel_linear_max_;
	double vel_angular_max_;





public:
	PowerPassFilter(
	    ros::NodeHandle &n,
	    double frequency,
	    std::string topic_input_wrench,
	    std::string topic_input_wrench_filtered,
	    std::string topic_output_wrench,
	    std::string topic_desired_velocity,
	    std::string topic_tank_state,
	    std::vector<double> M_a,
	    std::vector<double> D_a,
	    std::vector<double> ft_rotation
	);

	void Run();


private:

	void UpdateInputWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg);
	void PublishOutputWrench();
	void SimulateVelocity();
	void UpdateEnergyTank();
	void ComputeFilteredWrench();
	void DynRecCallback(ds_admittance_control::PowerPassFilterConfig &config, uint32_t level);
	void ComputeAdmittance();
	void PublishDesiredVelocity();



};

#endif //__POWER_PASS_FILTER_H__