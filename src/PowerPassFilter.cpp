#include "PowerPassFilter.h"

PowerPassFilter::PowerPassFilter(
    ros::NodeHandle &n,
    double frequency,
    std::string topic_input_wrench,
    std::string topic_tank_state,
    std::string topic_admittance_ratio,
    std::vector<double> M_a,
    std::vector<double> D_a
)
	:
	nh_(n),
	loop_rate_(frequency),
	real_loop_rate_(1 / frequency),
	M_a_(M_a.data()),
	D_a_(D_a.data())
{


	sub_input_wrench_ = nh_.subscribe(topic_input_wrench, 1000,
	                                  &PowerPassFilter::UpdateInputWrench, this,
	                                  ros::TransportHints().reliable().tcpNoDelay());

	pub_tank_state_ = nh_.advertise<std_msgs::Float32>(topic_tank_state, 1);

	pub_admittance_ratio_ = nh_.advertise<std_msgs::Float32>(topic_admittance_ratio, 1);


	dyn_rec_f_ = boost::bind(&PowerPassFilter::DynRecCallback, this, _1, _2);
	dyn_rec_srv_.setCallback(dyn_rec_f_);


	wrench_input_.setZero();
	simulated_velocity_.setZero();

	input_power_ = 0;
	output_power_ = 0;
	dissipate_power_ = 0;
	tank_energy_ = 0;


	// initialize reconf parameters, will be overwritten by dyn_reconfigure
	tank_size_ = 4;
	energy_trigger_ = 2;
	dissipation_rate_ = 2.5;

	acc_linear_max_ = 4;
	acc_angular_max_ = 1;
	vel_linear_max_ = 0.5;
	vel_angular_max_ = 1;


	for (int row = 0; row < 6; row++) {
		for (int col = 0; col < 6; col++) {

			if (col == row) {
				if (M_a_(col, row) <= 0 || D_a_(col, row) <= 0) {
					ROS_ERROR("Diagonal elements of mass and damping matrix should be positive");
					M_a_(col, row) = 100;
					M_a_(col, row) = 100;
				}
			}

			if (col != row) {
				if (M_a_(col, row) != 0 || D_a_(col, row) != 0) {
					ROS_WARN("Mass and damping matrix should be diagonal");
					M_a_(col, row) = 0;
					M_a_(col, row) = 0;
				}
			}
		}
	}


}



void PowerPassFilter::Run() {

	while (nh_.ok()) {

		SimulateVelocity();

		UpdateEnergyTank();



		ROS_INFO_STREAM_THROTTLE(1, "Running at             : " << real_loop_rate_.toSec());
		ROS_INFO_STREAM_THROTTLE(1, "Simulated inpout power : " << input_power_);
		ROS_INFO_STREAM_THROTTLE(1, "Tanks energy           : " << tank_energy_);
		ROS_INFO_STREAM_THROTTLE(1, "output power           : " << output_power_);

		ROS_INFO_STREAM_THROTTLE(1, "input Wrench           : " <<
		                         wrench_input_(0) << "\t" <<
		                         wrench_input_(1) << "\t" <<
		                         wrench_input_(2) << "\t" <<
		                         wrench_input_(3) << "\t" <<
		                         wrench_input_(4) << "\t" <<
		                         wrench_input_(5) );


		ROS_INFO_STREAM_THROTTLE(1, "-------------------------------------------");


		ros::spinOnce();
		loop_rate_.sleep();
		// Integrate for velocity based interface
		real_loop_rate_ = loop_rate_.expectedCycleTime();
	}
}


void PowerPassFilter::SimulateVelocity() {

	Vector6d simulated_acceleration;

	simulated_acceleration = M_a_.inverse() * (- D_a_ * simulated_velocity_ + wrench_input_);


	// limiting the accelaration for better stability and safety
	double acc_linear_norm = (simulated_acceleration.segment(0, 3)).norm();

	if (acc_linear_norm > acc_linear_max_) {
		ROS_WARN_STREAM_THROTTLE(1, "high simulated linear acceleration"
		                         << " norm: " << acc_linear_norm);
		simulated_acceleration.segment(0, 3) *= (acc_linear_max_ / acc_linear_norm);
	}

	double acc_angular_norm = (simulated_acceleration.segment(3, 3)).norm();

	if (acc_angular_norm > acc_angular_max_) {
		ROS_WARN_STREAM_THROTTLE(1, "high simulated angular acceleration"
		                         << " norm: " << acc_angular_norm);
		simulated_acceleration.segment(3, 3) *= (acc_angular_max_ / acc_angular_norm);
	}


	simulated_velocity_ += simulated_acceleration * real_loop_rate_.toSec();


	// limiting the velocities for better stability and safety
	double vel_linear_norm = (simulated_velocity_.segment(0, 3)).norm();

	if (vel_linear_norm > vel_linear_max_) {
		ROS_WARN_STREAM_THROTTLE(1, "high simulated linear velocity"
		                         << " norm: " << vel_linear_norm);
		simulated_velocity_.segment(0, 3) *= (vel_linear_max_ / vel_linear_norm);
	}

	double vel_angular_norm = (simulated_velocity_.segment(3, 3)).norm();

	if (vel_angular_norm > vel_angular_max_) {
		ROS_WARN_STREAM_THROTTLE(1, "high simulated angular velocity"
		                         << " norm: " << vel_angular_norm);
		simulated_velocity_.segment(3, 3) *= (vel_angular_max_ / vel_angular_norm);
	}


	input_power_ = simulated_velocity_.dot(wrench_input_);

}


void PowerPassFilter::UpdateEnergyTank() {

	// tank_energy_ += (input_power_  - output_power_ - dissipation_rate_ ) * real_loop_rate_.toSec();
	tank_energy_ += (input_power_  - output_power_ - dissipate_power_ ) * real_loop_rate_.toSec();

	// stay positive
	tank_energy_ = (tank_energy_ < 0) ? 0 : tank_energy_;

	std_msgs::Float32 msg;
	msg.data = tank_energy_;
	pub_tank_state_.publish(msg);

	double alpha;

	if(tank_energy_ > 0.99 * tank_size_){
		ROS_WARN_STREAM_THROTTLE(1,"Tank energy (" << tank_energy_ << ") higher than maximum (" << tank_size_ << ")" );
		tank_energy_ = 0.99 * tank_size_;
	}

	if (tank_energy_ > energy_trigger_) {
		alpha = (tank_energy_ - energy_trigger_ ) / (tank_size_ - energy_trigger_);
		alpha = (alpha > 1) ? 1 : alpha;
		output_power_ = alpha * input_power_;
		dissipate_power_ = (1 - alpha) * dissipation_rate_;
	}
	else {
		alpha = 0;
		output_power_ = 0;
		dissipate_power_ = dissipation_rate_;
	}

	std_msgs::Float32 msg_h;
	msg_h.data = alpha;
	pub_admittance_ratio_.publish(msg_h);

}


void PowerPassFilter::UpdateInputWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
	Vector6d raw_input;

	wrench_input_ << msg->wrench.force.x, msg->wrench.force.y,
	              msg->wrench.force.z, msg->wrench.torque.x,
	              msg->wrench.torque.y, msg->wrench.torque.z;

}





void PowerPassFilter::DynRecCallback(ds_admittance_control::PowerPassFilterConfig &config, uint32_t level) {

	ROS_INFO("Reconfigure request. Updatig the parameters ...");


	tank_size_ = config.tank_size;
	energy_trigger_ = config.trigger;

	if (energy_trigger_ > tank_size_) {
		ROS_WARN_STREAM("Trigger should be lower than size, setting back to " << tank_size_);
		energy_trigger_ = tank_size_;
	}

	dissipation_rate_ = config.dissipation_rate;

	acc_linear_max_ = config.acc_linear_max;
	acc_angular_max_ = config.acc_angular_max;
	vel_linear_max_ = config.vel_linear_max;
	vel_angular_max_ = config.vel_angular_max;

}
