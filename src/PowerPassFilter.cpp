#include "PowerPassFilter.h"

PowerPassFilter::PowerPassFilter(
    ros::NodeHandle &n,
    double frequency,
    std::string topic_input_wrench,
    std::string topic_output_wrench,
    std::string topic_desired_velocity
)
	:
	nh_(n),
	loop_rate_(frequency),
	real_loop_rate_(1 / frequency)
{


	sub_input_wrench_ = nh_.subscribe(topic_input_wrench, 1000,
	                                  &PowerPassFilter::UpdateInputWrench, this,
	                                  ros::TransportHints().reliable().tcpNoDelay());

	pub_output_wrench_ = nh_.advertise<geometry_msgs::Wrench>(topic_output_wrench, 1);

	pub_desired_velocity_ = nh_.advertise<geometry_msgs::Twist>(topic_desired_velocity, 1);


	dyn_rec_f_ = boost::bind(&PowerPassFilter::DynRecCallback, this, _1, _2);
	dyn_rec_srv_.setCallback(dyn_rec_f_);


	wrench_input_.setZero();
	wrench_output_.setZero();

	simulated_velocity_.setZero();
	desired_velocity_.setZero();

	input_power_ = 0;

	output_power_ = 0;
	tank_energy_ = 0;

	M_f_.setIdentity();
	D_f_.setIdentity();


	// initialize reconf parameters, will be overwritten by dyn_reconfigure
	tank_size_ = 10;
	energy_trigger_ = 9;
	dissipation_rate_ = 0.1;
	force_dead_zone_ = 0.1;
	torque_dead_zone_ = 0.5;
	filter_rate_ = 0.1;

	acc_linear_max_ = 4;
	acc_angular_max_ = 1;
	vel_linear_max_ = 0.5;
	vel_angular_max_ = 1;

}



void PowerPassFilter::Run() {

	while (nh_.ok()) {

		SimulateVelocity();

		UpdateEnergyTank();

		ComputeFilteredWrench();

		PublishOutputWrench();

		ComputeAdmittance();

		PublishDesiredVelocity();




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

		ROS_INFO_STREAM_THROTTLE(1, "output Wrench          : " <<
		                         wrench_output_(0) << "\t" <<
		                         wrench_output_(1) << "\t" <<
		                         wrench_output_(2) << "\t" <<
		                         wrench_output_(3) << "\t" <<
		                         wrench_output_(4) << "\t" <<
		                         wrench_output_(5) );

		ROS_INFO_STREAM_THROTTLE(1, "-------------------------------------------");


		ros::spinOnce();
		loop_rate_.sleep();
		// Integrate for velocity based interface
		real_loop_rate_ = loop_rate_.expectedCycleTime();
	}
}


void PowerPassFilter::SimulateVelocity() {

	Vector6d simulated_acceleration;

	simulated_acceleration = M_f_.inverse() * (- D_f_ * simulated_velocity_ + wrench_input_);


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

	tank_energy_ += (input_power_  - output_power_ - dissipation_rate_ ) * real_loop_rate_.toSec();

	// stay positive
	tank_energy_ = (tank_energy_ < 0) ? 0 : tank_energy_;
}


void PowerPassFilter::ComputeFilteredWrench() {

	if (tank_energy_ > energy_trigger_) {
		double alpha = (tank_energy_ - energy_trigger_ ) / (tank_size_ - energy_trigger_);
		alpha = (alpha > 1) ? 1 : alpha;
		output_power_ = alpha * input_power_;
		if (input_power_ != 0) {
			wrench_output_ = (output_power_ / input_power_) * wrench_input_;
		}
		else {
			wrench_output_.setZero();
		}
	}
	else {
		output_power_ = 0;
		wrench_output_.setZero();
	}

}


void PowerPassFilter::ComputeAdmittance() {
	Vector6d acceleration;

	acceleration = M_f_.inverse() * (- D_f_ * desired_velocity_ + wrench_output_);


	// limiting the accelaration for better stability and safety
	double acc_linear_norm = (acceleration.segment(0, 3)).norm();

	if (acc_linear_norm > acc_linear_max_) {
		ROS_WARN_STREAM_THROTTLE(1, "high simulated linear acceleration"
		                         << " norm: " << acc_linear_norm);
		acceleration.segment(0, 3) *= (acc_linear_max_ / acc_linear_norm);
	}

	double acc_angular_norm = (acceleration.segment(3, 3)).norm();

	if (acc_angular_norm > acc_angular_max_) {
		ROS_WARN_STREAM_THROTTLE(1, "high simulated angular acceleration"
		                         << " norm: " << acc_angular_norm);
		acceleration.segment(3, 3) *= (acc_angular_max_ / acc_angular_norm);
	}


	desired_velocity_ += acceleration * real_loop_rate_.toSec();


	// limiting the velocities for better stability and safety
	double vel_linear_norm = (desired_velocity_.segment(0, 3)).norm();

	if (vel_linear_norm > vel_linear_max_) {
		ROS_WARN_STREAM_THROTTLE(1, "high simulated linear velocity"
		                         << " norm: " << vel_linear_norm);
		desired_velocity_.segment(0, 3) *= (vel_linear_max_ / vel_linear_norm);
	}

	double vel_angular_norm = (desired_velocity_.segment(3, 3)).norm();

	if (vel_angular_norm > vel_angular_max_) {
		ROS_WARN_STREAM_THROTTLE(1, "high simulated angular velocity"
		                         << " norm: " << vel_angular_norm);
		desired_velocity_.segment(3, 3) *= (vel_angular_max_ / vel_angular_norm);
	}

	// maybe this power can be useful to regulate some behavior
	// final_input_power_ = desired_velocity_.dot(wrench_output_);	
	// final_input_power_ = real_velocity_.dot(wrench_output_);

}




void PowerPassFilter::UpdateInputWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
	Vector6d raw_input;

	raw_input << msg->wrench.force.x, msg->wrench.force.y,
	          msg->wrench.force.z, msg->wrench.torque.x,
	          msg->wrench.torque.y, msg->wrench.torque.z;

	// Dead zone for the FT sensor
	if (raw_input.topRows(3).norm() < force_dead_zone_) {
		raw_input.topRows(3).setZero();
	}
	if (raw_input.bottomRows(3).norm() < torque_dead_zone_) {
		raw_input.bottomRows(3).setZero();
	}

	// Filter and update
	wrench_input_ += (1 - filter_rate_) * (raw_input - wrench_input_);
}


void PowerPassFilter::PublishOutputWrench() {

	geometry_msgs::Wrench msg;

	msg.force.x  = wrench_output_(0);
	msg.force.y  = wrench_output_(1);
	msg.force.z  = wrench_output_(2);
	msg.torque.x = wrench_output_(3);
	msg.torque.y = wrench_output_(4);
	msg.torque.z = wrench_output_(5);

	pub_output_wrench_.publish(msg);

}

void PowerPassFilter::PublishDesiredVelocity(){

	geometry_msgs::Twist msg;
	msg.linear.x = desired_velocity_(0);
	msg.linear.y = desired_velocity_(1);
	msg.linear.z = desired_velocity_(2);
	msg.angular.x = desired_velocity_(3);
	msg.angular.y = desired_velocity_(4);
	msg.angular.z = desired_velocity_(5);

	pub_desired_velocity_.publish(msg);
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

	force_dead_zone_ = config.force_dead_zone;
	torque_dead_zone_ = config.torque_dead_zone;

	filter_rate_ = config.filter_rate;


	acc_linear_max_ = config.acc_linear_max;
	acc_angular_max_ = config.acc_angular_max;
	vel_linear_max_ = config.vel_linear_max;
	vel_angular_max_ = config.vel_angular_max;

}
