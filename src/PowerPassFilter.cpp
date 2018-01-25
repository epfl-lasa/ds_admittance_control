#include "PowerPassFilter.h"

PowerPassFilter::PowerPassFilter(
    ros::NodeHandle &n,
    double frequency,
    std::string topic_input_wrench,
    std::string topic_output_wrench
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



	dyn_rec_f_ = boost::bind(&PowerPassFilter::DynRecCallback, this, _1, _2);
	dyn_rec_srv_.setCallback(dyn_rec_f_);


	wrench_input_.setZero();
	wrench_output_.setZero();

	simulated_velocity_.setZero();

	input_power_ = 0;

	output_power_ = 0;
	tank_energy_ = 0;
	drain_power_ = 10.0 / frequency;

	M_f_.setIdentity();
	D_f_.setIdentity();

}


void PowerPassFilter::DynRecCallback(ds_admittance_control::PowerPassFilterConfig &config, uint32_t level){
	
}




void PowerPassFilter::Run() {

	while (nh_.ok()) {

		SimulateVelocity();

		UpdateEnergyTank();

		ComputeFilteredWrench();

		PublishOutputWrench();

		ros::spinOnce();



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



		loop_rate_.sleep();
		// Integrate for velocity based interface
		real_loop_rate_ = loop_rate_.expectedCycleTime();
	}
}


void PowerPassFilter::SimulateVelocity() {

	Vector6d simulated_acceleration;

	simulated_acceleration = M_f_.inverse() * (- D_f_ * simulated_velocity_ + wrench_input_);


	// limiting the accelaration for better stability and safety
	// double acc_linear_norm = (simulated_acceleration.segment(0, 3)).norm();

	// if (acc_linear_norm > acc_linear_max) {
	// 	ROS_WARN_STREAM_THROTTLE(1, "high simulated linear acceleration"
	// 	                         << " norm: " << p_acc_norm);
	// 	simulated_acceleration.segment(0, 3) *= (acc_linear_max / acc_linear_norm);
	// }



	simulated_velocity_ += simulated_acceleration * real_loop_rate_.toSec();


	input_power_ = simulated_velocity_.dot(wrench_input_);

}

void PowerPassFilter::UpdateEnergyTank() {

	tank_energy_ += (input_power_  - output_power_ - drain_power_ ) * real_loop_rate_.toSec();

	// stay positive
	tank_energy_ = (tank_energy_ < 0) ? 0 : tank_energy_;




}

void PowerPassFilter::ComputeFilteredWrench() {

	if (tank_energy_ > 10) {
		double alpha = (tank_energy_ - 10 ) / 1;
		alpha = (alpha > 1) ? 1 : alpha;
		output_power_ = alpha * input_power_;
		wrench_output_ = (output_power_ / input_power_) * wrench_input_;
	}
	else {
		output_power_ = 0;
		wrench_output_.setZero();
	}


}


void PowerPassFilter::UpdateInputWrench(const geometry_msgs::Wrench::ConstPtr& msg) {
	wrench_input_ << msg->force.x, msg->force.y,
	              msg->force.z, msg->torque.x,
	              msg->torque.y, msg->torque.z;

	// // Dead zone for the FT sensor
	// if (wrench_ft_frame.topRows(3).norm() < force_dead_zone_thres_) {
	//   wrench_ft_frame.topRows(3).setZero();
	// }
	// if (wrench_ft_frame.bottomRows(3).norm() < torque_dead_zone_thres_) {
	//   wrench_ft_frame.bottomRows(3).setZero();
	// }

	// // Filter and update
	// u_e_ <<  (1 - wrench_filter_factor_) * u_e_ +
	//      wrench_filter_factor_ * rotation_ft_base * wrench_ft_frame;



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