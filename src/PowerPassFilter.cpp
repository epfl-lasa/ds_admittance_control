#include "PowerPassFilter.h"

PowerPassFilter::PowerPassFilter(
    ros::NodeHandle &n,
    double frequency,
    std::string topic_input_wrench,
    std::string topic_output_wrench
)
	:
	nh_(n),
	loop_rate_(frequency)
{


	sub_input_wrench_ = nh_.subscribe(topic_input_wrench, 1000,
	                                    &PowerPassFilter::UpdateInputWrench, this,
	                                    ros::TransportHints().reliable().tcpNoDelay());

	pub_output_wrench_ = nh_.advertise<geometry_msgs::Wrench>(topic_output_wrench, 1);



	wrench_input_.setZero();
	wrench_output_.setZero();
}







void PowerPassFilter::Run() {

	while (nh_.ok()) {

		// ComputeDesiredVelocity();


		wrench_output_ = wrench_input_;
		PublishOutputWrench();

		ros::spinOnce();

		loop_rate_.sleep();
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



void PowerPassFilter::PublishOutputWrench(){

	geometry_msgs::Wrench msg;

	msg.force.x  = wrench_output_(0);
	msg.force.y  = wrench_output_(1);
	msg.force.z  = wrench_output_(2);
	msg.torque.x = wrench_output_(3);
	msg.torque.y = wrench_output_(4);
	msg.torque.z = wrench_output_(5);

	pub_output_wrench_.publish(msg);


}