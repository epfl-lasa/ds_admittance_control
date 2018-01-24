#include "PowerPassFilter.h"

PowerPassFilter::PowerPassFilter(
    ros::NodeHandle &n,
    double frequency,
    std::string topic_external_force,
    std::string topic_human_force
)
	:
	nh_(n),
	loop_rate_(frequency)
{


	sub_external_force_ = nh_.subscribe(topic_external_force, 1000,
	                                    &PowerPassFilter::UpdateExternalForce, this,
	                                    ros::TransportHints().reliable().tcpNoDelay());

	pub_human_force_ = nh_.advertise<geometry_msgs::Wrench>(topic_human_force, 1);


}


void PowerPassFilter::UpdateExternalForce(const geometry_msgs::Wrench::ConstPtr& msg) {
	// nothing for now
}




void PowerPassFilter::Run() {

	while (nh_.ok()) {

		// ComputeDesiredVelocity();

		// PublishDesiredVelocity();

		// PublishFuturePath();

		ros::spinOnce();

		loop_rate_.sleep();
	}
}