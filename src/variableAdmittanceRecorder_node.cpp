#include <iostream>
#include <fstream>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <ros/ros.h>

#include "cartesian_state_msgs/PoseTwist.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"


using namespace std;



class varAdmRecorder {

private:
	ros::NodeHandle nh_;
	ros::Rate loop_rate_;

	std::string recPath_;


	//real variables from the robot
	ofstream file_ee_pose_;
	ofstream file_ee_twist_real_;
	ofstream file_ee_twist_desired_;
	ofstream file_external_force_;
	ofstream file_tank_state_;
	ofstream file_human_factor_;


	ros::Subscriber sub_ee_state_; // it recieves both pose and twists
	ros::Subscriber sub_ee_twist_desired_;
	ros::Subscriber sub_external_force_;
	ros::Subscriber sub_tank_state_;
	ros::Subscriber sub_human_factor_;




public:
	varAdmRecorder(ros::NodeHandle &n, double frequency)
		: nh_(n), loop_rate_(frequency) {

		ROS_INFO_STREAM("The recorder node is created at: " << nh_.getNamespace() << " with freq: " << frequency << "Hz");

	}

	void Initialize() {

		// Creating a recording directory
		std::string recPath_ = "/home/mahdi/catkin_ws/Recordings/";
		mkdir(recPath_.c_str(), 0777);

		// Creating a subdirectory for a specific subject
		recPath_ += "VariableAdmittance/";
		mkdir(recPath_.c_str(), 0777);

		// Creating a subdirectory for a specific interaction based on time
		time_t rawtime;
		tm* timeinfo;
		char buffer [80];
		time(&rawtime);
		timeinfo = localtime(&rawtime);
		strftime(buffer, 80, "%Y-%m-%d-%H-%M-%S", timeinfo);
		recPath_ += string(buffer);
		mkdir(recPath_.c_str(), 0777);

		cout << "Recording to :" << recPath_.c_str() << endl;

		string temp_path;

		temp_path = recPath_ + "/ee_pose.txt";
		file_ee_pose_.open(temp_path);
		file_ee_pose_  << "Time \t x \t y \t z \t qx \t qy \t qz \t qw  \n";

		temp_path = recPath_ + "/ee_twist_real.txt";
		file_ee_twist_real_.open(temp_path);
		file_ee_twist_real_  << "Time \t x \t y \t z \t qx \t qy \t qz \t qw  \n";

		temp_path = recPath_ + "/ee_twist_desired.txt";
		file_ee_twist_desired_.open(temp_path);
		file_ee_twist_desired_  << "Time \t x \t y \t z \t qx \t qy \t qz \t qw  \n";

		temp_path = recPath_ + "/external_force.txt";
		file_external_force_.open(temp_path);
		file_external_force_  << "Time \t Fx \t Fy \t Fz \t tx \t ty \t tz \n";

		temp_path = recPath_ + "/tank_state.txt";
		file_tank_state_.open(temp_path);
		file_tank_state_  << "Time \t E \n";

		temp_path = recPath_ + "/human_factor.txt";
		file_human_factor_.open(temp_path);
		file_human_factor_  << "Time \t h \n";


		sub_ee_state_ = nh_.subscribe("/ur5/ur5_cartesian_velocity_controller_sim/ee_state" ,
		                              1000, &varAdmRecorder::state_arm_callback, this);


		sub_ee_twist_desired_ = nh_.subscribe("/ridgeback_velocity_controller/cmd_vel",
		                                      1000 , &varAdmRecorder::desird_twist_callback, this);

		sub_external_force_ = nh_.subscribe("/admittance_control/transformed/external_wrench",
		                                    1000 , &varAdmRecorder::external_force_callback, this);

		sub_tank_state_ = nh_.subscribe("/admittance/tank_state",
		                                    1000 , &varAdmRecorder::tank_state_callback, this);

		sub_human_factor_ = nh_.subscribe("/admittance_control/admittance_ratio",
		                                  1000 , &varAdmRecorder::human_factor_callback, this);


	}


	void Run() {

		while (nh_.ok()) {

			ros::spinOnce();
			loop_rate_.sleep();
		}

	}

	void state_arm_callback(
	    const cartesian_state_msgs::PoseTwistConstPtr msg) {

		file_ee_pose_  << ros::Time::now()	<< "\t"
		               << msg->pose.position.x 	<< "\t"
		               << msg->pose.position.y 	<< "\t"
		               << msg->pose.position.z 	<< "\t"
		               << msg->pose.orientation.x << "\t"
		               << msg->pose.orientation.y << "\t"
		               << msg->pose.orientation.z << "\t"
		               << msg->pose.orientation.w << "\n";

		file_ee_twist_real_  << ros::Time::now()	<< "\t"
		                     << msg->twist.linear.x << "\t"
		                     << msg->twist.linear.y << "\t"
		                     << msg->twist.linear.z << "\t"
		                     << msg->twist.angular.x << "\t"
		                     << msg->twist.angular.y << "\t"
		                     << msg->twist.angular.z << "\n";

	}



	void desird_twist_callback(const geometry_msgs::TwistPtr msg) {

		file_ee_twist_desired_ <<  ros::Time::now()	<< "\t"
		                       << msg->linear.x << "\t"
		                       << msg->linear.y << "\t"
		                       << msg->linear.z << "\t"
		                       << msg->angular.x << "\t"
		                       << msg->angular.y << "\t"
		                       << msg->angular.z << "\n";
	}

	void external_force_callback(const geometry_msgs::WrenchStampedConstPtr msg) {

		file_external_force_ <<  ros::Time::now()	<< "\t"
		                     << msg->wrench.force.x << "\t"
		                     << msg->wrench.force.y << "\t"
		                     << msg->wrench.force.z << "\t"
		                     << msg->wrench.torque.x << "\t"
		                     << msg->wrench.torque.y << "\t"
		                     << msg->wrench.torque.z << "\n";
	}

	void tank_state_callback(const std_msgs::Float32Ptr msg) {


		file_tank_state_ <<  ros::Time::now()	<< "\t"
		                 << msg->data << "\n";
	}

	void human_factor_callback(const std_msgs::Float32Ptr msg) {

		file_human_factor_ <<  ros::Time::now()	<< "\t"
		                   << msg->data << "\n";
	}


};






int main(int argc, char **argv)
{
	//Initiate ROS
	ros::init(argc, argv, "varAdmittance_recorder");



	ros::NodeHandle nh;
	double frequency = 100.0;
	varAdmRecorder my_recorder(nh, frequency);

	my_recorder.Initialize();

	my_recorder.Run();

	return 0;
}