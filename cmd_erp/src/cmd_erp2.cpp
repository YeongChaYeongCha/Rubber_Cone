#include "ros/ros.h"
#include <cmd_erp/erp_control_values.h>
#include <std_msgs/Float32.h>

int start=0;
float inclination_angle_;
float temp_inclination_angle_=0.0;

void TargetAngleCallback(const std_msgs::Float32& inclination_angle){
	inclination_angle_=inclination_angle.data;
	if(start==0){
		temp_inclination_angle_=inclination_angle_;
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "cmd_ERP");
	ros::NodeHandle nh;
	
	cmd_erp::erp_control_values drive_msg;
	
	//ros::Subscriber sub_target_angle=nh.subscribe("/target_angle", 10, &TargetAngleCallback);
	ros::Subscriber sub_target_angle=nh.subscribe("/inclination_angle", 10, &TargetAngleCallback);
	ros::Publisher cmd_vel_pub=nh.advertise<cmd_erp::erp_control_values>("/control_value", 1);
	
	ros::Rate loop_rate(5);
	
	while(ros::ok()){
		start++;
		if(abs(temp_inclination_angle_-inclination_angle_)>30.0){
			inclination_angle_=temp_inclination_angle_;
		}
		else{
			temp_inclination_angle_=inclination_angle_;
		}
		ROS_INFO("Recieved Inclination : %f", inclination_angle_);
		
		if(start<=6){
			drive_msg.velocity=7;
			drive_msg.steering=0.0;
		}
		else{
			drive_msg.velocity=5;
			drive_msg.steering=inclination_angle_;
		}
		
		cmd_vel_pub.publish(drive_msg);
		
		loop_rate.sleep();
		ros::spinOnce();
	}
	
	return 0;
}





