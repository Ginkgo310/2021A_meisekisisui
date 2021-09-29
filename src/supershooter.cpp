#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <iostream>
#include <unistd.h>

int move_phase = 1;
bool shoot_admin = true;

std_msgs::Int16 shoot_ok;

void addmission_ok(const std_msgs::Int16& yumiya_phase){
	move_phase = yumiya_phase.data;
}

int main(int argc,char** argv){
	ros::init(argc, argv, "supershooter");
	ros::NodeHandle nh;
	ros::Rate loop_rate(1000);

	ros::Publisher yumi_pub  = nh.advertise<std_msgs::Int16>("shooter",10);
	ros::Subscriber get_addmission = nh.subscribe("addmitter",10,addmission_ok);
	while(ros::ok()){
		if(shoot_admin == true && move_phase == 3){
			shoot_ok.data = 3;
			yumi_pub.publish(shoot_ok);
		}else if(move_phase == 1){
			shoot_ok.data = 1;
			yumi_pub.publish(shoot_ok);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}


}

