#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <meisekisisui/realsenseInfo.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <iostream>
#include <unistd.h>

const float x_error  = 1.0;
const float y_error  = 1.0;
const float x_target = 383.0;
const float y_target = 298.0;

int move_phase = 0;
bool shoot_admin[3] = {false,false,false};
bool shotornot[3] = {false,false,false};
int  returner = 0;

std_msgs::Int16 shoot_ok;

std_msgs::Int16 shoot_buf;

float x_dist[3] = {0,0,0};
float y_dist[3] = {0,0,0};
bool   check[3] = {0,0,0};
int   can_shoot = -1;
int count = -1;

void target_recognise(const std_msgs::Int16MultiArray& shoot_recogniser){
	can_shoot = shoot_recogniser.data[0];
	returner  = shoot_recogniser.data[1];
}

void addmission_ok(const std_msgs::Int16& yumiya_phase){
	move_phase = yumiya_phase.data;
}

void getMarker0(const meisekisisui::realsenseInfo& marker0){	
	x_dist[0] = marker0.x_distance;
	y_dist[0] = marker0.y_distance;
	check[0]  = marker0.captured;
}

void getMarker1(const meisekisisui::realsenseInfo& marker1){	
	x_dist[1] = marker1.x_distance;
	y_dist[1] = marker1.y_distance;
	check[1]  = marker1.captured;
}

void getMarker2(const meisekisisui::realsenseInfo& marker2){	
	x_dist[2] = marker2.x_distance;
	y_dist[2] = marker2.y_distance;
	check[2]  = marker2.captured;
}

int main(int argc,char** argv){
	ros::init(argc, argv, "supershooter");
	ros::NodeHandle nh;
	ros::Rate loop_rate(1000);

	ros::Publisher  yumi_pub       = nh.advertise<std_msgs::Int16>("shooter",10);
	ros::Publisher  buf_pub        = nh.advertise<std_msgs::Int16>("bufbuf" ,10);
	ros::Subscriber target_recog   = nh.subscribe("tar_recog",10,target_recognise);
	ros::Subscriber get_addmission = nh.subscribe("addmitter",10,addmission_ok);
	ros::Subscriber get_marker0    = nh.subscribe("marker0"  ,10,   getMarker0);
	ros::Subscriber get_marker1    = nh.subscribe("marker1"  ,10,   getMarker1);
	ros::Subscriber get_marker2    = nh.subscribe("marker2"  ,10,   getMarker2);

	while(ros::ok()){
		for(int i = 0; i < 3; ++i){
			shoot_admin[i] = false;
		}

		for(int i = 0; i < 3; ++i){
			if((x_dist[i] - x_target) < x_error && check[i] == 1){
				if((y_dist[i] - y_target) < y_error){
					shoot_admin[i]	= true;
				}					
			}
		}
		
		switch(move_phase){
			case 1:
				shoot_ok.data = 1;
				break;
			case 3:
				if(shoot_admin[can_shoot] == true){
					shoot_ok.data  = 3;
					shoot_buf.data = returner;
					buf_pub.publish(shoot_buf);
				}
				break;
		}
		yumi_pub.publish(shoot_ok);

		ros::spinOnce();
		loop_rate.sleep();
	}


}

