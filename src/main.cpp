#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <meisekisisui/realsenseInfo.h>
#include <iostream>
#include <unistd.h>

const int X_target      = 0;
const int Y_target      = 1;
const int Theta_target  = 2;
const int POS_id        = 3;
const int Go_addmission = 4;
const int Theta_culmode = 5;

const int Wheel_max     = 0;
const int Vx_max        = 1;
const int Vy_max        = 2;
const int Omega_max     = 3;

const int X_POS  = 0;
const int Y_POS  = 1;
const int THETA  = 2;
const int NOW_VX = 3;
const int NOW_VY = 4;

int shoot_buffer = 0;

float status[5];

int move_phase = 0;
int cmd_num = 0;
int addmitter = 0;
long int counter = 0;

float x_tar_sense = 416;
float tar_error;

float x_dist,y_dist;
int check;

bool arart = false;

bool catcher = false;

std_msgs::Int16MultiArray max_data;

std_msgs::Float32MultiArray under_carryer;

std_msgs::Int16 yumiya_phase;

std_msgs::Int16MultiArray shoot_recogniser;

std_msgs::Float32MultiArray error_data;

std_msgs::Int16 neck_pos;

void getMarker(const meisekisisui::realsenseInfo& marker0){
        x_dist = marker0.x_distance;
        y_dist = marker0.y_distance;
        check  = marker0.captured;
}

void arartCheck(int n){
	if(arart == 1){
		under_carryer.data[Go_addmission] = n;
	}else{
		under_carryer.data[Go_addmission] = n;
	}
}

void getcheck(const std_msgs::Int16& buffer){
	move_phase = buffer.data;
}

void getStatus(const std_msgs::Float32MultiArray& under_status){
	for(int i = 0; i <5; ++i){
		status[i] = under_status.data[i];
	}
}

void getshootbuf(const std_msgs::Int16& shoot_buf){
	shoot_buffer = shoot_buf.data;
}

void underdataSet(float x_target,float y_target,float theta_target,float id,float cul_mode){
	under_carryer.data[X_target]      = x_target;
	under_carryer.data[Y_target]      = y_target;
	under_carryer.data[Theta_target]  = theta_target;
	under_carryer.data[POS_id]        = id; 
	under_carryer.data[Theta_culmode] = cul_mode;
}

void maximdataSet(int wheel, int vx, int vy, int omega){
	max_data.data[Wheel_max] = wheel;
	max_data.data[Vx_max]    =    vx;
	max_data.data[Vy_max]    =    vy;	
	max_data.data[Omega_max] = omega;
}

void errordataSet(float x_error, float y_error, float theta_error){
	error_data.data[X_POS] =     x_error; 
	error_data.data[Y_POS] =     y_error;	
	error_data.data[THETA] = theta_error;
}

void getArart(const std_msgs::Int16& pushed_msg){
	arart = pushed_msg.data;
	//if(arart == 1){
	//	move_phase == 0;
	//}
	//ROS_INFO("%d",arart);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "main_com");
	ros::NodeHandle nh;
	ros::Rate loop_rate(1000);

	ros::Publisher max_pub         = nh.advertise<std_msgs::Int16MultiArray>("maxim", 10);
	ros::Publisher under_pub       = nh.advertise<std_msgs::Float32MultiArray>("asimawari",10);
	ros::Publisher error_pub       = nh.advertise<std_msgs::Float32MultiArray>("errors",10);
	ros::Publisher yumi_addmission = nh.advertise<std_msgs::Int16>("addmitter",10);
	ros::Publisher target_recog    = nh.advertise<std_msgs::Int16MultiArray>("tar_recog",10);
	ros::Publisher neck_pub        = nh.advertise<std_msgs::Int16>("necker",10);

	ros::Subscriber run_check      = nh.subscribe("check"   ,10,getcheck);
	ros::Subscriber status_sub     = nh.subscribe("statuses",10,getStatus);
	ros::Subscriber shoot_sub      = nh.subscribe("bufbuf"  ,10,getshootbuf);
	ros::Subscriber arart_sub      = nh.subscribe("pushed"  ,10,getArart);
	ros::Subscriber get_marker     = nh.subscribe("marker0" ,10,getMarker);


	max_data.data.resize(4);
	under_carryer.data.resize(6);
	error_data.data.resize(3);
	shoot_recogniser.data.resize(2);

	while(ros::ok()){
		switch(move_phase){
			case 0:
				under_carryer.data[Go_addmission] = 0;
				under_pub.publish(under_carryer);
				std::cin >> move_phase;
				break;
			case 1:
				neck_pos.data = 1;
				neck_pub.publish(neck_pos);
				underdataSet(300,40,0,2,0);
				arartCheck(1);
				under_pub.publish(under_carryer);
				break;
			case 2:
				underdataSet(300,400,0,3,0);
				arartCheck(1);
				under_pub.publish(under_carryer);
				break;
			case 3:
				underdataSet(300,400,90,4,0);
				arartCheck(1);
				under_pub.publish(under_carryer);
				break;
			case 4:
				yumiya_phase.data = 1;
				shoot_recogniser.data[0] = 0;
				shoot_recogniser.data[1] = 1;
				underdataSet(20,400,0,5,0);
				arartCheck(1);
				yumi_addmission.publish(yumiya_phase);
				target_recog.publish(shoot_recogniser);
				under_pub.publish(under_carryer);
				break;
			case 5:
				underdataSet(20,400,90,6,2);
				arartCheck(1);
				under_pub.publish(under_carryer);
				break;	
			case 6:
				if(check == 1){
					catcher == true;
				}
				tar_error = (x_tar_sense -  x_dist)*50/74.25;
				if(catcher == true){
					underdataSet(20,status[Y_POS] - tar_error,0,7,2);
					errordataSet(0.5,1.0,0.02);
				}else{
					underdataSet(20,200,0,7,2);
					errordataSet(2.0,2.0,0.03);
				}
				arartCheck(1);
				error_pub.publish(error_data);
				under_pub.publish(under_carryer);
				break;
			case 7:
				yumiya_phase.data = 3;
				if(counter < 5000){
					counter++;
				}else{
					counter = 0;
					move_phase = 8;
				}

				break;
			case 8:
				underdataSet(20,470,0,9,2);
				arartCheck(1);
				under_pub.publish(under_carryer);
				break;
			case 9:
				std::cin >> move_phase;
				break;
			case 10:
				yumiya_phase.data = 1;
				//yumi_pub.publish(yumiya_phase);
				underdataSet(20,20,0,14,2);
				arartCheck(1);
				under_pub.publish(under_carryer);
				//move_phase = 11;
				break;
			case 11:
				shoot_recogniser.data[0] = 0;
				shoot_recogniser.data[1] = 2;
				//target_recog.publish(shoot_recogniser);
				yumiya_phase.data = 3;
				//yumi_pub.publish(yumiya_phase);
				if(shoot_buffer == 2){
					if(counter < 100){
						++counter;
						yumiya_phase.data = 1;
					}else{
						counter = 0;
						move_phase = 12;
					}
				}
				break;
			case 12:
				shoot_recogniser.data[0] = 1;
				shoot_recogniser.data[1] = 3;
				//target_recog.publish(shoot_recogniser);
				yumiya_phase.data = 3;
				//yumi_pub.publish(yumiya_phase);
				if(shoot_buffer == 3){
					if(counter < 100){
						++counter;
						yumiya_phase.data = 1;
					}else{
						counter = 0;
						move_phase = 13;
					}
				}	
				break;
			case 13:
				shoot_recogniser.data[0] = 2;
				shoot_recogniser.data[1] = 4;
				//target_recog.publish(shoot_recogniser);
				yumiya_phase.data = 3;
				//yumi_pub.publish(yumiya_phase);
				//if(shoot_buffer == 4){
				//}
				break;
			case 14:
				std::cin >> move_phase;
				break;
			case 15:
				yumiya_phase.data = 1;
				underdataSet(20,120,0,16,2);
				arartCheck(1);
				under_pub.publish(under_carryer);
				break;
			case 16:
				underdataSet(20,120,-45,17,2);
				arartCheck(1);
				under_pub.publish(under_carryer);
				break;
			case 17:
				if(counter < 5000){
					counter++;
				}else{
					counter = 0;
					move_phase = 18;
				}

				break;
			case 18:
				yumiya_phase.data = 3;
				underdataSet(370,470,0,0,2);
				arartCheck(1);
				under_pub.publish(under_carryer);
				break;
		}		
		ROS_INFO("phase:%d x_position:%5.2f y_position:%5.2f theta:%5.5f vx:%5.2f vy:%5.2f ",move_phase,status[X_POS],status[Y_POS],status[THETA],status[NOW_VX],status[NOW_VY]);
		/*max_pub.publish(max_data);
		under_pub.publish(under_carryer);
		error_pub.publish(error_data);
		yumi_addmission.publish(yumiya_phase);
		target_recog.publish(shoot_recogniser);
		neck_pub.publish(neck_pos);*/
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
