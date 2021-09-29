#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
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

float status[5];

int move_phase = 0;
int cmd_num = 0;
int addmitter = 0;
long int counter = 0;

std_msgs::Int16MultiArray max_data;

std_msgs::Int16MultiArray under_carryer;

std_msgs::Int16 yumiya_phase;

std_msgs::Float32MultiArray error_data;

std_msgs::Int16 neck_pos;

void getcheck(const std_msgs::Int16& buffer){
	move_phase = buffer.data;
}

void getStatus(const std_msgs::Float32MultiArray& under_status){
	for(int i = 0; i <5; ++i){
		status[i] = under_status.data[i];
	}
}

void underdataSet(int x_target,int y_target,int theta_target,int id,int cul_mode){
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

int main(int argc, char** argv)
{
	ros::init(argc, argv, "main_com");
	ros::NodeHandle nh;
	ros::Rate loop_rate(1000);

	ros::Publisher max_pub         = nh.advertise<std_msgs::Int16MultiArray>("maxim", 10);
	ros::Publisher under_pub       = nh.advertise<std_msgs::Int16MultiArray>("asimawari",10);
	ros::Publisher error_pub       = nh.advertise<std_msgs::Float32MultiArray>("errors",10);
	ros::Publisher yumi_addmission = nh.advertise<std_msgs::Int16>("addmitter",10);
	ros::Publisher neck_pub        = nh.advertise<std_msgs::Int16>("necker",10);

	ros::Subscriber run_check      = nh.subscribe("check",10,getcheck);
	ros::Subscriber status_sub     = nh.subscribe("statuses",10,getStatus);

	max_data.data.resize(4);
	under_carryer.data.resize(6);
	error_data.data.resize(3);

	while(ros::ok()){
		switch(move_phase){
			case 0:
				std::cin >> move_phase;
				break;
				/*case 1:
				  yumiya_phase.data = 1;
				  if(counter < 5000){
				  counter++;
				  }else{
				  counter = 0;
				  move_phase = 2;
				  }
				  break;
				  case 2:
				  yumiya_phase.data = 3;
				  break;*/
			case 1:
				neck_pos.data = 1;
				neck_pub.publish(neck_pos);
				underdataSet(400,20,0,2,0);
				under_carryer.data[Go_addmission] = 1;
				under_pub.publish(under_carryer);
				break;
			case 2:
				underdataSet(400,400,0,3,0);
				under_carryer.data[Go_addmission] = 1;
				under_pub.publish(under_carryer);
				break;
			case 3:
				underdataSet(400,400,90,4,0);
				under_carryer.data[Go_addmission] = 1;
				under_pub.publish(under_carryer);
				break;
			case 4:
				//yumiya_phase.data = 1;
				//yumi_addmission.publish(yumiya_phase);
				underdataSet(20,400,0,5,0);
				under_carryer.data[Go_addmission] = 1;
				under_pub.publish(under_carryer);
				break;
			case 5:
				underdataSet(20,400,90,6,0);
				under_carryer.data[Go_addmission] = 1;
				under_pub.publish(under_carryer);
				break;	
			case 6:
				underdataSet(20,250,0,7,2);
				under_carryer.data[Go_addmission] = 1;
				under_pub.publish(under_carryer);
				break;
			case 7:
				underdataSet(20,250,-180,8,3);
				under_carryer.data[Go_addmission] = 1;
				under_pub.publish(under_carryer);
				break;
			case 8:
				underdataSet(20,250,0,9,0);
				under_carryer.data[Go_addmission] = 1;
				under_pub.publish(under_carryer);

				//yumiya_phase.data = 3;
				//yumi_addmission.publish(yumiya_phase);		
				break;
			case 9:
				if(counter < 5000){
                                  counter++;
                                }else{
                                  counter = 0;
                                  move_phase = 10;
                                }

				break;
			case 10:
				underdataSet(20,250,180,11,3);
				under_carryer.data[Go_addmission] = 1;
				under_pub.publish(under_carryer);
				break;
			case 11:
				underdataSet(20,20,0,12,2);
				under_carryer.data[Go_addmission] = 1;
				under_pub.publish(under_carryer);
				break;
			case 12:
				break;


		}		
		//std::cout << " phase:" << move_phase << "  x_position:" << status[X_POS] << "  y_position:" << status[Y_POS] << "  theta:" << status[THETA] << std::endl;
		ROS_INFO("phase:%d x_position:%5.2f y_position:%5.2f theta:%5.5f vx:%5.2f vy:%5.2f ",move_phase,status[X_POS],status[Y_POS],status[THETA],status[NOW_VX],status[NOW_VY]);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
