#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
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

const int X_INFO = 0;
const int Y_INFO = 1;

const int NOW  = 0;
const int PREV = 1;

int shoot_buffer = 0;

float status[5];

int move_phase = 0;
int cmd_num = 0;
int addmitter = 0;
long int counter = 0;

float x_tar_sense = 308;
float y_tar_sense = 420;
float tar_error_x;
float tar_error_y;
int depth_id = 0;

float x_dist[3];
float y_dist[3];
float depth[3];
int check[3];

int read_key = 0;

float lidar_info[2];

bool arart = false;
int arart_status[2] = {1,1};

float saveman_a[2];
float saveman_b[2];

bool catcher = false;
float saver[2] = {0,0};

int ppp[4] = {0,0,0,0};
float adder = 0;
bool check_the_target = false;

float shoot_line = 0;

int shoot_phase = 0;

std_msgs::Int16MultiArray max_data;

std_msgs::Float32MultiArray under_carryer;

std_msgs::Int16 yumiya_phase;

std_msgs::Int16MultiArray shoot_recogniser;

std_msgs::Float32MultiArray error_data;

std_msgs::Int16 neck_pos;

std_msgs::Int16 resetter;

std_msgs::Int16 spec_Add;

void getMarker(const meisekisisui::realsenseInfo& marker){
	x_dist[0] = marker.x_distance[0];
	y_dist[0] = marker.y_distance[0];
	for(int i = 0; i < 3; ++i){
		depth[i]  = marker.depth[i];
		check[i]  = marker.captured[i];
		if(depth[i] < 421 && depth[i] > 419){
			depth[i] = 420;
		}
		if(x_dist[i] > 388 && x_dist[i] < 394){
			x_dist[i] = 391;
		}
	}
}

void arartCheck(int n){
	if(arart == 1){
		under_carryer.data[Go_addmission] = 0;
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

void errordataSet(float x_error, float y_error, float theta_error, float k){
	error_data.data[X_POS] =     x_error; 
	error_data.data[Y_POS] =     y_error;	
	error_data.data[THETA] = theta_error;
	error_data.data[3]     = k;
}

void getArart(const std_msgs::Int16& pushed_msg){
	arart = pushed_msg.data;
	arart_status[NOW] = arart;
	if(arart_status[NOW] != arart_status[PREV]){
		if(arart_status[NOW] == 1){
			ROS_INFO("robot has been rocked by remote emargency");
		}else{
			ROS_INFO("robot is now avairable");
		}
	}
}

void getLidar(const std_msgs::Int32MultiArray& lidar_lidar){
	for(int i = 0; i < 2; ++i){
		lidar_info[i] = lidar_lidar.data[i] / 10;
		if(lidar_info[i] > 500){
			lidar_info[i] = 500;
		}
	}
}

void getshootcheck(const std_msgs::Int16& shooter){
	shoot_phase = shooter.data;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "main_com");
	ros::NodeHandle nh;
	ros::Rate loop_rate(1000);

	ros::Publisher max_pub         = nh.advertise<std_msgs::Int16MultiArray>("maxim", 10);
	ros::Publisher under_pub       = nh.advertise<std_msgs::Float32MultiArray>("asimawari",10);
	ros::Publisher error_pub       = nh.advertise<std_msgs::Float32MultiArray>("errors",10);
	ros::Publisher yumi_addmission = nh.advertise<std_msgs::Int16>("shooter",10);
	ros::Publisher target_recog    = nh.advertise<std_msgs::Int16MultiArray>("tar_recog",10);
	ros::Publisher neck_pub        = nh.advertise<std_msgs::Int16>("necker",10);
	ros::Publisher reset_pub       = nh.advertise<std_msgs::Int16>("reset",10);
	ros::Publisher spadd_pub       = nh.advertise<std_msgs::Int16>("special_add",10);

	ros::Subscriber run_check      = nh.subscribe("check"   ,10,getcheck);
	ros::Subscriber status_sub     = nh.subscribe("statuses",10,getStatus);
	ros::Subscriber shoot_sub      = nh.subscribe("bufbuf"  ,10,getshootbuf);
	ros::Subscriber arart_sub      = nh.subscribe("pushed"  ,10,getArart);
	ros::Subscriber get_marker     = nh.subscribe("marker" ,10,getMarker);
	ros::Subscriber get_lidar      = nh.subscribe("lidar_info",100,getLidar);
        ros::Subscriber shoot_check    = nh.subscribe("shooter_checker",10,getshootcheck);

	max_data.data.resize(4);
	under_carryer.data.resize(6);
	error_data.data.resize(3);
	shoot_recogniser.data.resize(2);

	spec_Add.data = 0;
	spadd_pub.publish(spec_Add);
	resetter.data = 0;
	reset_pub.publish(resetter);

	while(ros::ok()){
		switch(move_phase){
			case 0:
				under_carryer.data[Go_addmission] = 0;
				under_pub.publish(under_carryer);
				std::cin >> read_key;
				if(read_key >= 101){
					resetter.data = read_key;
					reset_pub.publish(resetter);
				}else{
					if(arart == 1){
						ROS_INFO("remote emargency switch online");
					}else{
						resetter.data = 10;
						reset_pub.publish(resetter);
						move_phase = read_key;
					}
				}
				break;
			case 7:
				yumiya_phase.data = 3;
				//yumi_addmission.publish(yumiya_phase);
				if(counter < 1000){
					counter++;
				}else if(counter < 3000){
					counter++;
					yumi_addmission.publish(yumiya_phase);
				}else{
					counter = 0;
					move_phase = 24;
				}

				break;
			case 20:
				neck_pos.data = 1;
				neck_pub.publish(neck_pos);
				shoot_recogniser.data[0] = 4;
				shoot_recogniser.data[1] = 1;
				target_recog.publish(shoot_recogniser);
				//sleep(2);
				underdataSet(0,0,-90,21,0);	
				arartCheck(1);
				under_pub.publish(under_carryer);
				break;
			case 21:
				underdataSet(400,30,0,22,0);
				arartCheck(1);
				under_pub.publish(under_carryer);
				break;
			case 22:
				underdataSet(400,30,90,23,0);
				arartCheck(1);
				under_pub.publish(under_carryer);
				break;
			case 23:
				maximdataSet(100,500,500,150);
				max_pub.publish(max_data);
				yumiya_phase.data = 1;
				yumi_addmission.publish(yumiya_phase);
				tar_error_y = (x_tar_sense - lidar_info[Y_INFO]);
				underdataSet(400,status[Y_POS] - tar_error_y,0,7,0);	
				errordataSet(1.0,1.0,0.005,40);
				arartCheck(1);
				under_pub.publish(under_carryer);
				error_pub.publish(error_data);
				break;
			case 24:
				if(catcher == false){
					catcher = true;
					saver[0] = status[X_POS];
					saver[1] = status[Y_POS];
				}
				underdataSet(saver[0],saver[1],180,25,3);
				arartCheck(1);
				under_pub.publish(under_carryer);
				maximdataSet(200,1000,1000,150);
				max_pub.publish(max_data);
				errordataSet(5.0,5.0,0.01,1);
				error_pub.publish(error_data);
				break;
			case 25:
				underdataSet(400,30,0,26,3);
				arartCheck(1);
				under_pub.publish(under_carryer);
				catcher = false;
				break;
			case 26:
				underdataSet(400,30,-180,27,0);
				arartCheck(1);
				under_pub.publish(under_carryer);
				break;
			case 27:
				resetter.data = 2;
				reset_pub.publish(resetter);
				move_phase = 0;
				break;
			case 30:              
				yumiya_phase.data = 1;
				yumi_addmission.publish(yumiya_phase);
				maximdataSet(100,500,500,150);
				max_pub.publish(max_data);
				//underdataSet(0,420,0,0,0);
				//arartCheck(1);
				//under_pub.publish(under_carryer);
				move_phase = 31;
				break;
			case 31:
				tar_error_x = depth[0] - y_tar_sense;
				if(check[0] == 1){
					underdataSet(status[X_POS] + tar_error_x,420,0,0,0);
					arartCheck(1);
					under_pub.publish(under_carryer);
				}else{
					underdataSet(0,420,0,0,0);
					arartCheck(1);
					under_pub.publish(under_carryer);
				}
				shoot_recogniser.data[0] = 0;
				shoot_recogniser.data[1] = 2;
				target_recog.publish(shoot_recogniser);
				yumiya_phase.data = 3;
				yumi_addmission.publish(yumiya_phase);
				if(shoot_buffer == 2){
					if(counter < 500){
						++counter;
						yumiya_phase.data = 1;
						yumi_addmission.publish(yumiya_phase);
					}else{
						counter = 0;
						move_phase = 32;
					}
				}
				ROS_INFO("x:%lf depth:%lf check:%d",x_dist[0],depth[0],check[0]);
				break;
			case 32:
				tar_error_x = depth[1] - y_tar_sense;
				if(check[1] == 1){
					underdataSet(status[X_POS] + tar_error_x,420,0,0,0);
					arartCheck(1);
					under_pub.publish(under_carryer);
				}else{
					underdataSet(0,420,0,0,0);
					arartCheck(1);
					under_pub.publish(under_carryer);
				}
				shoot_recogniser.data[0] = 1;
				shoot_recogniser.data[1] = 3;
				target_recog.publish(shoot_recogniser);
				yumiya_phase.data = 3;
				yumi_addmission.publish(yumiya_phase);
				if(shoot_buffer == 3){
					if(counter < 100){
						++counter;
						//yumiya_phase.data = 1;
						//yumi_addmission.publish(yumiya_phase);
					}else{
						counter = 0;
						move_phase = 33;
					}
				}
				ROS_INFO("x:%lf depth:%lf check:%d",x_dist[1],depth[1],check[1]);
				break;
			case 33:
				under_carryer.data[Go_addmission] = 0;
				under_pub.publish(under_carryer);
				break;
			case 40:
				neck_pos.data = 1;
				neck_pub.publish(neck_pos);
				maximdataSet(75,375,375,150);
				max_pub.publish(max_data);
				//sleep(2);
				underdataSet(0,420,0,43,0);
				arartCheck(1);
				under_pub.publish(under_carryer);
				yumiya_phase.data = 1;
				yumi_addmission.publish(yumiya_phase);
				errordataSet(5.0,5.0,0.01,1);
				error_pub.publish(error_data);
				catcher = false;
				move_phase = 41;
				break;
			case 41:
				if(lidar_info[Y_INFO] <= 358 && lidar_info[Y_INFO] >= 352 && catcher == false){
					catcher = true;
					counter = 0;
					saveman_a[0] = lidar_info[Y_INFO];
					saveman_a[1] = status[THETA];
				}
				if(catcher == true){
					if(counter < 500){
						yumiya_phase.data = 3;
						yumi_addmission.publish(yumiya_phase);
						++counter;
					}else if(counter < 700){
						yumiya_phase.data = 1;
						yumi_addmission.publish(yumiya_phase);
						++counter;
					}else{
						yumiya_phase.data = 1;
						yumi_addmission.publish(yumiya_phase);
						counter = 0;
						catcher = false;
						move_phase = 42;
					}
				}

				if(lidar_info[Y_INFO] <= 300){
					counter = 0;
					catcher = false;
					move_phase = 42;

				}
				break;
			case 42:
				if(lidar_info[Y_INFO] >= 148 && lidar_info[Y_INFO] <= 154 && catcher == false){
					catcher = true;
					counter = 0;
					saveman_b[0] = lidar_info[Y_INFO];
					saveman_b[1] = status[THETA];
				}
				if(catcher == true){
					yumiya_phase.data = 3;
					yumi_addmission.publish(yumiya_phase);
					if(counter < 1000){
						++counter;
					}else{
						counter = 0;
						catcher = false;
						move_phase = 43;
					}
				}else{
					yumiya_phase.data = 1;
					yumi_addmission.publish(yumiya_phase);
				} 
				break;
			case 43:
				yumiya_phase.data = 3;
				yumi_addmission.publish(yumiya_phase);
				sleep(3);
				if(catcher == false){
					catcher = true;
					saver[0] = status[X_POS];
					saver[1] = status[Y_POS];
				}
				underdataSet(saver[0],saver[1],180,44,3);
				arartCheck(1);
				under_pub.publish(under_carryer);
				maximdataSet(200,1000,1000,150);
				max_pub.publish(max_data);
				break;
			case 44:
				underdataSet(saver[0],20,0,45,3);
				arartCheck(1);
				under_pub.publish(under_carryer);
				counter = 0;
				catcher = false;
				break;
			case 45:
				underdataSet(saver[0],20,-90,46,2);
				arartCheck(1);
				under_pub.publish(under_carryer);
				break;
			case 46:
				tar_error_y = (40 - lidar_info[X_INFO]);
				underdataSet(saver[0]-370,status[Y_POS] + tar_error_y,0,47,2);
				arartCheck(1);
				under_pub.publish(under_carryer);
				break;
			case 47:
				ROS_INFO("first lidar:%lf theta:%lf second lidar:%lf theta:%lf",saveman_a[0],saveman_a[1],saveman_b[0],saveman_b[1]);
				move_phase = 0;
				break;
			case 50:
				counter = 0;
				errordataSet(1.0,1.0,0.01,40);
				error_pub.publish(error_data);
				catcher = false;
				move_phase = 51;
				break;
			case 51:
				if(counter < 1000){
					errordataSet(5.0,5.0,0.01,1);
					error_pub.publish(error_data);
					maximdataSet(240,1200,1200,150);
					max_pub.publish(max_data);
					if(catcher == false){
						underdataSet(0,0,-45,52,0);
						arartCheck(1);
						under_pub.publish(under_carryer);
						catcher = true;
					}
					++counter;
				}else{
					yumiya_phase.data = 1;
					yumi_addmission.publish(yumiya_phase);
					catcher = false;
					counter = 0;
				}
				break;
			case 52:
				underdataSet(420,420,0,0,0);
				arartCheck(1);
				under_pub.publish(under_carryer);
				move_phase = 53;
				break;
			case 53:
				if(status[X_POS] >= 200 && status[X_POS] <= 204){
					if(status[Y_POS] >= 200 && status[Y_POS] <= 204){
						if(status[THETA] >= -0.79 && status[THETA] <= -0.78)
						{	
							catcher = true;
						}
					}
				}

				if(status[X_POS] >= 210){
					if(status[Y_POS] >= 210){
						catcher = true;

					}
				}

				if(catcher == true){
					yumiya_phase.data = 3;
					yumi_addmission.publish(yumiya_phase);
					saveman_a[0] = status[X_POS];
					saveman_a[1] = status[Y_POS];
					saveman_b[0] = status[THETA];	
					move_phase = 54;
				}else{
					yumiya_phase.data = 1;
					yumi_addmission.publish(yumiya_phase);
				}
				break;
			case 54:
				ROS_INFO("SHOOOOT! at x:%lf y:%lf theta:%lf",saveman_a[0],saveman_a[1],saveman_b[0]);
				break;
			case 60:
				neck_pos.data = 1;
				neck_pub.publish(neck_pos);
				maximdataSet(75,375,375,150);
				max_pub.publish(max_data);
				yumiya_phase.data = 1;
				yumi_addmission.publish(yumiya_phase);
				underdataSet(0,420,1,63,0);
				arartCheck(1);
				under_pub.publish(under_carryer);
				errordataSet(5.0,5.0,0.01,1);
				error_pub.publish(error_data);
				catcher = false;
				check_the_target = false;
				move_phase = 61;
				break;
			case 61:
				if(lidar_info[X_INFO] <= 380 && check_the_target == false){
					adder = lidar_info[Y_INFO];
					check_the_target = true;
				}
				if(lidar_info[Y_INFO] <= adder - 0 && lidar_info[Y_INFO] >= adder - 4 && catcher == false && check_the_target == true){
					catcher = true;
					counter = 0;
					saveman_a[0] = lidar_info[Y_INFO];
					saveman_a[1] = status[THETA];
				}
				if(catcher == true){
					if(counter < 500){
						yumiya_phase.data = 3;
						yumi_addmission.publish(yumiya_phase);
						++counter;
					}else if(counter < 700){
						yumiya_phase.data = 1;
						yumi_addmission.publish(yumiya_phase);
						++counter;
					}else{
						yumiya_phase.data = 1;
						yumi_addmission.publish(yumiya_phase);
						counter = 0;
						check_the_target = false;
						catcher = false;
						move_phase = 62;
					}
				}
				break;
			case 62:
				if(lidar_info[X_INFO] <= 380 && check_the_target == false && lidar_info[Y_INFO] < 250){
					adder = lidar_info[Y_INFO];
					check_the_target = true;
				}

				if(lidar_info[Y_INFO] <= adder - 0 && lidar_info[Y_INFO] >= adder - 4 && catcher == false && check_the_target == true){
					catcher = true;
					counter = 0;
					saveman_b[0] = lidar_info[Y_INFO];
					saveman_b[1] = status[THETA];
				}
				if(catcher == true){
					yumiya_phase.data = 3;
					yumi_addmission.publish(yumiya_phase);
					if(counter < 1000){
						++counter;
					}else{
						counter = 0;
						check_the_target = false;
						catcher = false;
						move_phase = 63;
					}
				}else{
					yumiya_phase.data = 1;
					yumi_addmission.publish(yumiya_phase);
				} 
				break;
			case 63:
				yumiya_phase.data = 3;
				yumi_addmission.publish(yumiya_phase);
				sleep(3);
				if(catcher == false){
					catcher = true;
					saver[0] = status[X_POS];
					saver[1] = status[Y_POS];
				}
				underdataSet(saver[0],saver[1],180,64,3);
				arartCheck(1);
				under_pub.publish(under_carryer);
				maximdataSet(200,1000,1000,150);
				max_pub.publish(max_data);
				break;
			case 64:
				underdataSet(saver[0],20,0,65,3);
				arartCheck(1);
				under_pub.publish(under_carryer);
				counter = 0;
				catcher = false;
				break;
			case 65:
				underdataSet(saver[0],20,-90,66,2);
				arartCheck(1);
				under_pub.publish(under_carryer);
				break;
			case 66:
				tar_error_y = (40 - lidar_info[X_INFO]);
				underdataSet(saver[0]-370,status[Y_POS] + tar_error_y,0,67,2);
				arartCheck(1);
				under_pub.publish(under_carryer);
				break;
			case 67:
				ROS_INFO("first lidar:%lf theta:%lf second lidar:%lf theta:%lf",saveman_a[0],saveman_a[1],saveman_b[0],saveman_b[1]);
				move_phase = 0;
				break;
			case 70:
				counter = 0;
				errordataSet(1.0,1.0,0.01,40);
				error_pub.publish(error_data);
				catcher = false;
				move_phase = 71;
				break;
			case 71:
				if(counter < 1000){
					errordataSet(5.0,5.0,0.01,1);
					error_pub.publish(error_data);
					maximdataSet(240,1200,1200,150);
					max_pub.publish(max_data);
					if(catcher == false){
						underdataSet(0,0,-45,72,0);
						arartCheck(1);
						under_pub.publish(under_carryer);
						catcher = true;
					}
					++counter;
				}else{
					yumiya_phase.data = 1;
					yumi_addmission.publish(yumiya_phase);
					catcher = false;
					counter = 0;
					spec_Add.data = 1;
					spadd_pub.publish(spec_Add);
				}
				break;
			case 72:
				arartCheck(0);
                                spec_Add.data = 1;
                                spadd_pub.publish(spec_Add);
				if(shoot_phase != 2){
					yumiya_phase.data = 1;
                                        yumi_addmission.publish(yumiya_phase);
				}else{
					move_phase = 73;
				}
				break;
			case 73:
				underdataSet(420,420,0,0,0);
				arartCheck(1);
				under_pub.publish(under_carryer);
				move_phase = 74;
				break;
			case 74:
				if(status[X_POS] >= 220){
                                        if(status[Y_POS] >= 220){
						yumiya_phase.data = 3;
                                        	yumi_addmission.publish(yumiya_phase);
						//spec_Add.data = 0;
                                        	//spadd_pub.publish(spec_Add);
                                        }
                                }
				break;		
		}
		ROS_INFO("phase:%d x_position:%5.2f y_position:%5.2f theta:%5.5f shoot_phase:%d lidar_x:%5.2f lidar_y:%5.2f vvvv%d counter%ld adder:%5.2f tof:%d",move_phase,status[X_POS],status[Y_POS],status[THETA],shoot_phase,lidar_info[X_INFO],lidar_info[Y_INFO],catcher,counter,adder,check_the_target);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
