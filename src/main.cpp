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

int shoot_buffer = 0;

float status[5];

int move_phase = 0;
int cmd_num = 0;
int addmitter = 0;
long int counter = 0;

float x_tar_sense = 300;
float y_tar_sense = 420;
float tar_error_x;
float tar_error_y;
int depth_id = 0;

float x_dist[3];
float y_dist[3];
float depth[3];
int check[3];

float lidar_info[2];

bool arart = false;

float saveman_a[2];
float saveman_b[2];

bool catcher = false;
float saver[2] = {0,0};

int ppp[4] = {0,0,0,0};

std_msgs::Int16MultiArray max_data;

std_msgs::Float32MultiArray under_carryer;

std_msgs::Int16 yumiya_phase;

std_msgs::Int16MultiArray shoot_recogniser;

std_msgs::Float32MultiArray error_data;

std_msgs::Int16 neck_pos;

std_msgs::Int16 resetter;

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

void errordataSet(float x_error, float y_error, float theta_error, float k){
	error_data.data[X_POS] =     x_error; 
	error_data.data[Y_POS] =     y_error;	
	error_data.data[THETA] = theta_error;
	error_data.data[3]     = k;
}

void getArart(const std_msgs::Int16& pushed_msg){
	arart = pushed_msg.data;
	//if(arart == 1){
	//	move_phase == 0;
	//}
	//ROS_INFO("%d",arart);
}

void getLidar(const std_msgs::Int32MultiArray& lidar_lidar){
	for(int i = 0; i < 2; ++i){
		lidar_info[i] = lidar_lidar.data[i] / 10;
		if(lidar_info[i] > 500){
			lidar_info[i] = 500;
		}
	}
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
	ros::Publisher reset_pub       = nh.advertise<std_msgs::Int16>("reset",10);

	ros::Subscriber run_check      = nh.subscribe("check"   ,10,getcheck);
	ros::Subscriber status_sub     = nh.subscribe("statuses",10,getStatus);
	ros::Subscriber shoot_sub      = nh.subscribe("bufbuf"  ,10,getshootbuf);
	ros::Subscriber arart_sub      = nh.subscribe("pushed"  ,10,getArart);
	ros::Subscriber get_marker     = nh.subscribe("marker" ,10,getMarker);
	ros::Subscriber get_lidar      = nh.subscribe("lidar_info",100,getLidar);

	max_data.data.resize(4);
	under_carryer.data.resize(6);
	error_data.data.resize(3);
	shoot_recogniser.data.resize(2);

	while(ros::ok()){
		switch(move_phase){
			case 0:
				resetter.data = 1;
				reset_pub.publish(resetter);
				under_carryer.data[Go_addmission] = 0;
				under_pub.publish(under_carryer);
				std::cin >> move_phase;
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
			case 8:
				underdataSet(60,450,0,9,2);
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
				maximdataSet(100,500,500,150);
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
				errordataSet(1.0,1.0,0.01,40);
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
				underdataSet(400,30,-180,27,3);
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
				if(lidar_info[Y_INFO] <= 357 && lidar_info[Y_INFO] >= 355 && catcher == false){
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
				break;
			case 42:
				if(lidar_info[Y_INFO] >= 162 && lidar_info[Y_INFO] <= 164 && catcher == false){
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
				tar_error_y = (20 - lidar_info[X_INFO]);
				underdataSet(saver[0]-400,status[Y_POS] + tar_error_y,0,47,2);
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
				underdataSet(0,0,-45,51,0);
				arartCheck(1);
				under_pub.publish(under_carryer);
				break;
			case 51:
				errordataSet(5.0,5.0,0.01,1);
				error_pub.publish(error_data);
				maximdataSet(120,600,600,150);
				max_pub.publish(max_data);
				ROS_INFO("good night!");
				//usleep(3000000);
				ROS_INFO("ohayou");
				yumiya_phase.data = 1;
				yumi_addmission.publish(yumiya_phase);
				underdataSet(420,420,0,0,0);
				arartCheck(1);
				under_pub.publish(under_carryer);
				catcher = false;
				sleep(3);
				move_phase = 52;
				counter = 0;
				break;
			case 52:
				if(status[X_POS] >= 204 && status[X_POS] <= 208){
					if(status[Y_POS] >= 204 && status[Y_POS] <= 208){
						if(status[THETA] >= -0.79 && status[THETA] <= -0.78)
						{	
							catcher = true;
						}
					}
				}

				if(status[X_POS] >= 300){
                                        if(status[Y_POS] >= 300){
                                                        catcher = true;
                                                
                                        }
                                }

				if(catcher == true){
					yumiya_phase.data = 3;
					yumi_addmission.publish(yumiya_phase);
					saveman_a[0] = status[X_POS];
					saveman_a[1] = status[Y_POS];
					saveman_b[0] = status[THETA];	
					move_phase = 53;
				}else{
					yumiya_phase.data = 1;
					yumi_addmission.publish(yumiya_phase);
				}
				break;
			case 53:
				ROS_INFO("SHOOOOT! at x:%lf y:%lf theta:%lf",saveman_a[0],saveman_a[1],saveman_b[0]);
				//yumiya_phase.data = 1;
				//yumi_addmission.publish(yumiya_phase);
				move_phase = 0;
				break;
		}
		ROS_INFO("phase:%d x_position:%5.2f y_position:%5.2f theta:%5.5f depth:%5.2f lidar_x:%5.2f lidar_y:%5.2f vvvv%d counter%ld",move_phase,status[X_POS],status[Y_POS],status[THETA],depth[0],lidar_info[X_INFO],lidar_info[Y_INFO],catcher,counter);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
