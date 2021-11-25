#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <meisekisisui/realsenseInfo.h>
#include <iostream>
#include <unistd.h>
#include <thread>

#define tyuku_ABS     true
#define tyujitu_ABS   false
#define kind_of_arrow arrow_flag

//--param = 250--//

bool arrow_flag = 1;

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

int lidar_dist;
int supershoot;
int lidar_angle;

int target_distance = 670;

int shoot_buffer = 0;
int success_flag = true;

float status[5];

int move_phase = 0;
int cmd_num = 0;
int addmitter = 0;
long int counter = 0;

float x_tar_sense = 308;
float y_tar_sense = 420;
float tar_error_x;
float tar_error_y;

int read_key = 0;
char read_ch[5];

float lidar_info[2];

bool arart = false;
int arart_status[2] = {1,1};
bool ignore_flag = false;

float saveman_a[2];
float saveman_b[2];

bool catcher = false;
float saver[2] = {0,0};

int ppp[4] = {0,0,0,0};
float adder = 0;
bool check_the_target = false;

float shoot_line = 0;

int shoot_phase = 0;

bool consolar = false;

int min_er = 70;
int max_er = 74;
int shoot_counter;

float allowed_speed = 300;

std_msgs::Int16MultiArray max_data;
//std_msgs::Float32MultiArray under_carryer;
std_msgs::Int16MultiArray under_carryer;
std_msgs::Int8 yumiya_phase;
//std_msgs::Int16MultiArray shoot_recogniser;
std_msgs::Float32MultiArray error_data;
std_msgs::Int8 neck_pos;
std_msgs::Int16 resetter;
std_msgs::Int8  spec_Add;
std_msgs::Int8 lidar_sarvo;
std_msgs::Int16 lidar_distance;
std_msgs::Int16 supershooter;

void arartCheck(int n){
	if(arart == 1 && ignore_flag == false){
		under_carryer.data[Go_addmission] = 0;
	}else{
		under_carryer.data[Go_addmission] = n;
	}
}

void getcheck(const std_msgs::Int16& buffer){
	move_phase = buffer.data;
}

void getStatus(const std_msgs::Float32/*MultiArray*/& under_status){
	//for(int i = 0; i <5; ++i){
		status[THETA] = under_status.data;
	//}
}

/*void getIgnoreflag(const std_msgs::Int16& Ignore_flag){
  if(Ignore_flag.data == 1){
  ignore_flag = true;
  }else{
  ignore_flag == false;
  }
  }*/

/*void underdataSet(float x_target,float y_target,float theta_target,float id,float cul_mode){
  under_carryer.data[X_target]      = x_target;
  under_carryer.data[Y_target]      = y_target;
  under_carryer.data[Theta_target]  = theta_target;
  under_carryer.data[POS_id]        = id; 
  under_carryer.data[Theta_culmode] = cul_mode;
  }*/

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

void errordataSet(float x_error, float y_error, float theta_error, float k){
	error_data.data[X_POS] =     x_error; 
	error_data.data[Y_POS] =     y_error;	
	error_data.data[THETA] = theta_error;
	error_data.data[3]     = k;
}

void getArart(const std_msgs::Int8& pushed_msg){
	arart = pushed_msg.data;
	arart_status[NOW] = arart;
	if(arart_status[NOW] != arart_status[PREV]){
		if(arart_status[NOW] == 1){
			ROS_INFO("robot has been rocked by remote emargency");
		}else{
			ROS_INFO("robot is now avairable");
		}
	}
	arart_status[PREV] = arart_status[NOW];
}

void getLidar(const std_msgs::Int32MultiArray& lidar_lidar){
	for(int i = 0; i < 2; ++i){
		lidar_info[i] = lidar_lidar.data[i] / 10;
		if(lidar_info[i] > 900){
			lidar_info[i] = 900;

		}
	}
}


void getshootcheck(const std_msgs::Int8& shooter){
	shoot_phase = shooter.data;
}

void shoot_safety(int phase){
	//if(status[NOW_VX] < allowed_speed && status[NOW_VY] < allowed_speed){
	//	if(status[THETA] < -30/180*M_PI && status[THETA] > -150/180*M_PI){
	yumiya_phase.data = phase;
	//	}
	//}else{
	//	yumiya_phase.data = 0;
	//}
}

void read_thread(){
	while(true){
		std::cin >> read_ch;
		read_key = atoi(read_ch);
		if(read_key >= 101){
			resetter.data = read_key;
			ROS_INFO("phase:%d x_position:%5.2f y_position:%5.2f theta:%5.5f shoot_phase:%d lidar_x:%5.2f lidar_y:%5.2f vvvv%d counter%ld adder:%5.2f tof:%d",move_phase,status[X_POS],status[Y_POS],status[THETA],shoot_phase,lidar_info[X_INFO],lidar_info[Y_INFO],arart,counter,adder,check_the_target);
		}else{
			if(arart == 1){
				ROS_INFO("remote emargency switch online");
			}else{
				resetter.data = 10;	
				move_phase = read_key;
			}
		}
	}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "main_com");
	ros::NodeHandle nh;
	ros::Rate loop_rate(1000);

	ros::Publisher max_pub         = nh.advertise<std_msgs::Int16MultiArray>("maxim", 10);
	//ros::Publisher under_pub       = nh.advertise<std_msgs::Float32MultiArray>("asimawari",10);
	ros::Publisher under_pub       = nh.advertise<std_msgs::Int16MultiArray>("asimawari",10);
	ros::Publisher error_pub       = nh.advertise<std_msgs::Float32MultiArray>("errors",10);
	ros::Publisher yumi_addmission = nh.advertise<std_msgs::Int8>("shooter",10);
	ros::Publisher neck_pub        = nh.advertise<std_msgs::Int8>("necker",10);
	ros::Publisher reset_pub       = nh.advertise<std_msgs::Int16>("reset",10);
	ros::Publisher spadd_pub       = nh.advertise<std_msgs::Int8>("special_add",10);
	ros::Publisher lidsarv_pub     = nh.advertise<std_msgs::Int8>("lidar_sarvo_data",10);
	ros::Publisher lidar_dist_pub  = nh.advertise<std_msgs::Int16>("target_distance",10);
	ros::Publisher naname_tar_pub  = nh.advertise<std_msgs::Int16>("naname_target",10);

	ros::Subscriber run_check      = nh.subscribe("check"   ,10,getcheck);
	ros::Subscriber status_sub     = nh.subscribe("statuses",10,getStatus);
	ros::Subscriber arart_sub      = nh.subscribe("pushed"  ,10,getArart);
	ros::Subscriber get_lidar      = nh.subscribe("lidar_info",100,getLidar);
	ros::Subscriber shoot_check    = nh.subscribe("shooter_checker",10,getshootcheck);
	//ros::Subscriber ignore_sub     = nh.subscribe("ignored",10,getIgnoreflag);

	max_data.data.resize(4);
	under_carryer.data.resize(6);
	error_data.data.resize(3);
	//shoot_recogniser.data.resize(2);

	if(kind_of_arrow == tyujitu_ABS){
		lidar_dist = 10;
		supershoot = 107;
		lidar_angle = 60;
	}else if(tyuku_ABS){
		lidar_dist = 5;
		supershoot = 50/*25*/;//<--大きければ大きいほど発射タイミングが早い
		lidar_angle = 54;
		min_er = 88;
		max_er = 92;
	}

	spec_Add.data = 0;
	spadd_pub.publish(spec_Add);
	resetter.data = 0;
	reset_pub.publish(resetter);
	//lidar_sarvo.data = 45;
	neck_pos.data = 0;
	lidar_distance.data = 10;
	lidar_dist_pub.publish(lidar_distance);
	supershooter.data = supershoot;
	naname_tar_pub.publish(supershooter);

	std::thread read_thre(read_thread);

	while(ros::ok()){
		switch(move_phase){
			case -1:
				neck_pos.data = 1;
				neck_pub.publish(neck_pos);
				spec_Add.data = 3;
				spadd_pub.publish(spec_Add);
				break;
			case 0:
				//neck_pos.data = 0;
				neck_pub.publish(neck_pos);
				under_carryer.data[Go_addmission] = 0;
				under_pub.publish(under_carryer);
				reset_pub.publish(resetter);
				//lidar_sarvo.data = 90;
				//lidsarv_pub.publish(lidar_sarvo);
				break;
			case 7:
				yumiya_phase.data = 3;
				if(counter < 1000){
					counter++;
				}else if(counter < 3000){
					counter++;
					yumi_addmission.publish(yumiya_phase);
				}else{
					counter = 0;
					move_phase = 14;
				}

				break;
			/*case 10:
				neck_pos.data = 1;
				neck_pub.publish(neck_pos);
				//shoot_recogniser.data[0] = 4;
				//shoot_recogniser.data[1] = 1;
				underdataSet(0,0,-90,11,0);	
				arartCheck(1);
				under_pub.publish(under_carryer);
				break;
			case 11:
				underdataSet(400,30,0,12,0);
				arartCheck(1);
				under_pub.publish(under_carryer);
				break;
			case 12:
				underdataSet(400,30,90,13,0);
				arartCheck(1);
				under_pub.publish(under_carryer);
				break;
			case 13:
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
			case 14:
				if(catcher == false){
					catcher = true;
					saver[0] = status[X_POS];
					saver[1] = status[Y_POS];
				}
				underdataSet(saver[0],saver[1],180,15,3);
				arartCheck(1);
				under_pub.publish(under_carryer);
				maximdataSet(200,1000,1000,150);
				max_pub.publish(max_data);
				errordataSet(5.0,5.0,0.01,1);
				error_pub.publish(error_data);
				break;
			case 15:
				underdataSet(400,30,0,16,3);
				arartCheck(1);
				under_pub.publish(under_carryer);
				catcher = false;
				break;
			case 16:
				underdataSet(400,30,-180,17,0);
				arartCheck(1);
				under_pub.publish(under_carryer);
				break;
			case 17:
				resetter.data = 2;
				reset_pub.publish(resetter);
				move_phase = 0;
				break;
			case 20:
				neck_pos.data = 1;
				neck_pub.publish(neck_pos);
				maximdataSet(75,375,375,150);
				max_pub.publish(max_data);
				yumiya_phase.data = 1;
				yumi_addmission.publish(yumiya_phase);
				underdataSet(0,420,1,23,0);
				arartCheck(1);
				under_pub.publish(under_carryer);
				errordataSet(5.0,5.0,0.01,1);
				error_pub.publish(error_data);
				catcher = false;
				check_the_target = false;
				move_phase = 21;
				break;
			case 21:
				if(lidar_info[X_INFO] <= 380 && check_the_target == false){
					adder = lidar_info[Y_INFO];
					check_the_target = true;
				}
				if(lidar_info[Y_INFO] <= adder - min_er && lidar_info[Y_INFO] >= adder - max_er && catcher == false && check_the_target == true){
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
						move_phase = 22;
					}
				}
				break;
			case 22:
				if(lidar_info[X_INFO] <= 380 && check_the_target == false && lidar_info[Y_INFO] < 250){
					adder = lidar_info[Y_INFO];
					check_the_target = true;
				}

				if(lidar_info[Y_INFO] <= adder - min_er && lidar_info[Y_INFO] >= adder - max_er && catcher == false && check_the_target == true){
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
						move_phase = 23;
					}
				}else{
					yumiya_phase.data = 1;
					yumi_addmission.publish(yumiya_phase);
				} 
				break;
			case 23:
				yumiya_phase.data = 3;
				yumi_addmission.publish(yumiya_phase);
				sleep(3);
				if(catcher == false){
					catcher = true;
					saver[0] = status[X_POS];
					saver[1] = status[Y_POS];
				}
				underdataSet(saver[0],saver[1],180,24,3);
				arartCheck(1);
				under_pub.publish(under_carryer);
				maximdataSet(200,1000,1000,150);
				max_pub.publish(max_data);
				break;
			case 24:
				underdataSet(saver[0],20,0,25,3);
				arartCheck(1);
				under_pub.publish(under_carryer);
				counter = 0;
				catcher = false;
				break;
			case 25:
				underdataSet(saver[0],20,-90,26,2);
				arartCheck(1);
				under_pub.publish(under_carryer);
				break;
			case 26:
				tar_error_y = (40 - lidar_info[X_INFO]);
				underdataSet(saver[0]-370,status[Y_POS] + tar_error_y,0,27,2);
				arartCheck(1);
				under_pub.publish(under_carryer);
				break;
			case 27:
				ROS_INFO("first lidar:%lf theta:%lf second lidar:%lf theta:%lf",saveman_a[0],saveman_a[1],saveman_b[0],saveman_b[1]);
				move_phase = 0;
				break;
			case 30:
				counter = 0;
				errordataSet(1.0,1.0,0.01,40);
				error_pub.publish(error_data);
				catcher = false;
				move_phase = 31;
				break;
			case 31:
				if(counter < 1000){
					errordataSet(5.0,5.0,0.01,1);
					error_pub.publish(error_data);
					maximdataSet(240,1200,1200,150);
					max_pub.publish(max_data);
					if(catcher == false){
						underdataSet(0,0,-45,32,0);
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
			case 32:
				arartCheck(0);
				spec_Add.data = 1;
				spadd_pub.publish(spec_Add);
				if(shoot_phase != 2){
					yumiya_phase.data = 1;
					yumi_addmission.publish(yumiya_phase);
				}else{
					move_phase = 33;
				}
				break;
			case 33:
				underdataSet(420,420,0,0,0);
				arartCheck(1);
				under_pub.publish(under_carryer);
				move_phase = 34;
				break;
			case 34:
				if(status[X_POS] >= 220){
					if(status[Y_POS] >= 220){
						yumiya_phase.data = 3;
						yumi_addmission.publish(yumiya_phase);
						spec_Add.data = 0;
						spadd_pub.publish(spec_Add);
					}
				}
				break;*/	
			case 10:
				lidar_distance.data = lidar_dist;
				lidar_dist_pub.publish(lidar_distance);
				supershooter.data = supershoot;
				naname_tar_pub.publish(supershooter);
				lidar_sarvo.data = 50;
				lidsarv_pub.publish(lidar_sarvo);
				maximdataSet(240,1200,1200,170);
				max_pub.publish(max_data);
				neck_pos.data = 1;
				neck_pub.publish(neck_pos);
				underdataSet(40,600,0,11,0);
				arartCheck(1);
				under_pub.publish(under_carryer);
				break;
			case 11:
				counter = 0;
				errordataSet(3.0,3.0,0.01,40);
				error_pub.publish(error_data);
				catcher = false;
				move_phase = 12;
				break;
			case 12:
				lidar_sarvo.data = 50;
				lidsarv_pub.publish(lidar_sarvo);	
				errordataSet(1.0,1.0,0.01,40);
				error_pub.publish(error_data);
				maximdataSet(180,1000,1000,170);
				max_pub.publish(max_data);
				underdataSet(40,600,-135,13,6);
				arartCheck(1);
				under_pub.publish(under_carryer);
				//shoot_safety(1);
				//yumi_addmission.publish(yumiya_phase);
				spec_Add.data = 1;
				spadd_pub.publish(spec_Add);
				break;
			case 13:
				maximdataSet(240,1200,1200,170);
				max_pub.publish(max_data);
				arartCheck(0);
				spec_Add.data = 1;
				spadd_pub.publish(spec_Add);
				if(shoot_phase != 2){
					//yumiya_phase.data = 1;
					shoot_safety(1);
					yumi_addmission.publish(yumiya_phase);
				}else{
					move_phase = 14;
				}
				break;
			case 14:
				maximdataSet(240,1200,1200,170);
				max_pub.publish(max_data);
				underdataSet(600,40,-135,15,6);
				arartCheck(1);
				under_pub.publish(under_carryer);
				//move_phase = 45;
				break;
			case 15:
				/*if(status[X_POS] >= 400){
				  if(status[Y_POS] <= 400){
				  if(shoot_phase == 2){
				//yumiya_phase.data = 3;
				shoot_safety(3);
				yumi_addmission.publish(yumiya_phase);
				spec_Add.data = 0;
				spadd_pub.publish(spec_Add);
				ROS_INFO("untititititititititi");
				success_flag = false;
				}else{
				success_flag = true;
				}
				}
				}*/
				                                errordataSet(3.0,3.0,0.03,20);
                                error_pub.publish(error_data);
                                        underdataSet(600,40,-90,16,6);
                                        arartCheck(1);
                                        under_pub.publish(under_carryer);
				                                        spec_Add.data = 0;
                                        spadd_pub.publish(spec_Add);
					break;		
			case 16:
				if(shoot_phase == 2){
					shoot_safety(3);
					yumi_addmission.publish(yumiya_phase);
					spec_Add.data = 0;
					spadd_pub.publish(spec_Add);
				}else{
					move_phase = 17;
				}
				break;
			case 17:
				spec_Add.data = 0;
				spadd_pub.publish(spec_Add);
				maximdataSet(240,1200,1200,170);
				max_pub.publish(max_data);
				underdataSet(40,40,-90,18,0);
				arartCheck(1);
				under_pub.publish(under_carryer);
				break;
			case 18:
				underdataSet(40,40,0,0,0);
				arartCheck(1);
				under_pub.publish(under_carryer);
				ROS_INFO("%d",success_flag);
				break;
			case 20:
				                                neck_pos.data = 1;
                                neck_pub.publish(neck_pos);
				lidar_sarvo.data = 60;
				lidsarv_pub.publish(lidar_sarvo);
				underdataSet(20,30,0,21,0);
				arartCheck(1);
				under_pub.publish(under_carryer);
				errordataSet(3.0,3.0,0.01,10);
				error_pub.publish(error_data);
				errordataSet(1.0,1.0,0.01,20);
                                error_pub.publish(error_data);
                                catcher = false;
                                check_the_target = false;
				break;
			case 21:
				shoot_safety(1);
				yumi_addmission.publish(yumiya_phase);
				maximdataSet(75,375,375,170);
				max_pub.publish(max_data);
				sleep(1);
				target_distance = lidar_info[X_INFO] - 50/*670*/;
				underdataSet(620,30,0,26,0);
				arartCheck(1);
				under_pub.publish(under_carryer);
				//yumiya_phase.data = 1;
				//shoot_safety(1);
				//yumi_addmission.publish(yumiya_phase);
				errordataSet(5.0,5.0,0.01,1);
				error_pub.publish(error_data);
				move_phase = 23;
				break;
			case 22:
				if(lidar_info[X_INFO] <= target_distance && check_the_target == false){
					adder = lidar_info[Y_INFO];
					check_the_target = true;
				}
				if(lidar_info[Y_INFO] <= adder -  min_er && (status[THETA] > (-0.5*M_PI/180) && status[THETA] < (0.5*M_PI/180))/*&& lidar_info[Y_INFO] >= adder - max_er*/ && catcher == false && check_the_target == true){
					catcher = true;
					counter = 0;
					saveman_a[0] = status[Y_POS];
					saveman_a[1] = status[THETA];
				}
				if(catcher == true){
					if(counter < 500){
						//yumiya_phase.data = 3;
						shoot_safety(3);
						yumi_addmission.publish(yumiya_phase);
						++counter;
					}else if(counter < 700){
						//yumiya_phase.data = 1;
						shoot_safety(1);
						yumi_addmission.publish(yumiya_phase);
						++counter;
					}else{
						//yumiya_phase.data = 1;
						shoot_safety(1);
						yumi_addmission.publish(yumiya_phase);
						if(lidar_info[X_POS]  > target_distance + 40){
							counter = 0;
							check_the_target = false;
							catcher = false;
							move_phase = 23;
						}
					}
				}
				break;
			case 23:
				if(lidar_info[X_INFO] <= target_distance && check_the_target == false /*&& status[X_POS] < 400:260*/){
					adder = lidar_info[Y_INFO];
					check_the_target = true;
				}
				if(lidar_info[Y_INFO] <= adder - min_er&& (status[THETA] > (-0.5*M_PI/180) && status[THETA] < (0.5*M_PI/180)) /*&& lidar_info[Y_INFO] >= adder - max_er*/ && catcher == false && check_the_target == true){
					catcher = true;
					counter = 0;
					saveman_a[0] = lidar_info[Y_INFO];
					saveman_a[1] = status[THETA];
				}
				if(catcher == true){
					if(counter < 500){
						//yumiya_phase.data = 3;
						shoot_safety(3);
						yumi_addmission.publish(yumiya_phase);
						++counter;
					}else if(counter < 700){
						//yumiya_phase.data = 1;
						shoot_safety(1);
						yumi_addmission.publish(yumiya_phase);
						++counter;
					}else{
						//yumiya_phase.data = 1;
						shoot_safety(1);
						yumi_addmission.publish(yumiya_phase);
						if(lidar_info[X_POS]  > target_distance + 40){
							counter = 0;
							check_the_target = false;
							catcher = false;
							move_phase = 24;
						}
					}
				}else{
					//yumiya_phase.data = 1;
					shoot_safety(1);
					yumi_addmission.publish(yumiya_phase);
				}
				break;

			case 24:
				if(lidar_info[X_INFO] <= target_distance && check_the_target == false /*&& status[X_POS] < 250:470*/){
					adder = lidar_info[Y_INFO];
					check_the_target = true;
				}

				if(lidar_info[Y_INFO] <= adder - min_er && (status[THETA] > (-0.5*M_PI/180) && status[THETA] < (0.5*M_PI/180)) /*&& lidar_info[Y_INFO] >= adder - max_er*/ && catcher == false && check_the_target == true){
					catcher = true;
					counter = 0;
					saveman_b[0] = lidar_info[Y_INFO];
					saveman_b[1] = status[THETA];
				}
				if(catcher == true){
					//yumiya_phase.data = 3;
					shoot_safety(3);
					yumi_addmission.publish(yumiya_phase);
					if(counter < 1000){
						++counter;
					}else{
						/*counter = 0;
						  check_the_target = false;
						  catcher = false;*/
						move_phase = 25;
					}
				}else{
					//yumiya_phase.data = 1;
					shoot_safety(1);
					yumi_addmission.publish(yumiya_phase);
				}
				break;
			case 25:
				counter = 0;
				check_the_target = false;
				catcher = false;
				//yumiya_phase.data = 3;
				shoot_safety(3);
				yumi_addmission.publish(yumiya_phase);
				break;
			case 26:
				//yumiya_phase.data = 3;
				if(shoot_phase == 2){
					shoot_safety(3);
					yumi_addmission.publish(yumiya_phase);
					sleep(3);
				}
				if(catcher == false){
					catcher = true;
					saver[0] = status[X_POS];
					saver[1] = status[Y_POS];
				}
				errordataSet(5.0,5.0,0.03,1);
				error_pub.publish(error_data);
				underdataSet(620,30,180,27,7);
				arartCheck(1);
				under_pub.publish(under_carryer);
				maximdataSet(200,1000,1000,170);
				max_pub.publish(max_data);
				break;
			case 27:
				tar_error_y = (40 - lidar_info[Y_INFO]);
				underdataSet(40/* - lidar_info[X_INFO]+ status[Y_POS]*/,/*tar_error_y + status[X_POS]*/30,180,/*28*/0,7);
				arartCheck(1);
				under_pub.publish(under_carryer);
				counter = 0;
				catcher = false;
				break;
			case 28:
				if(catcher == false){
					catcher = true;
					saver[0] = status[X_POS];
					saver[1] = status[Y_POS];
				}
				underdataSet(/*saver[0]*/30,/*saver[1]*/30,0,0,7);
				arartCheck(1);
				under_pub.publish(under_carryer);
				break;
			case 40:
				lidar_distance.data = lidar_dist;
				lidar_dist_pub.publish(lidar_distance);
				supershooter.data = supershoot;
				naname_tar_pub.publish(supershooter);
				lidar_sarvo.data = 50;
				lidsarv_pub.publish(lidar_sarvo);
				maximdataSet(240,1200,1200,170);
				max_pub.publish(max_data);
				neck_pos.data = 1;
				neck_pub.publish(neck_pos);
				underdataSet(40,600,0,41,0);
				arartCheck(1);
				under_pub.publish(under_carryer);
				break;
			case 41:
				counter = 0;
				errordataSet(3.0,3.0,0.01,40);
				error_pub.publish(error_data);
				catcher = false;
				move_phase = 42;
				break;
			case 42:
				lidar_sarvo.data = 50;
				lidsarv_pub.publish(lidar_sarvo);	
				errordataSet(1.0,1.0,0.01,40);
				error_pub.publish(error_data);
				maximdataSet(180,1000,1000,170);
				max_pub.publish(max_data);
				underdataSet(40,600,-134,43,6);
				arartCheck(1);
				under_pub.publish(under_carryer);
				//shoot_safety(1);
				//yumi_addmission.publish(yumiya_phase);
				spec_Add.data = 1;
				spadd_pub.publish(spec_Add);
				break;
			case 43:
				maximdataSet(240,1200,1200,170);
				max_pub.publish(max_data);
				arartCheck(0);
				spec_Add.data = 1;
				spadd_pub.publish(spec_Add);
				if(shoot_phase != 2){
					//yumiya_phase.data = 1;
					shoot_safety(1);
					yumi_addmission.publish(yumiya_phase);
				}else{
					move_phase = 44;
				}
				break;
			case 44:
				maximdataSet(240,1200,1200,170);
				max_pub.publish(max_data);
				underdataSet(600,40,-134,45,6);
				arartCheck(1);
				under_pub.publish(under_carryer);
				//move_phase = 45;
				break;
			case 45:
				/*if(status[X_POS] >= 400){
				  if(status[Y_POS] <= 400){
				  if(shoot_phase == 2){
				//yumiya_phase.data = 3;
				shoot_safety(3);
				yumi_addmission.publish(yumiya_phase);
				spec_Add.data = 0;
				spadd_pub.publish(spec_Add);
				ROS_INFO("untititititititititi");
				success_flag = false;
				}else{
				success_flag = true;
				}
				}
				}*/
				if(shoot_phase == 2){
					underdataSet(600,40,-90,46,6);
					arartCheck(1);
					under_pub.publish(under_carryer);
				}else{
					move_phase = 47;
				}
				break;		
			case 46:
				if(shoot_phase == 2){
					shoot_safety(3);
					yumi_addmission.publish(yumiya_phase);
					spec_Add.data = 0;
					spadd_pub.publish(spec_Add);
				}else{
					move_phase = 47;
				}
				break;
			case 47:
				spec_Add.data = 0;
				spadd_pub.publish(spec_Add);
				errordataSet(3.0,3.0,0.03,20);
				error_pub.publish(error_data);
				maximdataSet(240,1200,1200,170);
				max_pub.publish(max_data);
				underdataSet(600,40,-270,48,6);
				arartCheck(1);
				under_pub.publish(under_carryer);
				break;
			case 48:
				underdataSet(40,40,-270,49,6);
				arartCheck(1);
				under_pub.publish(under_carryer);
				break;
			case 49:
				underdataSet(40,40,0,0,0);
				arartCheck(1);
				under_pub.publish(under_carryer);
				ROS_INFO("%d",success_flag);
				break;
			case 50:
				lidar_sarvo.data = 50;
				lidsarv_pub.publish(lidar_sarvo);
				underdataSet(20,30,0,51,0);
				arartCheck(1);
				under_pub.publish(under_carryer);
				errordataSet(3.0,3.0,0.01,10);
				error_pub.publish(error_data);
				break;
			case 51:
				neck_pos.data = 1;
				neck_pub.publish(neck_pos);
				//maximdataSet(75,375,375,170);
				//max_pub.publish(max_data);
				underdataSet(20,30,-90,52,0);
				arartCheck(1);
				under_pub.publish(under_carryer);
				errordataSet(1.0,1.0,0.01,20);
				error_pub.publish(error_data);
				catcher = false;
				check_the_target = false;
				break;
			case 52:
				shoot_safety(1);
				yumi_addmission.publish(yumiya_phase);
				maximdataSet(75,375,375,170);
				max_pub.publish(max_data);
				sleep(1);
				target_distance = lidar_info[X_INFO] - 30/*670*/;
				underdataSet(620,30,-90,57,0);
				arartCheck(1);
				under_pub.publish(under_carryer);
				//yumiya_phase.data = 1;
				//shoot_safety(1);
				//yumi_addmission.publish(yumiya_phase);
				errordataSet(5.0,5.0,0.01,1);
				error_pub.publish(error_data);
				move_phase = 53;
				break;
			case 53:
				if(lidar_info[X_INFO] <= target_distance && check_the_target == false){
					adder = status[X_POS];
					check_the_target = true;
				}
				if(status[X_POS] >= adder + min_er&& status[X_POS] <= adder + max_er && catcher == false && check_the_target == true){
					catcher = true;
					counter = 0;
					saveman_a[0] = status[Y_POS];
					saveman_a[1] = status[THETA];
				}
				if(catcher == true){
					if(counter < 500){
						//yumiya_phase.data = 3;
						shoot_safety(3);
						yumi_addmission.publish(yumiya_phase);
						++counter;
					}else if(counter < 700){
						//yumiya_phase.data = 1;
						shoot_safety(1);
						yumi_addmission.publish(yumiya_phase);
						++counter;
					}else{
						//yumiya_phase.data = 1;
						shoot_safety(1);
						yumi_addmission.publish(yumiya_phase);
						counter = 0;
						check_the_target = false;
						catcher = false;
						if(lidar_info[X_POS]+10  > target_distance){
							move_phase = 54;
						}
					}
				}
				break;
			case 54:
				if(lidar_info[X_INFO] <= target_distance && check_the_target == false /*&& status[X_POS] < 400:260*/){
					adder = status[X_POS];
					check_the_target = true;
				}
				if(status[X_POS] >= adder + min_er && status[X_POS] <= adder + max_er && catcher == false && check_the_target == true){
					catcher = true;
					counter = 0;
					saveman_a[0] = lidar_info[Y_INFO];
					saveman_a[1] = status[THETA];
				}
				if(catcher == true){
					if(counter < 500){
						//yumiya_phase.data = 3;
						shoot_safety(3);
						yumi_addmission.publish(yumiya_phase);
						++counter;
					}else if(counter < 700){
						//yumiya_phase.data = 1;
						shoot_safety(1);
						yumi_addmission.publish(yumiya_phase);
						++counter;
					}else{
						//yumiya_phase.data = 1;
						shoot_safety(1);
						yumi_addmission.publish(yumiya_phase);
						counter = 0;
						check_the_target = false;
						catcher = false;
						if(lidar_info[X_POS]+10  > target_distance){
							move_phase = 55;
						}
					}               
				}else{
					//yumiya_phase.data = 1;
					shoot_safety(1);
					yumi_addmission.publish(yumiya_phase);
				}
				break;

			case 55:
				if(lidar_info[X_INFO] <= target_distance && check_the_target == false /*&& status[X_POS] < 250:470*/){
					adder = status[X_POS];
					check_the_target = true;
				}

				if(status[X_POS] >= adder + min_er && status[X_POS] <= adder + max_er && catcher == false && check_the_target == true){
					catcher = true;
					counter = 0;
					saveman_b[0] = lidar_info[Y_INFO];
					saveman_b[1] = status[THETA];
				}
				if(catcher == true){
					//yumiya_phase.data = 3;
					shoot_safety(3);
					yumi_addmission.publish(yumiya_phase);
					if(counter < 1000){
						++counter;
					}else{
						counter = 0;
						check_the_target = false;
						catcher = false;
						move_phase = 56;
					}
				}else{
					//yumiya_phase.data = 1;
					shoot_safety(1);
					yumi_addmission.publish(yumiya_phase);
				} 
				break;
			case 56:
				//yumiya_phase.data = 3;
				shoot_safety(3);
				yumi_addmission.publish(yumiya_phase);
				break;
			case 57:
				//yumiya_phase.data = 3;
				if(shoot_phase == 2){
					shoot_safety(3);
					yumi_addmission.publish(yumiya_phase);
					sleep(3);
				}
				if(catcher == false){
					catcher = true;
					saver[0] = status[X_POS];
					saver[1] = status[Y_POS];
				}
				errordataSet(5.0,5.0,0.03,1);
				error_pub.publish(error_data);
				underdataSet(620,30,-270,58,6);
				arartCheck(1);
				under_pub.publish(under_carryer);
				maximdataSet(200,1000,1000,170);
				max_pub.publish(max_data);
				break;
			case 58:
				tar_error_y = (40 - lidar_info[Y_INFO]);
				underdataSet(40/* - lidar_info[X_INFO]+ status[Y_POS]*/,/*tar_error_y + status[X_POS]*/20,-270,59,6);
				arartCheck(1);
				under_pub.publish(under_carryer);
				counter = 0;
				catcher = false;
				break;
			case 59:
				if(catcher == false){
					catcher = true;
					saver[0] = status[X_POS];
					saver[1] = status[Y_POS];
				}
				underdataSet(/*saver[0]*/30,/*saver[1]*/30,0,0,0);
				arartCheck(1);
				under_pub.publish(under_carryer);
				break;
				/*case 80:
				  ROS_INFO("first lidar:%lf theta:%lf second lidar:%lf theta:%lf",saveman_a[0],saveman_a[1],saveman_b[0],saveman_b[1]);
				  move_phase = 0;
				  break;*/
			case 60:
				lidar_sarvo.data = 60;
				lidsarv_pub.publish(lidar_sarvo);
				underdataSet(20,30,0,61,0);
				arartCheck(1);
				under_pub.publish(under_carryer);
				errordataSet(3.0,3.0,0.01,10);
				error_pub.publish(error_data);
				break;
			case 61:
				neck_pos.data = 1;
				neck_pub.publish(neck_pos);
				//maximdataSet(75,375,375,170);
				//max_pub.publish(max_data);
				underdataSet(20,30,-90,62,0);
				arartCheck(1);
				under_pub.publish(under_carryer);
				errordataSet(1.0,1.0,0.01,20);
				error_pub.publish(error_data);
				catcher = false;
				check_the_target = false;
				break;
			case 62:
				shoot_safety(1);
				yumi_addmission.publish(yumiya_phase);
				maximdataSet(75,375,375,170);
				max_pub.publish(max_data);
				sleep(1);
				target_distance = lidar_info[X_INFO] - 50/*670*/;
				underdataSet(620,30,-90,67,0);
				arartCheck(1);
				under_pub.publish(under_carryer);
				//yumiya_phase.data = 1;
				//shoot_safety(1);
				//yumi_addmission.publish(yumiya_phase);
				errordataSet(5.0,5.0,0.01,1);
				error_pub.publish(error_data);
				move_phase = 63;
				break;
			case 63:
				if(lidar_info[X_INFO] <= target_distance && check_the_target == false){
					adder = lidar_info[Y_INFO];
					check_the_target = true;
				}
				if(lidar_info[Y_INFO] <= adder -  min_er && (status[THETA] > (-90.5*M_PI/180) && status[THETA] < (-89.5*M_PI/180))/*&& lidar_info[Y_INFO] >= adder - max_er*/ && catcher == false && check_the_target == true){
					catcher = true;
					counter = 0;
					saveman_a[0] = status[Y_POS];
					saveman_a[1] = status[THETA];
				}
				if(catcher == true){
					if(counter < 500){
						//yumiya_phase.data = 3;
						shoot_safety(3);
						yumi_addmission.publish(yumiya_phase);
						++counter;
					}else if(counter < 700){
						//yumiya_phase.data = 1;
						shoot_safety(1);
						yumi_addmission.publish(yumiya_phase);
						++counter;
					}else{
						//yumiya_phase.data = 1;
						shoot_safety(1);
						yumi_addmission.publish(yumiya_phase);
						if(lidar_info[X_POS]  > target_distance + 40){
							counter = 0;
							check_the_target = false;
							catcher = false;
							move_phase = 64;
						}
					}
				}
				break;
			case 64:
				if(lidar_info[X_INFO] <= target_distance && check_the_target == false /*&& status[X_POS] < 400:260*/){
					adder = lidar_info[Y_INFO];
					check_the_target = true;
				}
				if(lidar_info[Y_INFO] <= adder - min_er&& (status[THETA] > (-90.5*M_PI/180) && status[THETA] < (-89.5*M_PI/180)) /*&& lidar_info[Y_INFO] >= adder - max_er*/ && catcher == false && check_the_target == true){
					catcher = true;
					counter = 0;
					saveman_a[0] = lidar_info[Y_INFO];
					saveman_a[1] = status[THETA];
				}
				if(catcher == true){
					if(counter < 500){
						//yumiya_phase.data = 3;
						shoot_safety(3);
						yumi_addmission.publish(yumiya_phase);
						++counter;
					}else if(counter < 700){
						//yumiya_phase.data = 1;
						shoot_safety(1);
						yumi_addmission.publish(yumiya_phase);
						++counter;
					}else{
						//yumiya_phase.data = 1;
						shoot_safety(1);
						yumi_addmission.publish(yumiya_phase);
						if(lidar_info[X_POS]  > target_distance + 40){
							counter = 0;
							check_the_target = false;
							catcher = false;
							move_phase = 65;
						}
					}               
				}else{
					//yumiya_phase.data = 1;
					shoot_safety(1);
					yumi_addmission.publish(yumiya_phase);
				}
				break;

			case 65:
				if(lidar_info[X_INFO] <= target_distance && check_the_target == false /*&& status[X_POS] < 250:470*/){
					adder = lidar_info[Y_INFO];
					check_the_target = true;
				}

				if(lidar_info[Y_INFO] <= adder - min_er && (status[THETA] > (-90.5*M_PI/180) && status[THETA] < (-89.5*M_PI/180)) /*&& lidar_info[Y_INFO] >= adder - max_er*/ && catcher == false && check_the_target == true){
					catcher = true;
					counter = 0;
					saveman_b[0] = lidar_info[Y_INFO];
					saveman_b[1] = status[THETA];
				}
				if(catcher == true){
					//yumiya_phase.data = 3;
					shoot_safety(3);
					yumi_addmission.publish(yumiya_phase);
					if(counter < 1000){
						++counter;
					}else{
						/*counter = 0;
						  check_the_target = false;
						  catcher = false;*/
						move_phase = 66;
					}
				}else{
					//yumiya_phase.data = 1;
					shoot_safety(1);
					yumi_addmission.publish(yumiya_phase);
				} 
				break;
			case 66:
				counter = 0;
				check_the_target = false;
				catcher = false;
				//yumiya_phase.data = 3;
				shoot_safety(3);
				yumi_addmission.publish(yumiya_phase);
				break;
			case 67:
				//yumiya_phase.data = 3;
				if(shoot_phase == 2){
					shoot_safety(3);
					yumi_addmission.publish(yumiya_phase);
					sleep(3);
				}
				if(catcher == false){
					catcher = true;
					saver[0] = status[X_POS];
					saver[1] = status[Y_POS];
				}
				errordataSet(5.0,5.0,0.03,1);
				error_pub.publish(error_data);
				underdataSet(620,30,-270,68,6);
				arartCheck(1);
				under_pub.publish(under_carryer);
				maximdataSet(200,1000,1000,170);
				max_pub.publish(max_data);
				break;
			case 68:
				tar_error_y = (40 - lidar_info[Y_INFO]);
				underdataSet(40/* - lidar_info[X_INFO]+ status[Y_POS]*/,/*tar_error_y + status[X_POS]*/30,-270,69,6);
				arartCheck(1);
				under_pub.publish(under_carryer);
				counter = 0;
				catcher = false;
				break;
			case 69:
				if(catcher == false){
					catcher = true;
					saver[0] = status[X_POS];
					saver[1] = status[Y_POS];
				}
				underdataSet(/*saver[0]*/30,/*saver[1]*/30,0,0,0);
				arartCheck(1);
				under_pub.publish(under_carryer);
				break;
				/*case 80:
				  ROS_INFO("first lidar:%lf theta:%lf second lidar:%lf theta:%lf",saveman_a[0],saveman_a[1],saveman_b[0],saveman_b[1]);
				  move_phase = 0;
				  break;*/
			case 80:
				lidar_sarvo.data = lidar_angle;
				lidsarv_pub.publish(lidar_sarvo);
				neck_pos.data = 1;
				neck_pub.publish(neck_pos);
				underdataSet(40,40,0,81,0);
				arartCheck(1);
				under_pub.publish(under_carryer);
				errordataSet(3.0,3.0,0.01,10);
				error_pub.publish(error_data);
				//lidar_distance.data = lidar_dist;
                                //lidar_dist_pub.publish(lidar_distance);
				break;
			case 81:
				maximdataSet(240,1200,1200,170);
				max_pub.publish(max_data);
				underdataSet(40,40,-90,82,0);
				arartCheck(1);
				under_pub.publish(under_carryer);
				errordataSet(1.0,1.0,0.01,20);
				error_pub.publish(error_data);
				catcher = false;
				break;
			case 82:
				if(shoot_phase != 2){
                                	spec_Add.data = 2;
                                	spadd_pub.publish(spec_Add);
					shoot_safety(1);
					yumi_addmission.publish(yumiya_phase);
				}else{
					move_phase = 83;
				}
				break;
			case 83:
				underdataSet(620,40,-90,0,84);
				arartCheck(1);
				under_pub.publish(under_carryer);
				//yumiya_phase.data = 1;
				errordataSet(5.0,5.0,0.01,1);
				error_pub.publish(error_data);
				break;
			case 84:
				if(shoot_phase == 2){
					shoot_safety(3);
					yumi_addmission.publish(yumiya_phase);
				}else{
					move_phase = 0;
				}
				break;
			case 91:
				neck_pos.data = 1;
				neck_pub.publish(neck_pos);
				yumiya_phase.data = 1;
				yumi_addmission.publish(yumiya_phase);
				move_phase = 0;
				break;
			case 92:
				neck_pos.data = 1;
				neck_pub.publish(neck_pos);
				yumiya_phase.data = 3;
				yumi_addmission.publish(yumiya_phase);
				move_phase = 0;
				break;
			case 93:
				neck_pos.data = 1;
				neck_pub.publish(neck_pos);
				yumiya_phase.data = 4;
				yumi_addmission.publish(yumiya_phase);
				move_phase = 0;
				break;
			case 94:
				neck_pos.data = 1;
				neck_pub.publish(neck_pos);
				yumiya_phase.data = 0;
				yumi_addmission.publish(yumiya_phase);
				move_phase = 0;
				break;
			case 95:
				lidar_sarvo.data = 45;
				lidsarv_pub.publish(lidar_sarvo);
				move_phase = 0;
				break;
			case 96:
				lidar_sarvo.data = 48;
				lidsarv_pub.publish(lidar_sarvo);
				move_phase = 0;
				break;
			case 97:
				lidar_sarvo.data = 54;
				lidsarv_pub.publish(lidar_sarvo);
				move_phase = 0;
				break;
			case 98:
				neck_pos.data = 0;
				neck_pub.publish(neck_pos);
				move_phase = 0;
				break;
			case 99:
				neck_pos.data = 1;
				neck_pub.publish(neck_pos);
				move_phase = 0;
				break;

		}
		if(move_phase != 0){
			ROS_INFO("theta :%lf phase:%d shoot_phase:%d lidar_x:%4.1f lidar_y:%4.1f arart%d shoot_phase%d",status[THETA],move_phase,shoot_phase,lidar_info[X_INFO],lidar_info[Y_INFO],arart,shoot_phase);
			//ROS_INFO("x_position:%5.2f y_position:%5.2f theta:%5.5f",status[X_POS],status[Y_POS],status[THETA]);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	read_thre.join();
	return 0;
}
