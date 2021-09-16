#include <ros/ros.h>
#include <meisekisisui/ScrpSlave.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <iostream>
#include <unistd.h>

#define ID     0
#define CMD    1
#define DATA   2
#define CMD_ID 3

#define X_target      0
#define Y_target      1
#define Theta_target  2
#define POS_id        3
#define Go_addmission 4

int move_phase = 0;
int cmd_num = 0;
int addmitter = 0;
long int counter = 0;

std_msgs::Int16MultiArray directive;

std_msgs::Int16MultiArray under_carryer;

std_msgs::Int16 yumiya_phase;

void scrpSet(int id, int cmd, int data);

void getcheck(const std_msgs::Int16& buffer){
	move_phase = buffer.data;
}

void underdataSet(int x_target,int y_target,int theta_target,int id){
	under_carryer.data[X_target]     = x_target;
	under_carryer.data[Y_target]     = y_target;
	under_carryer.data[Theta_target] = theta_target;
	under_carryer.data[POS_id]       = id; 
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "main_com");
	ros::NodeHandle nh;
	ros::Rate loop_rate(1000);

	ros::Publisher  scrp_pub  = nh.advertise<std_msgs::Int16MultiArray>("datan", 10);
	ros::Publisher under_pub  = nh.advertise<std_msgs::Int16MultiArray>("asimawari",10);
	ros::Publisher  yumi_pub  = nh.advertise<std_msgs::Int16>("shooter",10);
	ros::Subscriber run_check = nh.subscribe("check",10,getcheck);

	directive.data.resize(4);
	under_carryer.data.resize(5);

	while(ros::ok()){
		switch(move_phase){
			case 0:
				std::cin >> move_phase;
				break;
			case 1:
				underdataSet(400,20,0,2);
				under_carryer.data[Go_addmission] = 1;
				under_pub.publish(under_carryer);
				break;
			case 2:
				underdataSet(400,400,0,3);
				under_carryer.data[Go_addmission] = 1;
				under_pub.publish(under_carryer);
				break;
			case 3:
				underdataSet(400,400,90,4);
				under_carryer.data[Go_addmission] = 1;
				under_pub.publish(under_carryer);
				break;
			case 4:
				yumiya_phase.data = 1;
				yumi_pub.publish(yumiya_phase);
				underdataSet(20,400,90,5);
				under_carryer.data[Go_addmission] = 1;
				under_pub.publish(under_carryer);
				break;
			case 5:
				underdataSet(20,400,178,6);
				under_carryer.data[Go_addmission] = 1;
				under_pub.publish(under_carryer);
				break;	
			case 6:
				underdataSet(20,250,178,7);
				under_carryer.data[Go_addmission] = 1;
				under_pub.publish(under_carryer);
				break;
			case 7:
				underdataSet(20,250,0,8);
				under_carryer.data[Go_addmission] = 1;
				under_pub.publish(under_carryer);
				break;
			case 8:
				yumiya_phase.data = 3;
				yumi_pub.publish(yumiya_phase);	
				if(counter < 5000){
					counter++;
				}else{
					counter = 0;
					move_phase = 9;
				}			
				break;
			case 9:
				underdataSet(20,250,178,10);
				under_carryer.data[Go_addmission] = 1;
				under_pub.publish(under_carryer);
				break;
			case 10:
				underdataSet(20,20,178,11);
				under_carryer.data[Go_addmission] = 1;
				under_pub.publish(under_carryer);
				break;
			case 11:
				underdataSet(20,20,0,12);
				under_carryer.data[Go_addmission] = 1;
				under_pub.publish(under_carryer);
				break;
			case 12:
				break;


		}		
		std::cout << move_phase << std::endl;
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
