//======================================================================
/*! \file inno_can_send.cpp
 *
 * \copydoc Copyright
 * \author Pengfei Guo
 * \date June 20, 2020
 *
 * Receive msg from ros and broadcast to inno can card
 * 
 * IMPORTANT: every time open the pc, you should run "sudo chmod a+rw /dev/ttyACM0"
 * to activate the CAN card
 *///-------------------------------------------------------------------

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <iostream>
#include <cstdlib>
#include <math.h>
#include <string.h>
#include <sstream>
#include <memory>
#include <rclcpp/node.hpp>
#include <stdio.h>
#include <string.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <errno.h>
#include <ctime>
#include <chrono>
// This library is generated in C env
#ifdef __cplusplus
extern "C"
{
	#endif
	
	#include "include/lib_emuc_2.h"

	#ifdef __cplusplus
}
#endif

using std::placeholders::_1;
using std::cout;
using std::endl;




//======================================================================

class Inno_Can_Sender:
	public rclcpp::Node

	{
	private:
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr msg_rec;
	rclcpp::Logger inno_logger=this->get_logger();

	//Device settings
	int com_port_sel=24;
	int can_id=EMUC_CAN_1;
	int id_type=EMUC_SID;
	//To check the status of the system
	int status;





	//To store data
	double longitude=0,latitude=0,altitude=0,heading=0,velocity=0,acceleration=0,brake=0;
	double ind_left_status=0,ind_right_status=0;
	unsigned int ind_left=0,ind_right=0,ind_double=0,motor_error=0,system_error=0,battery_error=0;
	unsigned int longitude_CAN=0,latitude_CAN=0,altitude_CAN=0,heading_CAN=0,velocity_CAN=0,acceleration_CAN=0,brake_CAN=0;
	unsigned int year=0, month=0, date_of_month=0,hour=0, minute=0,second=0, millisecond=0;

	public:
	Inno_Can_Sender()
	:Node("inno_can_send")
	{
		// Here the callback function has not been declared, it needs to be bind before declare
		msg_rec=this->create_subscription<nav_msgs::msg::Odometry>("carmessage",10,std::bind(&Inno_Can_Sender::Callback, this, _1));
		
	}
	
	virtual ~Inno_Can_Sender(){}

	public:


	void Callback(const nav_msgs::msg::Odometry::SharedPtr msg)
	{
		//Get current time
		time_t now =time(0);
		tm *ltm = localtime(&now);
		std::chrono::time_point<std::chrono::system_clock,std::chrono::milliseconds> tp = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
		auto tmp=std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch());
		std::time_t timestamp = tmp.count();

		// cout << "年: "<< 1900 + ltm->tm_year << endl;
		// cout << "月: "<< 1 + ltm->tm_mon<< endl;
		// cout << "日: "<<  ltm->tm_mday << endl;
		// cout << "时间: "<< ltm->tm_hour << ":";
		// cout << ltm->tm_min << ":";
		// cout << ltm->tm_sec << endl;
		year=(unsigned int)ltm->tm_year;
		month=(unsigned int)(ltm->tm_mon+1);
		date_of_month=(unsigned int)ltm->tm_mday;
		hour=(unsigned int)ltm->tm_hour;
		minute=(unsigned int)ltm->tm_min;
		second=(unsigned int)ltm->tm_sec;
		millisecond=(unsigned int)(timestamp%10000);
		
		
		
		
		//Get data from ros msg

		nav_msgs::msg::Odometry info=*msg;
		longitude=info.pose.pose.position.x;
		latitude=info.pose.pose.position.y;
		altitude=info.pose.pose.position.z;
		heading=info.pose.pose.orientation.z;

		velocity=info.twist.twist.linear.x;
		acceleration=info.twist.twist.linear.y;
		brake=info.twist.twist.linear.z;
		ind_left_status=info.pose.pose.orientation.x;
		ind_right_status=info.pose.pose.orientation.y;
		if (ind_left_status>0)ind_left=1;
		else ind_left=0;
		if (ind_right_status>0)ind_right=1;
		else ind_right=0;
		if (ind_left_status>0&&ind_right_status>0)ind_double=1;
		else ind_double=0;

		
		status=0;
		//Transform data with factor and offset to the int format
		velocity_CAN=(unsigned int)(velocity*200+20000);//factor is 0.005 offset is -100
		acceleration_CAN=(unsigned int)(acceleration*200+20000);//factor is 0.005 offset is -100
		brake_CAN=(unsigned int)((brake)*50);//factor is 0.02 offset is 0

		longitude_CAN=(unsigned int)((longitude)*(1e7)+1.8e9);//factor is 1e-7 offset is -180
		latitude_CAN=(unsigned int)((latitude)*(1e7)+1.8e9);//factor is 1e-7 offset is -180
		heading_CAN=(unsigned int)((heading)*(1e7));//factor is 1e-7 offset is 0
		altitude_CAN=(unsigned int)((altitude)*(5e6)+5e8);//factor is 2e-7 offset is -100

		//Ready to write into CAN message
		CAN_FRAME_INFO info_1,info_2,info_3, info_4;

		//Write into frame 1
		info_1.CAN_port=can_id;
		info_1.rtr=EMUC_DIS_RTR;
		info_1.dlc=8;
		info_1.id_type=id_type;
		info_1.id=1;
		info_1.data[0]=(velocity_CAN>>8)&0x00ff;
		info_1.data[1]=velocity_CAN&0x00ff;
		
		info_1.data[2]=(acceleration_CAN>>8)&0x00ff;
		info_1.data[3]=acceleration_CAN&0x00ff;
		info_1.data[4]=(brake_CAN>>8)&0x00ff;
		info_1.data[5]=brake_CAN&0x00ff;
		info_1.data[6]=((ind_left<<4)|(ind_right<<2)|(ind_double))&0x00ff;
		info_1.data[7]=((motor_error<<4)|(system_error<<2)|(battery_error))&0x00ff;
		status+=EMUCSend(com_port_sel,&info_1);

		//Write into frame 2
		info_2.CAN_port=can_id;
		info_2.rtr=EMUC_DIS_RTR;
		info_2.dlc=8;
		info_2.id=2;
		info_2.id_type=id_type;
		info_2.data[0]=(longitude_CAN>>24)&0x00ff;
		info_2.data[1]=(longitude_CAN>>16)&0x00ff;
		info_2.data[2]=(longitude_CAN>>8)&0x00ff;
		info_2.data[3]=(longitude_CAN)&0x00ff;
		info_2.data[4]=(latitude_CAN>>24)&0x00ff;
		info_2.data[5]=(latitude_CAN>>16)&0x00ff;
		info_2.data[6]=(latitude_CAN>>8)&0x00ff;
		info_2.data[7]=(latitude_CAN)&0x00ff;
		//cout<<"current lat is "<<latitude<<" CAN value is "<<latitude_CAN<<" bin format is "<<(int)info_2.data[4]<<","<<(int)info_2.data[5]<<","<<(int)info_2.data[6]<<","<<(int)info_2.data[7]<<endl;
		status+=EMUCSend(com_port_sel,&info_2);


		//Write into frame 3
		info_3.CAN_port=can_id;
		info_3.rtr=EMUC_DIS_RTR;
		info_3.dlc=8;
		info_3.id=3;
		info_3.id_type=id_type;
		info_3.data[0]=(altitude_CAN>>24)&0x00ff;
		info_3.data[1]=(altitude_CAN>>16)&0x00ff;
		info_3.data[2]=(altitude_CAN>>8)&0x00ff;
		info_3.data[3]=(altitude_CAN)&0x00ff;
		info_3.data[4]=(heading_CAN>>24)&0x00ff;
		info_3.data[5]=(heading_CAN>>16)&0x00ff;
		info_3.data[6]=(heading_CAN>>8)&0x00ff;
		info_3.data[7]=(heading_CAN)&0x00ff;
		status+=EMUCSend(com_port_sel,&info_3);


		//Write into frame 4
		info_4.CAN_port=can_id;
		info_4.rtr=EMUC_DIS_RTR;
		info_4.dlc=8;
		info_4.id=4;
		info_4.id_type=id_type;
		info_4.data[0]=(year)&0x00ff;
		info_4.data[1]=(month)&0x00ff;
		info_4.data[2]=(date_of_month)&0x00ff;
		info_4.data[3]=(hour)&0x00ff;
		info_4.data[4]=(minute)&0x00ff;
		info_4.data[5]=(second)&0x00ff;
		info_4.data[6]=(millisecond>>8)&0x00ff;
		info_4.data[7]=(millisecond)&0x00ff;
		status+=EMUCSend(com_port_sel,&info_4);


		std::stringstream msg_info;
		if(status==0){
			msg_info<<"[Time: ""年: "<< 1900 + ltm->tm_year << "月: "<< 1 + ltm->tm_mon<< "日: "<<  ltm->tm_mday << "时间: "<< ltm->tm_hour << ":"<< ltm->tm_min << ":"<<ltm->tm_sec << "] "<<"Message sent successfully!";
			RCLCPP_INFO(inno_logger,msg_info.str());
		}
		else{
			msg_info<<"[Time: "<<this->get_clock()->now().nanoseconds()<<"] "<<"Message sent failed!";
			RCLCPP_WARN(inno_logger,msg_info.str());
		}
	}

	void Open_Channel()
	{
		status=EMUCOpenDevice(com_port_sel);
		if(status==0)
		{
			RCLCPP_INFO(inno_logger,"CAN opened successfully!");
		}
		else{
			RCLCPP_FATAL(inno_logger,"Cannot open CAN device, please check your connection!");
		}

		status=EMUCInitCAN(com_port_sel,1,1);
		if(status==0)
		{
			RCLCPP_INFO(inno_logger,"CAN activated successfully!");
		}
		else{
			RCLCPP_FATAL(inno_logger,"Cannot activate CAN device, please check your settings!");
		}

	}

	void Close_Channel()
	{
		status=EMUCCloseDevice(com_port_sel);
		if(status==0)
		{
			RCLCPP_INFO(inno_logger,"CAN closed successfully!");
		}
		else{
			RCLCPP_WARN(inno_logger,"Cannot close CAN device, please check your connection!");
		}
	}

	void get_local_mac(char *if_name){
		struct ifreq m_ifreq;
		int sock = 0;
		char mac[32] = " ";

		sock = socket(AF_INET,SOCK_STREAM,0);
		strcpy(m_ifreq.ifr_name,if_name);

		ioctl(sock,SIOCGIFHWADDR,&m_ifreq);
		int i = 0;
		for(i = 0; i < 6; i++){
			sprintf(mac+3*i, "%02X:", (unsigned char) m_ifreq.ifr_hwaddr.sa_data[i]);
		}
		mac[strlen(mac) - 1] = 0;
		// printf("MAC: %s\n", mac);
		std::string original_mac="20:46:A1:05:84:9A";
		std::string mac_str=mac;
		if(mac_str!=original_mac) 
		{
			RCLCPP_FATAL(inno_logger,"Incorrect device!");
			exit (0);
		}
		
	
	}



};

int main(int argc, char** argv)
{
	
	rclcpp::init(argc, argv);
	auto sender=std::make_shared<Inno_Can_Sender>();
	sender->get_local_mac((char*)"eno1");
	sender->Open_Channel();
	// ros::NodeHandle np("~");
	rclcpp::spin(sender);
	sender->Close_Channel();
	
	exit(0);
}