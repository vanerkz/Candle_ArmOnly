#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <stdio.h>
#include <string>  
#include <iostream>  
#include "../include/feetech_controls/feetech_controller.h"
#include "feetech_controls/torqcontrol.h"
#include "feetechlib/SCServo.h"
#include "std_srvs/Empty.h"

#include "feetech_controls/jointcontrols.h"
#include "feetech_controls/jointfeedback.h"
#include "feetech_controls/readjointvalues.h"

class myClass{ 
    public:
     void checkmain(char*);
void readservo(int, int);
    bool torqcontrolfunc();
    bool getjointfunc();

    myClass() {
    service= n.advertiseService("torqcontrol", &myClass::torqcontrolfunc,this);
    service2= n.advertiseService("getjointvalues", &myClass::getjointfunc,this);
    joint_pub = n.advertise<feetech_controls::jointfeedback>("joint_output", 1);
    feetech_listener = n.subscribe("joint_input", 1,&myClass::chatterCallback,this);
    feetech_listener_sync = n.subscribe("joint_input_sync", 1,&myClass::chatterCallback2,this);
    client = n.serviceClient<std_srvs::Empty>("/shutdown");
    if(!sm.begin(115200, "/dev/feetech")){
        ROS_INFO("Failed to init smsbl motor!");
	}
       
    }

    protected:
    SMSBL sm;
    ros::ServiceServer service;
    ros::ServiceServer service2;
    ros::ServiceClient client;
    ros::NodeHandle n;
    ros::Subscriber feetech_listener;
    ros::Subscriber feetech_listener_sync;
    ros::Publisher joint_pub;
    feetech_controls::jointfeedback readjoints;
    int move[10] = {0,0,0,0,0,0, 0};
    bool condmove =true;
    int error =0;
	u8 ID[7] = {0,1,2,3,4,5, 6};
	s16 Position[7]={2047, 2047,2047,2047,2047, 2047, 2047};
	u16 Speed[7] = {500, 500,500,500,500, 500, 500};
	u8 ACC[7] = {100, 100, 100, 100 , 100, 100, 100};
	int Pos;

void chatterCallback(const feetech_controls::jointcontrols msg){
double calvalue = (-msg.jointvalues+3.14)*(180/3.14)/(0.08791);
sm.WritePosEx(msg.id, calvalue, Speed[msg.id], ACC[msg.id]);
ROS_INFO("Servo Data Received");
}

void chatterCallback2(const feetech_controls::jointfeedback &msg){
s16 Positionarm[7]={2047, 2047,2047,2047,2047, 2047, 2047};
Positionarm[0] = (-msg.jointvalues[0]+3.14)*(180/3.14)/(0.08791);
Positionarm[1] = (-msg.jointvalues[1]+3.14)*(180/3.14)/(0.08791);
Positionarm[2] = (-msg.jointvalues[2]+3.14)*(180/3.14)/(0.08791);
Positionarm[3] = (-msg.jointvalues[3]+3.14)*(180/3.14)/(0.08791);
Positionarm[4] = (-msg.jointvalues[4]+3.14)*(180/3.14)/(0.08791);
Positionarm[5] = (-msg.jointvalues[5]+3.14)*(180/3.14)/(0.08791);
Positionarm[6] = (-msg.jointvalues[6]+3.14)*(180/3.14)/(0.08791);
u8 IDarm[7] = {0,1,2,3,4,5, 6};
u16 Speed[7] = {500, 500,500,500,500, 500, 500};
u8 ACC[7] = {100, 100, 100, 100 , 100, 100, 100};
sm.SyncWritePosEx(IDarm, 7, Position, Speed, ACC);


}

bool torqcontrolfunc(feetech_controls::torqcontrol::Request &req, feetech_controls::torqcontrol::Response &res)
{
// Call the /shutdown service after processing torque control
    std_srvs::Empty srv;
    if (ros::service::call("/shutdown", srv))
    {
        ROS_INFO("Shutdown service called successfully.");
    }
    else
    {
        ROS_ERROR("Failed to call shutdown service.");
    }

    // Loop through each ID to check and enable/disable torque
    for (int i = 0; i < 7; i++)
    {

        if (req.idrec[i] == 0)
        {
            sm.EnableTorque(i, 0);
            ROS_INFO("disable %d", i);
        }
        else
        {
            sm.EnableTorque(i, 1);
            ROS_INFO("enable %d", i);
        }
    }


}



bool getjointfunc(feetech_controls::readjointvalues::Request  &req, feetech_controls::readjointvalues::Response &res)
{;
condmove= true;
while(condmove)
{
        res.jointreadvalues.clear();
	int checkcount =0;
	int size = req.idend -req.idstart +1;
	for(int i=req.idstart;i<=req.idend;i++)
	{
		
		if(sm.FeedBack(i)!=-1){
		Pos = sm.ReadPos(i);
		double out= ((-(Pos*0.08791)*(6.283185307/360))+3.14);
		res.jointreadvalues.push_back(out);
		ROS_INFO("%d:%f",i,out);
		}
		if(sm.ReadMove(i) ==0)
		{
		   checkcount++;
		}
        }	
	if (checkcount ==size) condmove=false;
}
	return true;
}

};

void myClass::checkmain(char* argv)
{
ros::Rate loop_rate(10);
	while (n.ok())
	{
	ros::spinOnce();
	
	loop_rate.sleep();
	
	}
}



int main (int argc, char **argv)
{
  ros::init(argc, argv, "torqcontrolnode");
  myClass object;
  object.checkmain("/dev/feetech");
  ros::spin();
}
