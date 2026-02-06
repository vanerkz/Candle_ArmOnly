
#include "../include/feetech_controls/feetech_controller.h"

namespace feetech_controller
{
        
	int FeetechController::init(int argc, char **argv){
         
	if(argc<2){
        ROS_INFO("argc error!");
        return 0;
	}
	std::cout<< "serial:"<<argv[1]<<std::endl;

        if(!sm.begin(115200, argv[1])){
        ROS_INFO("Failed to init smsbl motor!");
        return 0;
        }

	for(int i=0; i<7; i++){
        Speed[i] = 2024;
        ACC[i] = 100;
        }

        //sm.SyncWritePosEx(ID, 7, Position, Speed, ACC);
        }
    double Position_set[7]={0,0,0,0,0,0,0};
    double out;
    double anticw =25; // neg number = More cw, pos number = more anti cw(arm as 12 oclock)
    int cw = -15; // neg number = More cw, pos number = more anti cw (arm as 12 oclock)
     double FeetechController::readservo(int index)
     {
	if(sm.FeedBack(index)!=-1){
	Pos = sm.ReadPos(index);
	if(index==0 && Pos >2047)
	{
		Pos -= anticw;
	}
		if(index==0 && Pos <2047)
	{
		Pos -= cw;
	}
	out= ((-(Pos*0.0879335613)*(3.1415926536/180))+3.1415926536);
	return out;
	}
	else
        {
        out = Position_set[index];
	std::cout<< "read err ="<<std::endl;
        ROS_INFO("%d", index);
	return out;
	//sleep(1);
        }
      }

void FeetechController::set_joint(double newmsg, int index) {
int id =index;
Position[index] = ((-newmsg+3.1415926536)*(180/3.1415926536))/(0.0879335613);
if(index == 0 && Position[0]>2047)
{
Position[0] +=anticw;
}
if(index == 0 && Position[0]<2047)
{
Position[0] +=cw;
}
sm.WritePosEx(id, Position[index], Speed[index], ACC[index]);
}


}


