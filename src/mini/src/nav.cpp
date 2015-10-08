#include "ros/ros.h"
#include "mini/naviLoc.h"
#include "serialPort.h"
#include "stdio.h"
#include <sys/time.h>  
#include <time.h> 
#include <tf/transform_broadcaster.h>

const double PI = 3.1415926535897932384626433832795;

short lastpulsenum = 0;
short lastimupulsenum = 0;
double lastx = 0;
double lasty = 0;
double lastori = 0;
const double distancePerPulse = 3.832916739004521e-5;
const double arcPerPulse = PI/720;
const int maxPulse = 30000;
const int maxImuPulse = 3600;

bool ifInit = false;

void SerialData2Msg(unsigned int timeStamp, char* rcv_buf,int data_len,mini::naviLoc& msg)
{
	//ROS_INFO("[%02X] [%02X]",rcv_buf[data_len-5],rcv_buf[data_len-4]);
	short sAngle = -(*(short*)(&rcv_buf[data_len-5]));
	short sPulseNum =  *(short*)(&rcv_buf[data_len-3]);
	if(ifInit==false)
	{
		ifInit = true;
		lastpulsenum = sPulseNum;
		lastimupulsenum = sAngle;
	}
	//ROS_INFO("[%d]",int(*(short*)(&rcv_buf[data_len-5])));
	short deltaIP = sAngle - lastimupulsenum;
	if(deltaIP > maxImuPulse/2)
	{
		deltaIP -= maxImuPulse;
	}
	else if(deltaIP < -maxImuPulse/2)
	{
		deltaIP += maxImuPulse;
	}
	lastimupulsenum = sAngle;
	double deltaOri = deltaIP*arcPerPulse;
	double aveori_y = lastori+deltaOri/2+PI/2;
	lastori += deltaOri;

	short deltaP = sPulseNum - lastpulsenum;
	if( deltaP > maxPulse/2 )
	{
		deltaP -= maxPulse;
	}
	else if(deltaP<-maxPulse/2)
	{
		deltaP += maxPulse;
	}
	double distance = deltaP * distancePerPulse;
	lastpulsenum = sPulseNum;

	lastx += cos(aveori_y)*distance;
	lasty += sin(aveori_y)*distance;

	msg.timestamp = timeStamp;
	msg.x = lastx;
	msg.y = lasty;
	msg.theta = lastori;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "nav");
	ros::NodeHandle n;
	//ros::Publisher chatter_pub = n.advertise<mini::naviLoc>("navmsg", 1);
	tf::TransformBroadcaster br;
	tf::Transform transform_v2c,transform_w2v;
	tf::Quaternion q(-sqrt(2)/2,0,0,sqrt(2)/2); //need calibration
	transform_v2c.setOrigin(tf::Vector3(-0.025,0.125,0.23)); //need calibration
	transform_v2c.setRotation(q);	
	transform_v2c = transform_v2c.inverse();

	int fd = -1;
	fd = UART_Open(fd,"/dev/ttyS4");
	UART_Init(fd,115200,0,8,1,'N');	

	int data_len = 10;
	char rcv_buf[100];
	//FILE* fp = fopen("/home/haha/catkin_ws/odom.txt","w");

	int ret;

	while (ros::ok())
	{
		mini::naviLoc msg;	
		struct timeval tv;	

		while(1)
		{
			//ROS_INFO(" ");
			ret = UART_Recv(fd, rcv_buf,1);
			//ROS_INFO("[%d]",ret);
			//usleep(10000);
			if(ret==1)
			{
				//ROS_INFO("[%02X]",rcv_buf[0]);
			}
			if(ret==1 && rcv_buf[0]==char(0xA2))
			{
				break;
			}
		}
		int counter = 1;
		while(counter<data_len)
		{
			//ROS_INFO(" ");
			ret = UART_Recv(fd, &rcv_buf[counter],1);
			//ROS_INFO("[%d]",ret);
			/*for(int i=0;i<ret;i++)
			{
				printf("%02X ",rcv_buf[i]);
			}
			printf("\n");*/
			if(ret==-1)
			{
				continue;
			}
			counter++;
		}
 		gettimeofday(&tv,0);
		if(rcv_buf[data_len-1]!=char(0x2A))
		{
			continue;
		}
		unsigned int timeStamp = tv.tv_sec%100000*1000+tv.tv_usec/1000;
		SerialData2Msg(timeStamp, rcv_buf, data_len, msg);
		
		ros::Time ros_time = ros::Time::now();
		transform_w2v.setOrigin(tf::Vector3(msg.x,msg.y,0.0)); 
		q.setRPY(0.0,0.0,msg.theta);
		transform_w2v.setRotation(q);
		transform_w2v = transform_w2v.inverse();
		
		br.sendTransform(tf::StampedTransform(transform_w2v, ros_time, "/base_frame","/odom_frame"));
		br.sendTransform(tf::StampedTransform(transform_v2c, ros_time, "/camera_rgb_optical_frame","/base_frame"));
    	//chatter_pub.publish(msg);
		ROS_INFO("[%d],[%f],[%f],[%f]",msg.timestamp,msg.x,msg.y,msg.theta);
		//fprintf(fp,"%d %f %f %f\n",msg.timestamp,msg.x,msg.y,msg.theta);
		
		//fflush(fp);
    	ros::spinOnce();
	}
	UART_Close(fd);
	//fclose(fp);
	return 0;
}

