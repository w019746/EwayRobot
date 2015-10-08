#include "ros/ros.h"
#include "mini/naviLoc.h"
#include "mini/serialPort.h"
#include "stdio.h"
#include <sys/time.h>  
#include <time.h> 
#include <QSerialPort>

const double PI = 3.1415926535897932384626433832795;

short lastpulsenum = 0;
double lastx = 0;
double lasty = 0;
double lastori = 0;
const double distancePerPulse = 3.832916739004521e-5;//need calibration
const int maxPulse = 30000;

double oriOffset = 0;
bool ifInit = false;

QSerialPort a; 

void SerialData2Msg(unsigned int timeStamp, char* rcv_buf,int data_len,mini::naviLoc& msg)
{
	//ROS_INFO("[%02X] [%02X]",rcv_buf[data_len-5],rcv_buf[data_len-4]);
	short sAngle = -(*(unsigned short*)(&rcv_buf[data_len-5]));
	double tmpori=PI * double(sAngle) /1800.0 + PI/2;
	short sPulseNum =  *(unsigned short*)(&rcv_buf[data_len-3]);
	if(ifInit==false)
	{
		ifInit = true;
		lastpulsenum = sPulseNum;
		oriOffset = tmpori;
	}
	tmpori -= oriOffset;
	//ROS_INFO("[%d]",int(*(unsigned short*)(&rcv_buf[data_len-5])));
	double aveori_y;
	double deltaOri = tmpori-lastori;
	if(deltaOri>PI)
	{
		deltaOri -= 2*PI;
	}
	else if(deltaOri<-PI)
	{
		deltaOri += 2*PI;
	}
	aveori_y = lastori+deltaOri/2+PI/2;
	lastori=tmpori;

	short deltaP = sPulseNum - lastpulsenum;
	if( deltaP > maxPulse/2 )
	{
		deltaP -= maxPulse;
	}
	else if(deltaP<-maxPulse/2)
	{
		deltaP += maxPulse;
	}
	double distance = (deltaP) * distancePerPulse;
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
  ros::Publisher chatter_pub = n.advertise<mini::naviLoc>("navmsg", 1);
	
	QSerialPort sp; 
	sp.setPortName("/dev/ttyS4");
	sp.setBaudRate(QSerialPort::Baud115200);
	sp.setDataBits(QSerialPort::Data8);
	sp.setParity(QSerialPort::NoParity);
	sp.setStopBits(QSerialPort::OneStop);
	sp.open(QIODevice::ReadWrite);

	int data_len = 10;
	char rcv_buf[100];
	FILE* fp = fopen("/home/haha/catkin_ws/odom.txt","w");

	int ret;

  while (ros::ok())
  {
		mini::naviLoc msg;	
		struct timeval tv;	
		int ret;

		while(1)
		{
			//ROS_INFO(" ");
			ret = sp.read(rcv_buf,1);
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
			ret = sp.read(&rcv_buf[counter],1);
			//ROS_INFO("[%d]",ret);
			if(ret!=1)
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

    chatter_pub.publish(msg);
		//ROS_INFO("[%d],[%f],[%f],[%f]",msg.timestamp,msg.x,msg.y,msg.theta);
		//fprintf(fp,"%d %f %f %f\n",msg.timestamp,msg.x,msg.y,msg.theta);
		//fflush(fp);
    ros::spinOnce();
		//usleep(20000);
  }
	sp.close();
	fclose(fp);
  return 0;
}

