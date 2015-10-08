#include "ros/ros.h"
#include "mini/controlOutput.h"
#include "serialPort.h"

int fd = -1;
short steerOffset = -97;

void sendCommand(const mini::controlOutput::ConstPtr& msg)
{
  ROS_INFO("I heard: [%d] [%d]", msg->speed,msg->steer);
  char dataSend[7];
  dataSend[0] = 0xF8;
  dataSend[1] = 4;
  *(short*) &dataSend[2] = msg->steer+steerOffset;
  *(short*) &dataSend[4] = msg->speed;
  dataSend[6] = 0x8F;
  int ret = UART_Send(fd,dataSend,7);
  
}

int main(int argc, char **argv)
{
  fd = UART_Open(fd,"/dev/ttyS4");
  int ret = UART_Init(fd,115200,0,8,1,'N');

  ros::init(argc, argv, "controller");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("ctrmsg", 1, sendCommand);
  ros::spin();
  
  UART_Close(fd);
  return 0;
}
