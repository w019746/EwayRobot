

#include<stdio.h>
#include<stdlib.h>
#include<unistd.h>
#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h>
#include<termios.h>
#include<errno.h>
#include<string.h>

#define FALSE -1
#define TRUE 0

int UART_Open(int fd,char* port);
void UART_Close(int fd);
int UART_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity);
int UART_Init(int fd, int speed,int flow_ctrlint ,int databits,int stopbits,char parity);
int UART_Recv(int fd, char *rcv_buf,int data_len);
int UART_Send(int fd, char *send_buf,int data_len);

/*****************************************************************
* 名称： UART0_Open
* 功能： 打开串口并返回串口设备文件描述
* 入口参数： fd :文件描述符 port :串口号(ttyS0,ttyS1,ttyS2)
* 出口参数： 正确返回为1，错误返回为0
*****************************************************************/
int UART_Open(int fd,char* port)
{
    
  fd = open( port, O_RDWR|O_NDELAY);
  if (FALSE == fd){
    perror("Can't Open Serial Port");
      return(FALSE);
  }
	tcflush(fd, TCIOFLUSH);
	int n = fcntl(fd, F_GETFL, 0);
	fcntl(fd, F_SETFL, n & ~O_NDELAY);

  return fd;
}

void UART_Close(int fd)
{
  	close(fd);
}

/*******************************************************************
* 名称： UART0_Set
* 功能： 设置串口数据位，停止位和效验位
* 入口参数： fd 串口文件描述符
* speed 串口速度
* flow_ctrl 数据流控制
* databits 数据位 取值为 7 或者8
* stopbits 停止位 取值为 1 或者2
* parity 效验类型 取值为N,E,O,,S
*出口参数： 正确返回为1，错误返回为0
*******************************************************************/
int UART_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity)
{
    
	int speed_arr[] = { B38400, B19200, B9600, B4800, B2400, B1200, B300,
             B57600, B115200
       };
  int name_arr[] = {
       38400, 19200, 9600, 4800, 2400, 1200, 300, 57600,
             115200
       };
	struct termios options;

   /*tcgetattr(fd,&options)得到与fd指向对象的相关参数，并将它们保存于options,该函数,还可以测试配置是否正确，该串口是否可用等。若调用成功，函数返回值为0，若调用失败，函数返回值为1.
    */
  if(tcgetattr(fd,&options) < 0){
		perror("SetupSerial 1");
		return(FALSE);
  }
      
        //设置串口输入波特率和输出波特率
  for(int i = 0;i < sizeof(speed_arr) / sizeof(int);i++) {     
	  if (speed == name_arr[i]) {
	            cfsetispeed(&options, speed_arr[i]);
	            cfsetospeed(&options, speed_arr[i]);
	  }
  }    
	     //设置数据位
	if (databits == 7 && (parity=='M' || parity=='m' || parity == 'S' || parity == 's'))
	{
		databits = 8;
	}
  options.c_cflag &= ~CSIZE; //屏蔽其他标志位
  switch (databits){
    case 5 :
        options.c_cflag |= CS5;
        break;
    case 6    :
        options.c_cflag |= CS6;
        break;
    case 7    :
        options.c_cflag |= CS7;
        break;
    case 8:
        options.c_cflag |= CS8;
        break;
       default:
        fprintf(stderr,"Unsupported data size\n");
        return (FALSE);
  }
	//修改控制模式，保证程序不会占用串口        
	options.c_cflag |= CLOCAL;
	//修改控制模式，使得能够从串口中读取输入数据
	options.c_cflag |= CREAD;

	//parity
	options.c_cflag &= ~(PARENB | PARODD);
	if (parity == 'E' || parity == 'e')
	{
		options.c_cflag |= PARENB;
	}
	else if (parity == 'O' || parity == 'o')
	{
		options.c_cflag |= (PARENB | PARODD);
	}
	
   // 设置停止位
  switch (stopbits){
    case 1:
        options.c_cflag &= ~CSTOPB;
				break;
    case 2:
        options.c_cflag |= CSTOPB;
				break;
    default:
				fprintf(stderr,"Unsupported stop bits\n");
				return (FALSE);
  }
	options.c_iflag=IGNBRK;
	
	//software handshake
	options.c_iflag &= ~(IXON | IXOFF | IXANY);

	options.c_lflag=0;
	options.c_oflag=0;

	options.c_cc[VTIME]=1;
	options.c_cc[VMIN]=60;

	/*int mcs=0;
	ioctl(fd, TIOCMGET, &mcs);
	mcs |= TIOCM_RTS;
	ioctl(fd, TIOCMSET, &mcs);*/

	//hardware handshake
	options.c_cflag &= ~CRTSCTS;

  //设置等待时间和最小接收字符
  options.c_cc[VTIME] = 1; /* 读取一个字符等待1*(1/10)s */
  options.c_cc[VMIN] = 1; /* 读取字符的最少个数为1 */
 
  //激活配置 (将修改后的termios数据设置到串口中）
  if (tcsetattr(fd,TCSANOW,&options) < 0)
  {
     perror("com set error!/n");
     return (FALSE);
  }
  return (TRUE);
}


int UART_Init(int fd, int speed,int flow_ctrlint ,int databits,int stopbits,char parity)
{
     //设置串口数据帧格式
    if (FALSE == UART_Set(fd,speed,flow_ctrlint,databits,stopbits,parity)) {         
        return FALSE;
        } else {
           return TRUE;
       }
}



/*******************************************************************
* 名称： UART0_Recv
* 功能： 接收串口数据
* 入口参数： fd :文件描述符
* rcv_buf :接收串口中数据存入rcv_buf缓冲区中
* data_len :一帧数据的长度
* 出口参数： 正确返回为1，错误返回为0
*******************************************************************/
int UART_Recv(int fd, char *rcv_buf,int data_len)
{
    int len,fs_sel;
    fd_set fs_read;
    
    struct timeval time;
    
    FD_ZERO(&fs_read);
    FD_SET(fd,&fs_read);
    
    time.tv_sec = 100;
    time.tv_usec = 0;
    
    //使用select实现串口的多路通信
    fs_sel = select(fd+1,&fs_read,NULL,NULL,&time);
    if(fs_sel){
     len = read(fd,rcv_buf,data_len);    
     return len;
        } else {
        return FALSE;
    }    
}

/*******************************************************************
* 名称： UART0_Send
* 功能： 发送数据
* 入口参数： fd :文件描述符
* send_buf :存放串口发送数据
* data_len :一帧数据的个数
* 出口参数： 正确返回为1，错误返回为0
*******************************************************************/
int UART_Send(int fd, char *send_buf,int data_len)
{
    int ret;
    
    ret = write(fd,send_buf,data_len);
    if (data_len == ret ){    
     return ret;
    } else {
     tcflush(fd,TCOFLUSH);
     return FALSE;
        
    }
    
}




/*  int main(int argc, char **argv)
{
    int fd = FALSE;             
    int ret;                          
    char rcv_buf[512];
    int i;
    if(argc != 2){
     printf("Usage: %s /dev/ttySn \n",argv[0]);    
     return FALSE;    
    }
    fd = UART_Open(fd,argv[1]);
    if(FALSE == fd){    
     printf("open error\n");    
     exit(1);     
    }
    ret = UART_Init(fd,9600,0,8,1,'N');
    if (FALSE == ret){    
     printf("Set Port Error\n");    
     exit(1);
    }
    
    ret = UART_Send(fd,"*IDN?\n",6);
    if(FALSE == ret){
     printf("write error!\n");
     exit(1);
    }
    
    printf("command: %s\n","*IDN?");
    memset(rcv_buf,0,sizeof(rcv_buf));
    for(i=0;;i++)
    {
     ret = UART_Recv(fd, rcv_buf,512);    
         if( ret > 0){
         rcv_buf[ret]='\0';        
         printf("%s",rcv_buf);    
     } else {    
     printf("cannot receive data1\n");    
            break;
     }
     if('\n' == rcv_buf[ret-1])
         break;
    }
    UART_Close(fd);
    return 0;
}*/


