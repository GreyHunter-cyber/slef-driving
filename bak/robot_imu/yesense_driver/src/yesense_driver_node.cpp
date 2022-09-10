/***********************************************************************************************
yesense_node
功能：YESENSE IMU 数据解析
作者：李星, 朱仁杰
创建日期：2019/10/28
修改日期：NA

                        buff:数据缓存；
			acc ：加速度；    m/ss
			angularspeed：角速度； deg/s
			magnetic：磁场归一化值；
			magneticstrength：磁场强度；
			eulerangle：欧拉角；
			quaternion：四元数；
			Sampletimestamp：采样时间戳；	
			timesynchronization：同步输出时间戳；
************************************************************************************************/

#define bufferlength 256
#define framelength	107
#define PI 3.141592653
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "uart.h"
#include <string>

int main(int argc, char** argv){
  unsigned char buff[256];
  UART cuart;
  int readLength = 0;

  ros::init(argc,argv,"yesense_driver");
  ros::NodeHandle n;
  ros::NodeHandle n_private("~");
  int baud_rate = 406800;
  std::string device = "/dev/ttyUSB0";
  std::string imu_topic = "/imu/data";
  n_private.param<int>("baud_rate", baud_rate, 460800);
  n_private.param<std::string>("device", device, "/dev/ttyUSB0");
  n_private.param<std::string>("imu_topic", imu_topic, "/imu/data");

  ros::Publisher IMU_pub = n.advertise<sensor_msgs::Imu>(imu_topic, 100);
  while(!cuart.OpenUart(device.c_str(), baud_rate))
  {
      printf("nav com open failed!\n");
      cuart.CloseUart();
  }

  ros::Rate loop_rate(200);
  while (ros::ok())
  {
    readLength = 0;
    memset(buff, 0, sizeof(unsigned char) * bufferlength);
    readLength = cuart.ReadUart(buff, bufferlength);
    int i = 0;
  	unsigned char check1 = 0;
    unsigned char check2 = 0;
    int messagelength = 0;
	unsigned int seq = 0;
    while(i+messagelength+6 < readLength && ros::ok())
	{
	  //printf("%X ", buff[i]);
  	  if(buff[i] == 0x59) //查询帧头
  	  {
  	    if(buff[i+1]==0x53)
  	    {	
  	    	messagelength =	buff[i+4];

			//printf("readlength: %d, length: %d, i: %d\n",readLength, messagelength, i);
  	    	for(int a = i+2;a<i+messagelength+5;a++) //计算校验和
  	    	{
  	    	  check1 = check1+buff[a];
  	    	  check2 = check2+check1;
  	    	}
			//printf("%X, %X, %X, %X\n", check1, check2, buff[i+messagelength+5], buff[i+messagelength+6]);
			// for(int j = i; j<= i+messagelength+6; ++j){
			// 	printf("%X ", buff[j]);
			// }
			//printf("\n");
  	    	if((check1==buff[i+messagelength+5])&&(check2==buff[i+messagelength+6])) //判断数据校验
  	    	{
			  //printf("pub msg!\n");
			  sensor_msgs::Imu msg;
              msg.header.stamp = ros::Time::now();
              msg.header.seq = seq++;
              msg.header.frame_id = "imu";
  	    	  msg.linear_acceleration.x = ((buff[i+7])|(buff[i+8]<<8)|(buff[i+9]<<16)|(buff[i+10]<<24))*0.000001;
  	    	  msg.linear_acceleration.y = ((buff[i+11])|(buff[i+12]<<8)|(buff[i+13]<<16)|(buff[i+14]<<24))*0.000001;
			  msg.linear_acceleration.z = ((buff[i+15])|(buff[i+16]<<8)|(buff[i+17]<<16)|(buff[i+18]<<24))*0.000001;
  	    	  msg.angular_velocity.x = ((buff[i+21])|(buff[i+22]<<8)|(buff[i+23]<<16)|(buff[i+24]<<24))*PI*0.000001/180.0;
  	    	  msg.angular_velocity.y = ((buff[i+25])|(buff[i+26]<<8)|(buff[i+27]<<16)|(buff[i+28]<<24))*PI*0.000001/180.0;
  	    	  msg.angular_velocity.z = ((buff[i+29])|(buff[i+30]<<8)|(buff[i+31]<<16)|(buff[i+32]<<24))*PI*0.000001/180.0;
  	    	//   magnetic[0] = 0;
  	    	//   magnetic[1] = 0;
  	    	//   magnetic[2] = 0;
  	    	//   magnetic[0] = ((float *)((buff[i+35])|(buff[i+36]<<8)|(buff[i+37]<<16)|(buff[i+38]<<24)))*0.000001;
  	    	//   magnetic[1] = ((float *)((buff[i+39])|(buff[i+40]<<8)|(buff[i+41]<<16)|(buff[i+42]<<24)))*0.000001;
  	    	//   magnetic[2] = ((float *)((buff[i+43])|(buff[i+44]<<8)|(buff[i+45]<<16)|(buff[i+46]<<24)))*0.000001;
  	    	//   magneticstrength[0] = 0;
  	    	//   magneticstrength[1] = 0;
  	    	//   magneticstrength[2] = 0;
  	    	//   magneticstrength[0] = ((float *)((buff[i+49])|(buff[i+50]<<8)|(buff[i+51]<<16)|(buff[i+52]<<24)))*0.001;
  	    	//   magneticstrength[1] = ((float *)((buff[i+53])|(buff[i+54]<<8)|(buff[i+55]<<16)|(buff[i+56]<<24)))*0.001;
  	    	//   magneticstrength[2] = ((float *)((buff[i+57])|(buff[i+58]<<8)|(buff[i+59]<<16)|(buff[i+60]<<24)))*0.001;
  	    	//   eulerangle[0] = 0;
  	    	//   eulerangle[1] = 0;
  	    	//   eulerangle[2] = 0;
  	    	//   eulerangle[0] = ((float *)((buff[i+63])|(buff[i+64]<<8)|(buff[i+65]<<16)|(buff[i+66]<<24)))*0.000001;
  	    	//   eulerangle[1] = ((float *)((buff[i+67])|(buff[i+68]<<8)|(buff[i+69]<<16)|(buff[i+70]<<24)))*0.000001;
  	    	//   eulerangle[2] = ((float *)((buff[i+71])|(buff[i+72]<<8)|(buff[i+73]<<16)|(buff[i+74]<<24)))*0.000001;
  	    	  msg.orientation.w = ((buff[i+77])|(buff[i+78]<<8)|(buff[i+79]<<16)|(buff[i+80]<<24))*0.000001;
  	    	  msg.orientation.x = ((buff[i+81])|(buff[i+82]<<8)|(buff[i+83]<<16)|(buff[i+84]<<24))*0.000001;
  	    	  msg.orientation.y = ((buff[i+85])|(buff[i+86]<<8)|(buff[i+87]<<16)|(buff[i+88]<<24))*0.000001;
  	    	  msg.orientation.z = ((buff[i+89])|(buff[i+90]<<8)|(buff[i+91]<<16)|(buff[i+92]<<24))*0.000001;
  	    	//   *Sampletimestamp = (int *)((buff[i+95])|(buff[i+96]<<8)|(buff[i+97]<<16)|(buff[i+98]<<24));
  	    	//   *timesynchronization =  (int *)((buff[i+101])|(buff[i+102]<<8)|(buff[i+103]<<16)|(buff[i+104]<<24));
			  msg.orientation_covariance[0] = msg.orientation_covariance[4] = 0.0;
			  msg.orientation_covariance[8] = 0.0;

			  msg.angular_velocity_covariance[0] = msg.angular_velocity_covariance[4] 
			  = msg.angular_velocity_covariance[8] = 0.0;

			  msg.linear_acceleration_covariance[0] = msg.linear_acceleration_covariance[4] 
			  = msg.linear_acceleration_covariance[8] = 0.098;

  	    	  IMU_pub.publish(msg); 
  	    	}
			i += messagelength+7; //后移一帧数据
  	      }
		  else{
			++i;
		  }
  	    }
		else{
		  ++i;	
		}  
      }
	  //printf("\n");
	  loop_rate.sleep();
  }  
  cuart.CloseUart();
  return 0;   
}

