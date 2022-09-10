
/* 
    by luohai wangpeng 
    2021.03.11
*/


#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "uart.h"
#include <string>
#define bufferlength 256
ros::Publisher IMU_pub;
enum YesenseMsg {
    YESENSE_TEMPERATURE = 0x01,
    YESENSE_ACCEL = 0x10,
    YESENSE_ANGLUAR = 0x20,
    YESENSE_EULER = 0x40,
    YESENSE_QUATERNION = 0x41,
    YESENSE_UTCTIME = 0x50,
    YESENSE_SAMPLESTAMP = 0x51,
    YESENSE_SYNCSTAMP = 0x52,
    YESENSE_LOCATION = 0x68,
    YESENSE_SPEED = 0x70,
};

struct yesense_temperature {
    int16_t temp_e2;
};
struct yesense_accel {
    int32_t ax_e6;
    int32_t ay_e6;
    int32_t az_e6;
};
struct yesense_angular {
    int32_t wx_e6;
    int32_t wy_e6;
    int32_t wz_e6;
};
struct yesense_euler {
    int32_t pitch_e6;
    int32_t roll_e6;
    int32_t yaw_e6;
};
struct yesense_quaternion {
    int32_t q_e6[4];
};
struct yesense_utctime {
    uint32_t ms;
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  min;
    uint8_t  sec;
};
struct yesense_stamp {
    uint32_t us;
};
struct yesense_location {
    int64_t  lat_e10;
    int64_t  long_e10;
    int32_t  alt_e3;
};
struct yesense_speed {
    int32_t ve_e3;
    int32_t vn_e3;
    int32_t vu_e3;
};

// 一包数据解析结果
struct yesense_msg {
    unsigned tid;
    struct yesense_temperature *temperature;
    struct yesense_accel *accel;
    struct yesense_angular *angular;
    struct yesense_euler *euler;
    struct yesense_quaternion *queaternion;
    struct yesense_utctime *utc;
    struct yesense_stamp *samplestamp;
    struct yesense_stamp *syncstamp;
    struct yesense_location *location;
    struct yesense_speed *speed;
};

static void yesense_checksum(const unsigned char *buf, size_t sz, unsigned char *ck1, unsigned char *ck2)
{
    unsigned char c1 = 0, c2 = 0;
    for (int i = 0; i < sz; i++) {
        c1 += buf[i];
        c2 += c1;
    }
    *ck1 = c1;
    *ck2 = c2;
}

static void yesense_msg_parse(const unsigned char *buf, size_t sz, struct yesense_msg *msg)
{
    msg->accel = NULL;
    msg->angular = NULL;
    msg->euler = NULL;
    msg->location = NULL;
    msg->queaternion = NULL;
    msg->samplestamp = NULL;
    msg->speed = NULL;
    msg->syncstamp = NULL;
    msg->temperature = NULL;
    msg->utc = NULL;
    // 利用静态存储解析数据，所以解析部分并不线程安全，也不需要线程安全
    static struct yesense_temperature y_temperature;
    static struct yesense_accel       y_accel;
    static struct yesense_angular     y_angular;
    static struct yesense_euler       y_euler;
    static struct yesense_quaternion  y_quaternion;
    static struct yesense_utctime     y_utctime;
    static struct yesense_stamp       y_samplestamp;
    static struct yesense_stamp       y_syncstamp;
    static struct yesense_location    y_location;
    static struct yesense_speed       y_speed;
    ROS_INFO("TID=%u, PAYLOAD=%u", msg->tid, sz);
    int idx = 0;
    while (idx < sz) {
        int remain = sz - idx;
        if (remain < 2) {
            ROS_ERROR("unexpected end of message");
            break;
        }
        int id = buf[idx];
        int len = buf[idx + 1];
        const unsigned char *p = &buf[idx + 2];
        if (remain < (2 + len)) {
            ROS_ERROR("unexpected end of message");
            break;
        }
        switch (id) {
        case YESENSE_TEMPERATURE:
            assert(len == sizeof(y_temperature) && "bad temperature msg size");
            memcpy(&y_temperature, p, len);
            msg->temperature = &y_temperature;
            break;
        case YESENSE_ACCEL:
            assert(len == sizeof(y_accel) && "bad accel msg size");
            memcpy(&y_accel, p, len);
            msg->accel = &y_accel;
            break;
        case YESENSE_ANGLUAR:
            assert(len == sizeof(y_angular) && "bad angular msg size");
            memcpy(&y_angular, p, len);
            msg->angular = &y_angular;
            break;
        case YESENSE_EULER:
            assert(len == sizeof(y_euler) && "bad euler msg size");
            memcpy(&y_euler, p, len);
            msg->euler = &y_euler;
            break;
        case YESENSE_LOCATION:
            assert(len == sizeof(y_location) && "bad location msg size");
            memcpy(&y_location, p, len);
            msg->location = &y_location;
            break;
        case YESENSE_QUATERNION:
            assert(len == sizeof(y_quaternion) && "bad quaternion msg size");
            memcpy(&y_quaternion, p, len);
            msg->queaternion = &y_quaternion;
            break;
        case YESENSE_SAMPLESTAMP:
            assert(len == sizeof(y_samplestamp) && "bad samplestamp msg size");
            memcpy(&y_samplestamp, p, len);
            msg->samplestamp = &y_samplestamp;
            break;
        case YESENSE_SPEED:
            assert(len == sizeof(y_speed) && "bad speed msg size");
            memcpy(&y_speed, p, len);
            msg->speed = &y_speed;
            break;
        case YESENSE_SYNCSTAMP:
            assert(len == sizeof(y_syncstamp) && "bad syncstamp msg size");
            memcpy(&y_syncstamp, p, len);
            msg->syncstamp = &y_syncstamp;
            break;
        case YESENSE_UTCTIME:
            assert(len == sizeof(y_utctime) && "bad utctime msg size");
            memcpy(&y_utctime, p, len);
            msg->utc = &y_utctime;
            break;
	case 0x30:
	case 0x31:
	    break;
        default:
            ROS_INFO("unsupported msg id=0x%02X len=%d", id, len);
            break;
        }
        idx += len + 2;
    }
}

size_t yesense_process(const unsigned char *buf, size_t len)
{
    // 小于 7 字节无法解析出完整数据
    if (len < 7) {
        return len;
    }
    // 检查头，失败则从下一字节开始
    if (buf[0] != 0x59 || buf[1] != 0x53) {
        return len - 1;
    }
    // 检查长度
    unsigned mlen = buf[4];
    // 总长度是否完成一个包
    if (len < (7 + mlen)) {
        return len;
    }
    // 计算校验值，从 TID 到 PAYLOAD 结束
    unsigned char c1, c2;
    yesense_checksum(buf + 2, 3 + mlen, &c1, &c2);
    if (c1 != buf[5 + mlen] || c2 != buf[6 + mlen]) {
        ROS_ERROR("checksum failed, expect %02x, %02x actual %02x, %02x", c1, c2, buf[5 + mlen], buf[6 + mlen]);
        return len - (7 + mlen);
    }
    uint16_t tid = buf[2] + (buf[3] << 8);
    struct yesense_msg y_msg = {0};
    y_msg.tid = tid;

    yesense_msg_parse(buf + 5, mlen, &y_msg);

    //TODO:time
    struct tm utc = {0};
    utc.tm_year = y_msg.utc->year + 2000 - 1900;
    utc.tm_mon = y_msg.utc->month - 1;
    utc.tm_mday = y_msg.utc->day;
    utc.tm_hour = y_msg.utc->hour;
    utc.tm_min = y_msg.utc->min;
    utc.tm_sec = y_msg.utc->sec;
    time_t t = mktime(&utc) - __timezone;
    ros::Time imuT;
    imuT.fromNSec((t * 1000L + y_msg.utc->ms)*1000);
    //TODO:
    sensor_msgs::Imu imumsg;
    imumsg.header.frame_id = "imu";
    imumsg.header.stamp = imuT;
    imumsg.linear_acceleration.x = y_msg.accel->ax_e6*1e-6;
    imumsg.linear_acceleration.y = y_msg.accel->ay_e6*1e-6;
    imumsg.linear_acceleration.z = y_msg.accel->az_e6*1e-6;
    imumsg.angular_velocity.x = y_msg.angular->wx_e6*1e-6;
    imumsg.angular_velocity.y = y_msg.angular->wy_e6*1e-6;
    imumsg.angular_velocity.z = y_msg.angular->wz_e6*1e-6;
    imumsg.orientation.w = y_msg.queaternion->q_e6[0]*1e-6;
    imumsg.orientation.x = y_msg.queaternion->q_e6[1]*1e-6;
    imumsg.orientation.y = y_msg.queaternion->q_e6[2]*1e-6;
    imumsg.orientation.z = y_msg.queaternion->q_e6[3]*1e-6;
    IMU_pub.publish(imumsg);
    return len;
}

int main(int argc, char** argv){
  unsigned char buff[256];
  UART cuart;
  size_t readLength = 0;

  ros::init(argc,argv,"yesense_driver");
  ros::NodeHandle n;
  ros::NodeHandle n_private("~");
  int baud_rate = 406800;
  std::string device = "/dev/ttyUSB0";
  std::string imu_topic = "/imu/data";
  n_private.param<int>("baud_rate", baud_rate, 460800);
  n_private.param<std::string>("device", device, "/dev/ttyUSB0");
  n_private.param<std::string>("imu_topic", imu_topic, "/imu/data");

  IMU_pub = n.advertise<sensor_msgs::Imu>(imu_topic, 100);
  while(!cuart.OpenUart(device.c_str(), baud_rate))
  {
      printf("nav com open failed!\n");
      cuart.CloseUart();
  }

  ros::Rate loop_rate(200);
  while(ros::ok())
  {
    readLength = 0;
    memset(buff, 0, sizeof(unsigned char) * bufferlength);
    readLength = cuart.ReadUart(buff, bufferlength);
    yesense_process(buff, readLength);
    ros::spinOnce();
  }
}
