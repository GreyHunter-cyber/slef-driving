// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

#include "utilityLidar.h"
// #include <integrationBase.h>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <mutex>
using namespace Eigen;
Eigen::Vector3d gw(0,0,-9.81);
    ofstream OpenFile;
class TransformFusion {
 private:
  ros::NodeHandle nh;

  ros::Publisher pubLaserOdometry2;
  ros::Subscriber subLaserOdometry;
  ros::Subscriber subOdomAftMapped;
  ros::Subscriber subImu;
  ros::Subscriber reset_point_sub;

  nav_msgs::Odometry laserOdometry2;
  tf::StampedTransform laserOdometryTrans2;
  tf::TransformBroadcaster tfBroadcaster2;

  tf::StampedTransform map_2_camera_init_Trans;
  tf::TransformBroadcaster tfBroadcasterMap2CameraInit;

  tf::StampedTransform camera_2_base_link_Trans;
  tf::TransformBroadcaster tfBroadcasterCamera2Baselink;

  float transformSum[6];
  float transformIncre[6];
  float transformMapped[6];
  float transformBefMapped[6];
  float transformAftMapped[6];

  std_msgs::Header currentHeader;


  //后添加的从这开始
    bool first_imu = false;
    bool laserodomflag=false;
    ros::Publisher pubLaserOdometry3;
    nav_msgs::Odometry laserOdometry3;
    tf::StampedTransform laserOdometryTrans3;
    tf::TransformBroadcaster tfBroadcaster3;
    double first_yaw,first_roll,first_pitch;
    double lastVx,lastVy,lastVz;

    int imu_cnt = 0;
    double lastImuTime,lastlaserTime;
    // Vector3d Pwb=Eigen::Vector3d(0, 0, 0);             // position :    from  imu measurements
    // Quaterniond Qwb=Eigen::Quaterniond::Identity();            // quaterniond:  from imu measurements
    // Vector3d Vw;          // velocity  :   from imu measurements
    // Vector3d gw;    // ENU frame

Eigen::Quaterniond Qwb;
Eigen::Vector3d Vw,Pwb, last_gyro, last_acc, linearized_ba, linearized_bg;
Eigen::Matrix<double, 15, 15> jacobian;



    // StatePredictor* filter_;  // Kalman filter pointer
    // GlobalState globalState_;

        //初始位姿
    float x_op, y_op, z_op, roll_op, pitch_op, yaw_op;
    string fileDirectory;
    std::mutex init_mutex, mux, vmux;
    bool restart;
    bool USING_IMU_ODOM = true;
    int count = 10;
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // enum FusionStatus {
    //     STATUS_INIT = 0,
    //     STATUS_FIRST_SCAN = 1,
    //     STATUS_SECOND_SCAN = 2,
    //     STATUS_RUNNING = 3,
    //     STATUS_RESET = 4,
    // } status_;
  TransformFusion() {


        //private_nh.param<float>("car_front",car_front,0.55);
       // private_nh.param<float>("car_up",car_up,0.65);
        nh.param<float>("/x_op",x_op,0.0);
        nh.param<float>("/y_op",y_op,0.0);
        nh.param<float>("/z_op",z_op,0.0);
        nh.param<float>("/roll_op",roll_op,0.0);
        nh.param<float>("/pitch_op",pitch_op,0.0);
        nh.param<float>("/yaw_op",yaw_op,0.0);
        nh.param<float>("/yaw_op",yaw_op,0.0);
        nh.param<float>("/yaw_op",yaw_op,0.0);
        std::cout << "\n-----------------------initial pose:------------------\n"
                  << x_op << ", " << y_op << ", " << z_op << ", " << yaw_op << std::endl;
        nh.param<bool>("/USING_IMU_ODOM",USING_IMU_ODOM,true);

    pubLaserOdometry2 =
        nh.advertise<nav_msgs::Odometry>("/integrated_to_init", 5);
    subLaserOdometry = nh.subscribe<nav_msgs::Odometry>(
        "/laser_odom_to_init", 5, &TransformFusion::laserOdometryHandler, this);
    subOdomAftMapped = nh.subscribe<nav_msgs::Odometry>(
        "/aft_mapped_to_init", 5, &TransformFusion::odomAftMappedHandler, this);

    subImu = nh.subscribe<sensor_msgs::Imu>(
        "/imu/data", 5, &TransformFusion::ImuHandler, this);
    reset_point_sub = nh.subscribe("/reset_point", 1, &TransformFusion::reset_point_callback, this);
    // reset_point_sub = nh.subscribe("/initialpose", 1, &TransformFusion::reset_point_callback, this);
    pubLaserOdometry3 =
        nh.advertise<nav_msgs::Odometry>("/combined_to_init", 5);

    laserOdometry3.header.frame_id = "/map";
    laserOdometry3.child_frame_id = "/combine";

    laserOdometryTrans3.frame_id_ = "/map";
    laserOdometryTrans3.child_frame_id_ = "/combine";


    laserOdometry2.header.frame_id = "/camera_init";
    laserOdometry2.child_frame_id = "/camera";

    laserOdometryTrans2.frame_id_ = "/camera_init";
    laserOdometryTrans2.child_frame_id_ = "/camera";

    map_2_camera_init_Trans.frame_id_ = "/map";
    map_2_camera_init_Trans.child_frame_id_ = "/camera_init";


    for (int i = 0; i < 6; ++i) {
      transformSum[i] = 0;
      transformIncre[i] = 0;
      transformMapped[i] = 0;
      transformBefMapped[i] = 0;
      transformAftMapped[i] = 0;
    }

    // filter_ = new StatePredictor();

        restart = false;
        lastVx = lastVy = lastVz = 0;
	OpenFile.open("~/maps/test/IMU.txt");
	OpenFile <<"x "<< ","<<"y"<<","<<"z"<<endl;
  }
  ~TransformFusion(){
    //   delete filter_;
      }  

    void _init_()
    {
        // map_2_Start_Trans.frame_id_ = "/map";
        // map_2_Start_Trans.child_frame_id_ = "/start";
        // tf::Quaternion q2 = tf::createQuaternionFromRPY(roll_op , pitch_op , yaw_op );
        // map_2_Start_Trans.setOrigin(tf::Vector3(x_op, y_op, z_op));
        // map_2_Start_Trans.setRotation(q2);
        
        Qwb.x() = 0;
        Qwb.y() = 0;
        Qwb.z() = 0;
        Qwb.w() = 1;
        Vw.setZero();
        Pwb.setZero();
        last_gyro.setZero();
        last_acc.setZero();

        transformSum[0] = pitch_op ;
        transformSum[1] = yaw_op ;
        transformSum[2] = roll_op ;
        transformSum[3] = y_op;
        transformSum[4] = z_op;
        transformSum[5] = x_op;

        for (int i = 0; i < 6; ++i)
        {
            transformAftMapped[i] = 0;
            // transformSum[i] = 0;
            transformIncre[i] = 0;
            transformMapped[i] = 0;
            transformBefMapped[i]=0;
        }

        // for (int i = 0; i < 6; ++i){
        //     transformMapped[i] = transformSum[i];
        //     transformBefMapped[i] = transformSum[i];
        //     transformAftMapped[i] = transformSum[i];
        // }
        first_imu = false;
        laserodomflag=false;
    }

void midPointIntegration(double _dt, const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
      const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1,
      const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q,
      const Eigen::Vector3d &delta_v, const Eigen::Vector3d &linearized_ba,
      const Eigen::Vector3d &linearized_bg, Eigen::Vector3d &result_delta_p,
      Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
      Eigen::Vector3d &result_linearized_ba,
      Eigen::Vector3d &result_linearized_bg, bool update_jacobian)
{
    // static constexpr unsigned int DIM_OF_STATE_ = 18;
    // static constexpr unsigned int DIM_OF_NOISE_ = 12;
    // static constexpr unsigned int pos_ = 0;
    // static constexpr unsigned int vel_ = 3;
    // static constexpr unsigned int att_ = 6;
    // static constexpr unsigned int acc_ = 9;
    // static constexpr unsigned int gyr_ = 12;
    // static constexpr unsigned int gra_ = 15;

    Vector3d un_acc_0 = delta_q * (_acc_0 - linearized_ba);
    Vector3d un_gyr = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
    result_delta_q =
        delta_q * Quaterniond(1, un_gyr(0) * _dt / 2, un_gyr(1) * _dt / 2,
                              un_gyr(2) * _dt / 2);
    result_delta_q.normalize();
    Vector3d un_acc_1 = result_delta_q * (_acc_1 - linearized_ba);
    Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1 + 2*gw);
    result_delta_p = delta_p + delta_v * _dt + 0.5 * un_acc * _dt * _dt;
    result_delta_v = delta_v + un_acc * _dt;

    result_linearized_ba = linearized_ba;
    result_linearized_bg = linearized_bg;

    if (update_jacobian) {
      Vector3d w_x =
          0.5 * (_gyr_0 + _gyr_1) - linearized_bg;  // angular_velocity
      Vector3d a_0_x =
          _acc_0 - linearized_ba;  // acceleration measurement - bias
      Vector3d a_1_x = _acc_1 - linearized_ba;
      Matrix3d R_w_x, R_a_0_x, R_a_1_x;

      R_w_x << 0, -w_x(2), w_x(1),  // [w-b]x, cross product
          w_x(2), 0, -w_x(0), -w_x(1), w_x(0), 0;
      R_a_0_x << 0, -a_0_x(2), a_0_x(1),  // [w-a]x
          a_0_x(2), 0, -a_0_x(0), -a_0_x(1), a_0_x(0), 0;
      R_a_1_x << 0, -a_1_x(2), a_1_x(1), a_1_x(2), 0, -a_1_x(0), -a_1_x(1),
          a_1_x(0), 0;

      // the order of a and theta is exchanged. and F = I + F*dt + 0.5*F^2*dt^2
      MatrixXd F = MatrixXd::Zero(15, 15);
      F.block<3, 3>(0, 0) =
          Matrix3d::Identity();
      F.block<3, 3>(0, 6) =
          -0.25 * delta_q.toRotationMatrix() * R_a_0_x * _dt * _dt +
          -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x *
              (Matrix3d::Identity() - R_w_x * _dt) * _dt * _dt;
      F.block<3, 3>(0, 3) =
          MatrixXd::Identity(3, 3) * _dt;
      F.block<3, 3>(0, 9) =
          -0.25 *
          (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) *
          _dt * _dt;
      F.block<3, 3>(0, 12) =
          -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * _dt *
          -_dt;

      F.block<3, 3>(6, 6) =
          Matrix3d::Identity() - R_w_x * _dt;
      F.block<3, 3>(6, 12) =
          -1.0 * MatrixXd::Identity(3, 3) * _dt;

      F.block<3, 3>(3, 6) =
          -0.5 * delta_q.toRotationMatrix() * R_a_0_x * _dt +
          -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x *
              (Matrix3d::Identity() - R_w_x * _dt) * _dt;
      F.block<3, 3>(3, 3) =
          Matrix3d::Identity();
      F.block<3, 3>(3, 9) =
          -0.5 *
          (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) *
          _dt;
      F.block<3, 3>(3, 12) =
          -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * -_dt;

      F.block<3, 3>(9, 9) =
          Matrix3d::Identity();
      F.block<3, 3>(12, 12) =
          Matrix3d::Identity();

      jacobian = F * jacobian;
    }

    /*
    Vector3d vk0 = _acc_0*dt;
    Vector3d ak0 = _gyr_0*dt;
    Vector3d vk1 = _acc_1*dt;
    Vector3d ak1 = _gyr_1*dt;

    Vector3d dv = vk1 + 0.5*ak1.cross(vk1) + 1.0/12*(ak0.cross(vk1) +
    vk0.cross(ak1)); Vector3d da = 0.5*(ak0+ak1);

    result_delta_q = delta_q * Quaterniond(1, da(0)/2, da(1)/2, da(2)/2);
    result_delta_v = delta_v + result_delta_q*dv;
    Vector3d aver_v = 0.5*(result_delta_v + delta_v);
    result_delta_p = delta_p + aver_v * _dt;

    result_linearized_ba = linearized_ba;
    result_linearized_bg = linearized_bg;*/
  }


  void ImuHandler(const sensor_msgs::Imu::ConstPtr& imuIn)
  {
    clock_t t1 = clock();
    if(USING_IMU_ODOM)
    {
        if(!first_imu)
        {
            // double roll, pitch, yaw;
            // tf::Quaternion orientation;
            // tf::quaternionMsgToTF(imuIn->orientation, orientation);
            // tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
            // first_yaw = yaw;
            // first_roll = roll;
            // first_pitch = pitch;
            first_imu = true;
            lastImuTime = imuIn->header.stamp.toSec();

            last_acc = Eigen::Vector3d(imuIn->linear_acceleration.x,imuIn->linear_acceleration.y,imuIn->linear_acceleration.z);
            last_gyro = Eigen::Vector3d(imuIn->angular_velocity.x,imuIn->angular_velocity.y,imuIn->angular_velocity.z);
        
            return;
        }
        double imu_time = imuIn->header.stamp.toSec();
        double dt = imu_time - lastImuTime;
        lastImuTime = imu_time;

        Vector3d imu_acc;
        Vector3d imu_gyro;
        imu_acc(0) = imuIn->linear_acceleration.x;
        imu_acc(1) = imuIn->linear_acceleration.y;
        imu_acc(2) = imuIn->linear_acceleration.z;
        imu_gyro(0) = imuIn->angular_velocity.x;
        imu_gyro(1) = imuIn->angular_velocity.y;
        imu_gyro(2) = imuIn->angular_velocity.z;
        // acc_0_ = imu_acc;
        // gyr_0_ = imu_gyro;
        
        if(!laserodomflag)
         return;

        // double lastOdometryTime = currentHeader.stamp.toSec();
        // double timed = imu_time - lastOdometryTime;

        // if(timed<0)
        // {
        //     ROS_ERROR("time stemp is error");
        //     return;
        // }
        Vector3d result_delta_p;
        Quaterniond result_delta_q;
        Vector3d result_delta_v;
        Vector3d result_linearized_ba;
        Vector3d result_linearized_bg;
        midPointIntegration(dt, last_acc, last_gyro, imu_acc, imu_gyro, Pwb, Qwb, Vw, 
                            linearized_ba, linearized_bg, result_delta_p, result_delta_q, 
                            result_delta_v, result_linearized_ba, result_linearized_bg, true);
        Pwb = result_delta_p;
        Qwb = result_delta_q;
        Vw = result_delta_v;
        linearized_ba = result_linearized_ba;
        linearized_bg = result_linearized_bg;
        last_acc = imu_acc;
        last_gyro = imu_gyro;

            std::cout<<"dt "<<dt<<" pose: "<<Pwb(0)<<" "
                               <<Pwb(1)<<" "
                               <<Pwb(2)<<" "
                      <<" Vel: "<<Vw(0)<<" "
                               <<Vw(1)<<" "
                               <<Vw(2)<<std::endl;
            
            // Quaterniond Qwb = Quaterniond(quad_);

            // std::lock_guard<std::mutex> lock(mux);
            laserOdometry3.header.stamp = imuIn->header.stamp;
            laserOdometry3.pose.pose.orientation.x = Qwb.x();
            laserOdometry3.pose.pose.orientation.y = Qwb.y();
            laserOdometry3.pose.pose.orientation.z = Qwb.z();
            laserOdometry3.pose.pose.orientation.w = Qwb.w();
            laserOdometry3.pose.pose.position.x = Pwb(0);
            laserOdometry3.pose.pose.position.y = Pwb(1);
            laserOdometry3.pose.pose.position.z = Pwb(2);
            pubLaserOdometry3.publish(laserOdometry3);

            laserOdometryTrans3.stamp_ = imuIn->header.stamp;
            laserOdometryTrans3.setRotation(
                tf::Quaternion(Qwb.x(),Qwb.y(),Qwb.z(),Qwb.w()));
            laserOdometryTrans3.setOrigin(tf::Vector3(
                Pwb(0),Pwb(1),Pwb(2)));
            tfBroadcaster3.sendTransform(laserOdometryTrans3);
            clock_t t2 = clock();
            std::cout << (t2 - t1)/CLOCKS_PER_SEC<<std::endl;
            OpenFile<<Pwb(0)<<","<<Pwb(1)<<","<<Pwb(2)<<std::endl;
    }
  }

    void reset_point_callback(const plan_msgs::PointSYK::ConstPtr& msg)
    {
        if(msg->ks == 1.0)
        {
            restart = true;

            x_op = msg->x;
            y_op = msg->y;
            z_op = msg->s;
            yaw_op = msg->yaw;
            
        }
    }

  void transformAssociateToMap() {
    float x1 =
        cos(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) -
        sin(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);
    float y1 = transformBefMapped[4] - transformSum[4];
    float z1 =
        sin(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) +
        cos(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);

    float x2 = x1;
    float y2 = cos(transformSum[0]) * y1 + sin(transformSum[0]) * z1;
    float z2 = -sin(transformSum[0]) * y1 + cos(transformSum[0]) * z1;

    transformIncre[3] = cos(transformSum[2]) * x2 + sin(transformSum[2]) * y2;
    transformIncre[4] = -sin(transformSum[2]) * x2 + cos(transformSum[2]) * y2;
    transformIncre[5] = z2;

    float sbcx = sin(transformSum[0]);
    float cbcx = cos(transformSum[0]);
    float sbcy = sin(transformSum[1]);
    float cbcy = cos(transformSum[1]);
    float sbcz = sin(transformSum[2]);
    float cbcz = cos(transformSum[2]);

    float sblx = sin(transformBefMapped[0]);
    float cblx = cos(transformBefMapped[0]);
    float sbly = sin(transformBefMapped[1]);
    float cbly = cos(transformBefMapped[1]);
    float sblz = sin(transformBefMapped[2]);
    float cblz = cos(transformBefMapped[2]);

    float salx = sin(transformAftMapped[0]);
    float calx = cos(transformAftMapped[0]);
    float saly = sin(transformAftMapped[1]);
    float caly = cos(transformAftMapped[1]);
    float salz = sin(transformAftMapped[2]);
    float calz = cos(transformAftMapped[2]);

    float srx = -sbcx * (salx * sblx + calx * cblx * salz * sblz +
                         calx * calz * cblx * cblz) -
                cbcx * sbcy *
                    (calx * calz * (cbly * sblz - cblz * sblx * sbly) -
                     calx * salz * (cbly * cblz + sblx * sbly * sblz) +
                     cblx * salx * sbly) -
                cbcx * cbcy *
                    (calx * salz * (cblz * sbly - cbly * sblx * sblz) -
                     calx * calz * (sbly * sblz + cbly * cblz * sblx) +
                     cblx * cbly * salx);
    transformMapped[0] = -asin(srx);

    float srycrx = sbcx * (cblx * cblz * (caly * salz - calz * salx * saly) -
                           cblx * sblz * (caly * calz + salx * saly * salz) +
                           calx * saly * sblx) -
                   cbcx * cbcy *
                       ((caly * calz + salx * saly * salz) *
                            (cblz * sbly - cbly * sblx * sblz) +
                        (caly * salz - calz * salx * saly) *
                            (sbly * sblz + cbly * cblz * sblx) -
                        calx * cblx * cbly * saly) +
                   cbcx * sbcy *
                       ((caly * calz + salx * saly * salz) *
                            (cbly * cblz + sblx * sbly * sblz) +
                        (caly * salz - calz * salx * saly) *
                            (cbly * sblz - cblz * sblx * sbly) +
                        calx * cblx * saly * sbly);
    float crycrx = sbcx * (cblx * sblz * (calz * saly - caly * salx * salz) -
                           cblx * cblz * (saly * salz + caly * calz * salx) +
                           calx * caly * sblx) +
                   cbcx * cbcy *
                       ((saly * salz + caly * calz * salx) *
                            (sbly * sblz + cbly * cblz * sblx) +
                        (calz * saly - caly * salx * salz) *
                            (cblz * sbly - cbly * sblx * sblz) +
                        calx * caly * cblx * cbly) -
                   cbcx * sbcy *
                       ((saly * salz + caly * calz * salx) *
                            (cbly * sblz - cblz * sblx * sbly) +
                        (calz * saly - caly * salx * salz) *
                            (cbly * cblz + sblx * sbly * sblz) -
                        calx * caly * cblx * sbly);
    transformMapped[1] = atan2(srycrx / cos(transformMapped[0]),
                               crycrx / cos(transformMapped[0]));

    float srzcrx = (cbcz * sbcy - cbcy * sbcx * sbcz) *
                       (calx * salz * (cblz * sbly - cbly * sblx * sblz) -
                        calx * calz * (sbly * sblz + cbly * cblz * sblx) +
                        cblx * cbly * salx) -
                   (cbcy * cbcz + sbcx * sbcy * sbcz) *
                       (calx * calz * (cbly * sblz - cblz * sblx * sbly) -
                        calx * salz * (cbly * cblz + sblx * sbly * sblz) +
                        cblx * salx * sbly) +
                   cbcx * sbcz *
                       (salx * sblx + calx * cblx * salz * sblz +
                        calx * calz * cblx * cblz);
    float crzcrx = (cbcy * sbcz - cbcz * sbcx * sbcy) *
                       (calx * calz * (cbly * sblz - cblz * sblx * sbly) -
                        calx * salz * (cbly * cblz + sblx * sbly * sblz) +
                        cblx * salx * sbly) -
                   (sbcy * sbcz + cbcy * cbcz * sbcx) *
                       (calx * salz * (cblz * sbly - cbly * sblx * sblz) -
                        calx * calz * (sbly * sblz + cbly * cblz * sblx) +
                        cblx * cbly * salx) +
                   cbcx * cbcz *
                       (salx * sblx + calx * cblx * salz * sblz +
                        calx * calz * cblx * cblz);
    transformMapped[2] = atan2(srzcrx / cos(transformMapped[0]),
                               crzcrx / cos(transformMapped[0]));

    x1 = cos(transformMapped[2]) * transformIncre[3] -
         sin(transformMapped[2]) * transformIncre[4];
    y1 = sin(transformMapped[2]) * transformIncre[3] +
         cos(transformMapped[2]) * transformIncre[4];
    z1 = transformIncre[5];

    x2 = x1;
    y2 = cos(transformMapped[0]) * y1 - sin(transformMapped[0]) * z1;
    z2 = sin(transformMapped[0]) * y1 + cos(transformMapped[0]) * z1;

    transformMapped[3] = transformAftMapped[3] - (cos(transformMapped[1]) * x2 +
                                                  sin(transformMapped[1]) * z2);
    transformMapped[4] = transformAftMapped[4] - y2;
    transformMapped[5] =
        transformAftMapped[5] -
        (-sin(transformMapped[1]) * x2 + cos(transformMapped[1]) * z2);
  }

  void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry) {
        if(restart)
        {
            std::lock_guard<std::mutex> lock(init_mutex);
            _init_();
            ROS_INFO("**init_trans_state**");
            restart = false;
        }
    currentHeader = laserOdometry->header;
   if(!laserodomflag)
   {
        // status_ = STATUS_FIRST_SCAN;
        // pos_.setZero();
        // vel_.setZero();
        // quad_.setIdentity();
        lastVx = lastVy = lastVz = 0;

        // filter_->initialization(laserOdometry->header.stamp.toSec(), V3D(0, 0, 0), V3D(0, 0, 0),
        //                         V3D(0, 0, 0), V3D(0, 0, 0), acc_0_,gyr_0_);
        laserodomflag = true;
        lastlaserTime = currentHeader.stamp.toSec();
   }
        // status_ = STATUS_RUNNING;
        double dlt = currentHeader.stamp.toSec() - lastlaserTime;
        lastlaserTime = currentHeader.stamp.toSec();
        if(dlt<0.09)
        {
            dlt = 0.1;
        }

    double roll, pitch, yaw;
    geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w))
        .getRPY(roll, pitch, yaw);
    

    double vx = 0.5*(lastVx + (transformSum[3] - laserOdometry->pose.pose.position.x)/dlt);
    double vy = 0.5*(lastVy + (transformSum[4] - laserOdometry->pose.pose.position.y)/dlt);
    double vz = 0.5*(lastVz + (transformSum[5] - laserOdometry->pose.pose.position.z)/dlt);

    lastVx = vx;
    lastVy = vy;
    lastVz = vz;
    // //坐标系不同

    transformSum[0] = -pitch + pitch_op ;
    transformSum[1] = -yaw + yaw_op ;
    transformSum[2] = roll + roll_op ;
    double dy = laserOdometry->pose.pose.position.x;
    double dz = laserOdometry->pose.pose.position.y;
    double dx = laserOdometry->pose.pose.position.z;
    transformSum[3] = y_op + dy * cos(yaw_op) + dx * sin(yaw_op);
    transformSum[4] = z_op + dz;
    transformSum[5] = x_op + dx * cos(yaw_op) - dy * sin(yaw_op);

    transformAssociateToMap();

    geoQuat = tf::createQuaternionMsgFromRollPitchYaw(
        transformMapped[2], -transformMapped[0], -transformMapped[1]);
        Quaterniond quat;
    {
        std::lock_guard<std::mutex> lock(mux);
        Qwb.w() = geoQuat.w;
        Qwb.x() = geoQuat.x;
        Qwb.y() = -geoQuat.y;
        Qwb.z() = -geoQuat.z;
        // quad_ = Matrix3d(quat);
        // vel_(0) = vz;
        // vel_(1) = vx;
        // vel_(2) = vy;
        Vw(0) = vz;
        Vw(1) = vx;
        Vw(2) = vy;
        Pwb(0) = transformMapped[5];
        Pwb(1) = transformMapped[3];
        Pwb(2) = transformMapped[4];
        count = 0;
    }
    count++;

    laserOdometry2.header.stamp = laserOdometry->header.stamp;
    laserOdometry2.pose.pose.orientation.x = -geoQuat.y;
    laserOdometry2.pose.pose.orientation.y = -geoQuat.z;
    laserOdometry2.pose.pose.orientation.z = geoQuat.x;
    laserOdometry2.pose.pose.orientation.w = geoQuat.w;
    laserOdometry2.pose.pose.position.x = transformMapped[3];
    laserOdometry2.pose.pose.position.y = transformMapped[4];
    laserOdometry2.pose.pose.position.z = transformMapped[5];
    pubLaserOdometry2.publish(laserOdometry2);



    laserOdometryTrans2.stamp_ = laserOdometry->header.stamp;
    laserOdometryTrans2.setRotation(
        tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
    laserOdometryTrans2.setOrigin(tf::Vector3(
        transformMapped[3], transformMapped[4], transformMapped[5]));
    tfBroadcaster2.sendTransform(laserOdometryTrans2);


    // if(!USING_IMU_ODOM)
    // {
    // // std::lock_guard<std::mutex> lock(mux);
    // laserOdometry3.header.stamp = laserOdometry->header.stamp;
    // laserOdometry3.pose.pose.orientation.x = geoQuat.x;
    // laserOdometry3.pose.pose.orientation.y = -geoQuat.y;
    // laserOdometry3.pose.pose.orientation.z = -geoQuat.z;
    // laserOdometry3.pose.pose.orientation.w = geoQuat.w;
    // laserOdometry3.pose.pose.position.x = transformMapped[5];
    // laserOdometry3.pose.pose.position.y = transformMapped[3];
    // laserOdometry3.pose.pose.position.z = transformMapped[4];
    // pubLaserOdometry3.publish(laserOdometry3);

    // laserOdometryTrans3.stamp_ = laserOdometry->header.stamp;
    // laserOdometryTrans3.setRotation(tf::Quaternion(geoQuat.x, -geoQuat.y, -geoQuat.z, geoQuat.w));
    // laserOdometryTrans3.setOrigin(tf::Vector3(transformMapped[5],transformMapped[3],transformMapped[4]));
    // tfBroadcaster3.sendTransform(laserOdometryTrans3);
    // }

  }

  void odomAftMappedHandler(const nav_msgs::Odometry::ConstPtr& odomAftMapped) {
    double roll, pitch, yaw;
    geometry_msgs::Quaternion geoQuat = odomAftMapped->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w))
        .getRPY(roll, pitch, yaw);

    transformAftMapped[0] = -pitch;
    transformAftMapped[1] = -yaw;
    transformAftMapped[2] = roll;

    transformAftMapped[3] = odomAftMapped->pose.pose.position.x;
    transformAftMapped[4] = odomAftMapped->pose.pose.position.y;
    transformAftMapped[5] = odomAftMapped->pose.pose.position.z;

    transformBefMapped[0] = odomAftMapped->twist.twist.angular.x;
    transformBefMapped[1] = odomAftMapped->twist.twist.angular.y;
    transformBefMapped[2] = odomAftMapped->twist.twist.angular.z;

    transformBefMapped[3] = odomAftMapped->twist.twist.linear.x;
    transformBefMapped[4] = odomAftMapped->twist.twist.linear.y;
    transformBefMapped[5] = odomAftMapped->twist.twist.linear.z;

    // {
    //     std::lock_guard<std::mutex> lock(mux);

    //     Qwb.x() = geoQuat.z;
    //     Qwb.y() = geoQuat.x;
    //     Qwb.z() = geoQuat.y;
    //     Qwb.w() = geoQuat.w;

    //     Pwb(0) = transformAftMapped[5];
    //     Pwb(1) = transformAftMapped[3];
    //     Pwb(2) = transformAftMapped[4];
    // } 

    if(!USING_IMU_ODOM)
    {
    // std::lock_guard<std::mutex> lock(mux);
    laserOdometry3.header.stamp = odomAftMapped->header.stamp;
    laserOdometry3.pose.pose.orientation.x = -geoQuat.z;
    laserOdometry3.pose.pose.orientation.y = -geoQuat.x;
    laserOdometry3.pose.pose.orientation.z = geoQuat.y;
    laserOdometry3.pose.pose.orientation.w = geoQuat.w;
    laserOdometry3.pose.pose.position.x = transformAftMapped[5];
    laserOdometry3.pose.pose.position.y = transformAftMapped[3];
    laserOdometry3.pose.pose.position.z = transformAftMapped[4];
    pubLaserOdometry3.publish(laserOdometry3);

    laserOdometryTrans3.stamp_ = odomAftMapped->header.stamp;
    laserOdometryTrans3.setRotation(tf::Quaternion(geoQuat.x, -geoQuat.y, -geoQuat.z, geoQuat.w));
    laserOdometryTrans3.setOrigin(tf::Vector3(transformAftMapped[5],transformAftMapped[3],transformAftMapped[4]));
    tfBroadcaster3.sendTransform(laserOdometryTrans3);
    }
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "lego_loam");

  TransformFusion TFusion;

  ROS_INFO("\033[1;32m---->\033[0m Transform Fusion Started.");

  ros::spin();
  OpenFile.close();
  return 0;
}
