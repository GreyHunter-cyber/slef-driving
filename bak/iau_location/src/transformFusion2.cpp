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
#include <mutex>

ofstream OpenFile;
class TransformFusion2{

private:

    ros::NodeHandle nh;

    ros::Subscriber subLaserOdometry;
    ros::Subscriber subOdomAftMapped;
    ros::Subscriber subImu;

    tf::StampedTransform map_2_Start_Trans;
    tf::TransformBroadcaster tfBroadcasterMap2Start;

    ros::Publisher pubCombinedOdometry;
    ros::Publisher pubBackwheelOdometry;

    nav_msgs::Odometry combinedOdometry;
    nav_msgs::Odometry backwheelOdometry;

    tf::StampedTransform combinedOdometryTrans;
    tf::StampedTransform backwheelOdometryTrans;

    tf::TransformBroadcaster tfBroadcaster_combinedOdometryTrans;
    tf::TransformBroadcaster tfBroadcaster_backwheelOdometryTrans;

    tf::StampedTransform imu_2_map_Trans;
    tf::TransformBroadcaster tfBroadcaster_Imu;

    double transformSum[6];
    double transformIncre[6];
    double transformMapped[6];
    double transformBefMapped[6];
    double transformAftMapped[6];
    
    std_msgs::Header currentHeader;

    int imuPointerFront;
    int imuPointerLast;
    int imuPointerLastIteration;

    double imuTime[imuQueLength];
    float imuRoll[imuQueLength];
    float imuPitch[imuQueLength];
    float imuYaw[imuQueLength];

    float imuAccX[imuQueLength];
    float imuAccY[imuQueLength];
    float imuAccZ[imuQueLength];

    float imuVeloX[imuQueLength];
    float imuVeloY[imuQueLength];
    float imuVeloZ[imuQueLength];

    float imuShiftX[imuQueLength];
    float imuShiftY[imuQueLength];
    float imuShiftZ[imuQueLength];

    float imuAngularVeloX[imuQueLength];
    float imuAngularVeloY[imuQueLength];
    float imuAngularVeloZ[imuQueLength];

    float imuAngularRotationX[imuQueLength];
    float imuAngularRotationY[imuQueLength];
    float imuAngularRotationZ[imuQueLength];

    double xlast, ylast, zlast;
    double xvel, yvel, zvel;
    double lastOdometryTime;

    double pose_x, pose_y, pose_z, pose_roll, pose_pitch, pose_yaw;

    bool first_laserodom, first_aft_laserodom, first_imu;
    double first_yaw;
        //初始位姿
    float x_op, y_op, z_op, roll_op, pitch_op, yaw_op;
    string fileDirectory;
    int IMU_count,IMU_filter_num;//imu过滤
    std::mutex init_mutex;
    bool restart;
    ros::Subscriber reset_point_sub;

public:

    TransformFusion2(){

        pubCombinedOdometry = nh.advertise<nav_msgs::Odometry> ("/integrated_to_init", 5);
        pubBackwheelOdometry = nh.advertise<nav_msgs::Odometry>("/wheel_to_init", 5);
        subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/laser_odom_to_init", 5, &TransformFusion2::laserOdometryHandler, this);
        subOdomAftMapped = nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init", 5, &TransformFusion2::odomAftMappedHandler, this);
        subImu = nh.subscribe<sensor_msgs::Imu>(imuTopic, 50, &TransformFusion2::imuHandler, this);
        reset_point_sub = nh.subscribe("/initialpose", 1, &TransformFusion2::reset_point_callback, this);


        nh.param<string>("/fileDirectory",fileDirectory,"/home/luoluolll");
        nh.param<int>("/IMU_filter_num",IMU_filter_num,0);
        IMU_count=0;//
        OpenFile.open(fileDirectory+"traj.txt");
        //private_nh.param<float>("car_front",car_front,0.55);
       // private_nh.param<float>("car_up",car_up,0.65);
        nh.param<float>("/x_op",x_op,0.0);
        nh.param<float>("/y_op",y_op,0.0);
        nh.param<float>("/z_op",z_op,0.0);
        nh.param<float>("/roll_op",roll_op,0.0);
        nh.param<float>("/pitch_op",pitch_op,0.0);
        nh.param<float>("/yaw_op",yaw_op,0.0);
        std::cout << "\n-----------------------initial pose:------------------\n"
                  << x_op << ", " << y_op << ", " << z_op << ", " << yaw_op << std::endl;

        combinedOdometry.header.frame_id = "/map";
        combinedOdometry.child_frame_id = "/base_link";
        combinedOdometryTrans.frame_id_ = "/map";
        combinedOdometryTrans.child_frame_id_ = "/base_link";

        backwheelOdometry.header.frame_id = "/map";
        backwheelOdometry.child_frame_id = "/wheel";
        backwheelOdometryTrans.frame_id_ = "/map";
        backwheelOdometryTrans.child_frame_id_ = "/wheel";

        imu_2_map_Trans.frame_id_ = "/map";
        imu_2_map_Trans.child_frame_id_ = "/imu";
        restart = false;
        _init_();
    }

    void _init_()
    {
        map_2_Start_Trans.frame_id_ = "/map";
        map_2_Start_Trans.child_frame_id_ = "/start";
        tf::Quaternion q2 = tf::createQuaternionFromRPY(roll_op , pitch_op , yaw_op );
        map_2_Start_Trans.setOrigin(tf::Vector3(x_op, y_op, z_op));
        map_2_Start_Trans.setRotation(q2);

        transformSum[0] = pitch_op ;
        transformSum[1] = yaw_op ;
        transformSum[2] = roll_op ;
        transformSum[3] = y_op;
        transformSum[4] = z_op;
        transformSum[5] = x_op;

        for (int i = 0; i < 6; ++i)
        {
            transformAftMapped[i] = 0;
            transformSum[i] = 0;
            transformIncre[i] = 0;
            transformMapped[i] = 0;
            transformBefMapped[i]=0;
        }

        for (int i = 0; i < 6; ++i){
            transformMapped[i] = transformSum[i];
            transformBefMapped[i] = transformSum[i];
            transformAftMapped[i] = transformSum[i];
        }

        imuPointerFront = 0;
        imuPointerLast = -1;
        imuPointerLastIteration = 0;

        for (int i = 0; i < imuQueLength; ++i)
        {
            imuTime[i] = 0;
            imuRoll[i] = 0; imuPitch[i] = 0; imuYaw[i] = 0;
            imuAccX[i] = 0; imuAccY[i] = 0; imuAccZ[i] = 0;
            imuVeloX[i] = 0; imuVeloY[i] = 0; imuVeloZ[i] = 0;
            imuShiftX[i] = 0; imuShiftY[i] = 0; imuShiftZ[i] = 0;
            imuAngularVeloX[i] = 0; imuAngularVeloY[i] = 0; imuAngularVeloZ[i] = 0;
            imuAngularRotationX[i] = 0; imuAngularRotationY[i] = 0; imuAngularRotationZ[i] = 0;
        }


        xlast = x_op, ylast = y_op, zlast = z_op;
        xvel = 0, yvel = 0, zvel = 0;
        lastOdometryTime = 0;

        pose_x = x_op, pose_y = y_op, pose_z = z_op, pose_roll = roll_op, pose_pitch = pitch_op, pose_yaw = yaw_op;

        first_laserodom = false;
        first_aft_laserodom = false;
        first_imu = false;
        first_yaw = 0;

    }

    void outputTransforms()
    {
        std::cout << "\n-----------------transformSum:----------------" << std::endl;
        for (int i = 0; i < 6; i++)
        {
            std::cout << transformSum[i] << ",  ";
        }
        std::cout << "\n-----------------transformIncre:----------------" << std::endl;
        for (int i = 0; i < 6; i++)
        {
            std::cout << transformIncre[i] << ",  ";
        }
        std::cout << "\n-----------------transformBefMapped:----------------" << std::endl;
        for (int i = 0; i < 6; i++)
        {
            std::cout << transformBefMapped[i] << ",  ";
        }
        std::cout << "\n-----------------transformAftMapped:----------------" << std::endl;
        for (int i = 0; i < 6; i++)
        {
            std::cout << transformAftMapped[i] << ",  ";
        }
        std::cout << "\n-----------------transformMapped:----------------" << std::endl;
        for (int i = 0; i < 6; i++)
        {
            std::cout << transformMapped[i] << ",  ";
        }
    }

    void transformAssociateToMap()
    {
        float x1 = cos(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) 
                 - sin(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);
        float y1 = transformBefMapped[4] - transformSum[4];
        float z1 = sin(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) 
                 + cos(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);

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

        float srx = -sbcx*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz)
                  - cbcx*sbcy*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                  - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
                  - cbcx*cbcy*(calx*salz*(cblz*sbly - cbly*sblx*sblz) 
                  - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx);
        transformMapped[0] = -asin(srx);

        float srycrx = sbcx*(cblx*cblz*(caly*salz - calz*salx*saly)
                     - cblx*sblz*(caly*calz + salx*saly*salz) + calx*saly*sblx)
                     - cbcx*cbcy*((caly*calz + salx*saly*salz)*(cblz*sbly - cbly*sblx*sblz)
                     + (caly*salz - calz*salx*saly)*(sbly*sblz + cbly*cblz*sblx) - calx*cblx*cbly*saly)
                     + cbcx*sbcy*((caly*calz + salx*saly*salz)*(cbly*cblz + sblx*sbly*sblz)
                     + (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly) + calx*cblx*saly*sbly);
        float crycrx = sbcx*(cblx*sblz*(calz*saly - caly*salx*salz)
                     - cblx*cblz*(saly*salz + caly*calz*salx) + calx*caly*sblx)
                     + cbcx*cbcy*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx)
                     + (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz) + calx*caly*cblx*cbly)
                     - cbcx*sbcy*((saly*salz + caly*calz*salx)*(cbly*sblz - cblz*sblx*sbly)
                     + (calz*saly - caly*salx*salz)*(cbly*cblz + sblx*sbly*sblz) - calx*caly*cblx*sbly);
        transformMapped[1] = atan2(srycrx / cos(transformMapped[0]), 
                                   crycrx / cos(transformMapped[0]));
        
        float srzcrx = (cbcz*sbcy - cbcy*sbcx*sbcz)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                     - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx)
                     - (cbcy*cbcz + sbcx*sbcy*sbcz)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                     - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
                     + cbcx*sbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);
        float crzcrx = (cbcy*sbcz - cbcz*sbcx*sbcy)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                     - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
                     - (sbcy*sbcz + cbcy*cbcz*sbcx)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                     - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx)
                     + cbcx*cbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);
        transformMapped[2] = atan2(srzcrx / cos(transformMapped[0]), 
                                   crzcrx / cos(transformMapped[0]));

        x1 = cos(transformMapped[2]) * transformIncre[3] - sin(transformMapped[2]) * transformIncre[4];
        y1 = sin(transformMapped[2]) * transformIncre[3] + cos(transformMapped[2]) * transformIncre[4];
        z1 = transformIncre[5];

        x2 = x1;
        y2 = cos(transformMapped[0]) * y1 - sin(transformMapped[0]) * z1;
        z2 = sin(transformMapped[0]) * y1 + cos(transformMapped[0]) * z1;

        transformMapped[3] = transformAftMapped[3] 
                           - (cos(transformMapped[1]) * x2 + sin(transformMapped[1]) * z2);
        transformMapped[4] = transformAftMapped[4] - y2;
        transformMapped[5] = transformAftMapped[5] 
                           - (-sin(transformMapped[1]) * x2 + cos(transformMapped[1]) * z2);
    }

    void reset_point_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
    {
            restart = true;

            x_op = msg.pose.pose.position.x;
            y_op = msg.pose.pose.position.y;
            z_op = msg.pose.pose.position.z;
            double roll, pitch, yaw;
            geometry_msgs::Quaternion geoQuat = msg.pose.pose.orientation;
            tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);
            yaw_op = yaw;

            
    }

    void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry)
    {
        if(restart)
        {
            std::lock_guard<std::mutex> lock(init_mutex);
            _init_();
            ROS_INFO("**init_trans_state**");
            restart = false;
        }
        
        map_2_Start_Trans.stamp_ = ros::Time::now();
        tfBroadcasterMap2Start.sendTransform(map_2_Start_Trans);
        
        currentHeader = laserOdometry->header;

        double roll, pitch, yaw;
        geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
        tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

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
        std::cout << "laserOdometry correct" << std::endl;

        if(first_laserodom)
        {
            double currentTime = laserOdometry->header.stamp.toSec();
            double timediff = (currentTime - lastOdometryTime);
            xvel = (transformMapped[5] - xlast) / timediff;
            yvel = (transformMapped[3] - ylast) / timediff;
            zvel = (transformMapped[4] - zlast) / timediff;

            lastOdometryTime = currentTime;
            xlast = transformMapped[5], ylast = transformMapped[3], zlast = transformMapped[4];
            pose_x = xlast, pose_y = ylast, pose_z = zlast;
            pose_roll = transformMapped[2], pose_pitch = transformMapped[0], pose_yaw = transformMapped[1];
        }
        else
        {
            lastOdometryTime = laserOdometry->header.stamp.toSec();
            xlast = transformMapped[5], ylast = transformMapped[3], zlast = transformMapped[4];
            pose_x = xlast, pose_y = ylast, pose_z = zlast;
            pose_roll = transformMapped[2], pose_pitch = transformMapped[0], pose_yaw = transformMapped[1];
            first_laserodom = true;
        }

        geoQuat = tf::createQuaternionMsgFromRollPitchYaw(transformMapped[2], -transformMapped[0], -transformMapped[1]);

        combinedOdometry.header.stamp = laserOdometry->header.stamp;
        combinedOdometry.pose.pose.orientation.x = geoQuat.x;
        combinedOdometry.pose.pose.orientation.y = -geoQuat.y;
        combinedOdometry.pose.pose.orientation.z = -geoQuat.z;
        combinedOdometry.pose.pose.orientation.w = geoQuat.w;
        combinedOdometry.pose.pose.position.x = transformMapped[5];
        combinedOdometry.pose.pose.position.y = transformMapped[3];
        combinedOdometry.pose.pose.position.z = transformMapped[4];
        pubCombinedOdometry.publish(combinedOdometry);

        combinedOdometryTrans.stamp_ = laserOdometry->header.stamp;
        combinedOdometryTrans.setRotation(tf::Quaternion(geoQuat.x, -geoQuat.y, -geoQuat.z, geoQuat.w));
        combinedOdometryTrans.setOrigin(tf::Vector3(transformMapped[5], transformMapped[3], transformMapped[4]));
        tfBroadcaster_combinedOdometryTrans.sendTransform(combinedOdometryTrans);

        backwheelOdometry.header.stamp = laserOdometry->header.stamp;
        backwheelOdometry.pose.pose.orientation.x = geoQuat.x;
        backwheelOdometry.pose.pose.orientation.y = -geoQuat.y;
        backwheelOdometry.pose.pose.orientation.z = -geoQuat.z;
        backwheelOdometry.pose.pose.orientation.w = geoQuat.w;
        double wheel_x = transformMapped[5] - car_front * cos(transformMapped[1]);
        double wheel_y = transformMapped[3] - car_front * sin(transformMapped[1]);
        double wheel_z = transformMapped[4] - car_up;
        backwheelOdometry.pose.pose.position.x = wheel_x;
        backwheelOdometry.pose.pose.position.y = wheel_y;
        backwheelOdometry.pose.pose.position.z = wheel_z;
        pubBackwheelOdometry.publish(backwheelOdometry);

        backwheelOdometryTrans.stamp_ = laserOdometry->header.stamp;
        backwheelOdometryTrans.setRotation(tf::Quaternion(geoQuat.x, -geoQuat.y, -geoQuat.z, geoQuat.w));
        backwheelOdometryTrans.setOrigin(tf::Vector3(wheel_x, wheel_y, wheel_z));
        tfBroadcaster_backwheelOdometryTrans.sendTransform(backwheelOdometryTrans);
        
        std::cout << "x:" << pose_x << ", y:" << pose_y << ", z:" << pose_z << std::endl;

        /*
        */
    }

    void odomAftMappedHandler(const nav_msgs::Odometry::ConstPtr& odomAftMapped)
    {
        first_aft_laserodom = true;

        double roll, pitch, yaw;
        geometry_msgs::Quaternion geoQuat = odomAftMapped->pose.pose.orientation;
        tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

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
    }


    void AccumulateIMUShiftAndRotation()
    {
        int imuPointerBack = (imuPointerLast + imuQueLength - 1) % imuQueLength;
        double timeDiff = imuTime[imuPointerLast] - imuTime[imuPointerBack];
        if (timeDiff < scanPeriod) {
            imuAngularRotationX[imuPointerLast] = imuAngularRotationX[imuPointerBack] + imuAngularVeloX[imuPointerBack] * timeDiff;
            imuAngularRotationY[imuPointerLast] = imuAngularRotationY[imuPointerBack] + imuAngularVeloY[imuPointerBack] * timeDiff;
            imuAngularRotationZ[imuPointerLast] = imuAngularRotationZ[imuPointerBack] + imuAngularVeloZ[imuPointerBack] * timeDiff;
        
            float roll = imuRoll[imuPointerLast];
            float pitch = imuPitch[imuPointerLast];
            float yaw = imuYaw[imuPointerLast]-yaw_op;
            float accX = imuAccX[imuPointerLast];
            float accY = imuAccY[imuPointerLast];
            float accZ = imuAccZ[imuPointerLast];

            float x1 = cos(roll) * accX - sin(roll) * accY;
            float y1 = sin(roll) * accX + cos(roll) * accY;
            float z1 = accZ;

            float x2 = x1;
            float y2 = cos(pitch) * y1 - sin(pitch) * z1;
            float z2 = sin(pitch) * y1 + cos(pitch) * z1;

            accX = cos(yaw) * x2 + sin(yaw) * z2;
            accY = y2;
            accZ = -sin(yaw) * x2 + cos(yaw) * z2;

            double dy = (yvel * timeDiff + accX * timeDiff * timeDiff / 2);
            double dx = (xvel * timeDiff + accZ * timeDiff * timeDiff / 2);
            double dz = (zvel * timeDiff + accY * timeDiff * timeDiff / 2);

            pose_y += dy;
            pose_x += dx;
            pose_z += dz;

            double dyvel = accX * timeDiff;
            double dxvel = accZ * timeDiff;
            double dzvel = accY * timeDiff;

            yvel += dyvel;
            xvel += dxvel;
            zvel += dzvel;

            pose_roll += imuAngularVeloX[imuPointerBack] * timeDiff;
            pose_pitch += imuAngularVeloY[imuPointerBack] * timeDiff;
            pose_yaw += imuAngularVeloZ[imuPointerBack] * timeDiff;
        }
    }

    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn)
    {
        if(!first_imu)
        {
            double roll, pitch, yaw;
            tf::Quaternion orientation;
            tf::quaternionMsgToTF(imuIn->orientation, orientation);
            tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
            first_yaw = yaw;
            first_imu = true;
            return;
        }

        if(first_imu)
        {
            double imu_time = imuIn->header.stamp.toSec();
            double timed = abs(imu_time - lastOdometryTime);
            if(timed > 0.2)
            {
                std::cout<<"no laserOdometry received"<<std::endl;
                return;
            }
        }
        if(IMU_count<IMU_filter_num)
        {
            IMU_count++;
            return;
        }
        else
        {
        
        IMU_count=0;
        
        //std::cout << "Imu Time:" << imuIn->header.stamp << std::endl;

        double roll, pitch, yaw;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(imuIn->orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        yaw = yaw - first_yaw + yaw_op;

        float accX = imuIn->linear_acceleration.y - sin(roll) * cos(pitch) * 9.81;   
        float accY = imuIn->linear_acceleration.z - cos(roll) * cos(pitch) * 9.81;
        float accZ = imuIn->linear_acceleration.x + sin(pitch) * 9.81;


        imuPointerLast = (imuPointerLast + 1) % imuQueLength;

        //std::cout <<"Imu time stamp: "<< imuIn->header.stamp << endl;
        imuTime[imuPointerLast] = imuIn->header.stamp.toSec();

        imuRoll[imuPointerLast] = roll;
        imuPitch[imuPointerLast] = pitch;
        imuYaw[imuPointerLast] = yaw;

        imuAccX[imuPointerLast] = accX;
        imuAccY[imuPointerLast] = accY;
        imuAccZ[imuPointerLast] = accZ;

        imuAngularVeloX[imuPointerLast] = imuIn->angular_velocity.x;
        imuAngularVeloY[imuPointerLast] = imuIn->angular_velocity.y;
        imuAngularVeloZ[imuPointerLast] = imuIn->angular_velocity.z;

        AccumulateIMUShiftAndRotation();


        //std::cout << "****************************************************************" << std::endl;
        //std::cout << "Imu Time:" << imuIn->header.stamp << std::endl;
        //std::cout << "accumulate only imu:\nroll:" << imuAngularRotationX[imuPointerLast]
        //          << ", pitch:" << imuAngularRotationY[imuPointerLast]
         //         << ", yaw:" << imuAngularRotationZ[imuPointerLast] << std::endl;
        //std::cout << "****************************************************************" << std::endl;
        //std::cout << "imu to map:\nroll:" << pose_roll
        //          << ", pitch:" << pose_pitch
        //          << ", yaw:" << pose_yaw << std::endl;

        // std::cout << "x:" << pose_x << ", y:" << pose_y << ", z:" << pose_z << std::endl;
        // geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(pose_roll, pose_pitch, pose_yaw);

        // combinedOdometry.header.stamp = imuIn->header.stamp;
        // combinedOdometry.pose.pose.orientation.x = geoQuat.x;
        // combinedOdometry.pose.pose.orientation.y = geoQuat.y;
        // combinedOdometry.pose.pose.orientation.z = geoQuat.z;
        // combinedOdometry.pose.pose.orientation.w = geoQuat.w;
        // combinedOdometry.pose.pose.position.x = pose_x;
        // combinedOdometry.pose.pose.position.y = pose_y;
        // combinedOdometry.pose.pose.position.z = pose_z;
        // pubCombinedOdometry.publish(combinedOdometry);

        // combinedOdometryTrans.stamp_ = imuIn->header.stamp;
        // combinedOdometryTrans.setRotation(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));
        // combinedOdometryTrans.setOrigin(tf::Vector3(pose_x, pose_y, pose_z));
        // tfBroadcaster_combinedOdometryTrans.sendTransform(combinedOdometryTrans);

        // backwheelOdometry.header.stamp = imuIn->header.stamp;
        // backwheelOdometry.pose.pose.orientation.x = geoQuat.x;
        // backwheelOdometry.pose.pose.orientation.y = geoQuat.y;
        // backwheelOdometry.pose.pose.orientation.z = geoQuat.z;
        // backwheelOdometry.pose.pose.orientation.w = geoQuat.w;
        // double wheel_x = pose_x - car_front * cos(pose_yaw);
        // double wheel_y = pose_y - car_front * sin(pose_yaw);
        // double wheel_z = pose_z - car_up;
        // backwheelOdometry.pose.pose.position.x = wheel_x;
        // backwheelOdometry.pose.pose.position.y = wheel_y;
        // backwheelOdometry.pose.pose.position.z = wheel_z;
        // pubBackwheelOdometry.publish(backwheelOdometry);
        // OpenFile << imuIn->header.stamp << " ";
        //     OpenFile <<wheel_x<< " "
        //         << wheel_y << " "
        //         << wheel_z << " "
        //         << geoQuat.w << " "
        //         << geoQuat.x << " "
        //         << geoQuat.y << " "
        //         << geoQuat.z << endl;
        // backwheelOdometryTrans.stamp_ = imuIn->header.stamp;
        // backwheelOdometryTrans.setRotation(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));
        // backwheelOdometryTrans.setOrigin(tf::Vector3(wheel_x, wheel_y, wheel_z));
        // tfBroadcaster_backwheelOdometryTrans.sendTransform(backwheelOdometryTrans);
        }
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lego_loc");

    TransformFusion2 TFusion;

    ROS_INFO("\033[1;32m---->\033[0m Transform Fusion Started.");

    ros::spin();
OpenFile.close();
    return 0;
}


