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
//   T. Shan and B. Englot. LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain
//      IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). October 2018.
#include "utilityLidar.h"
#include <omp.h>
#include <mutex>
ofstream OpenFile;
string fileDirectory;
float deltaR = 0.0,deltaT = 0.0;
class mapOptimization{

private:

    ros::NodeHandle nh;

    ros::Publisher pubOdomAftMapped;
    ros::Publisher pubKeyPoses;
    ros::Publisher pubRegisteredCloud;

    ros::Subscriber subLaserCloudCornerLast;
    ros::Subscriber subLaserCloudSurfLast;
    ros::Subscriber subOutlierCloudLast;
    ros::Subscriber subLaserOdometry;
    ros::Subscriber subImu;
    ros::Subscriber subControlPose;

    ros::Subscriber subSurfaceMap;
    ros::Subscriber subCornerMap;
    bool openFlag, openFlag1, openFlag2, openFlag3;

    pcl::PointCloud<PointType>::Ptr surfaceMap;
    pcl::PointCloud<PointType>::Ptr cornerMap;

    sensor_msgs::PointCloud2 msgSurfaceMap;
    sensor_msgs::PointCloud2 msgCornerMap;

    nav_msgs::Odometry odomAftMapped;
    tf::StampedTransform aftMappedTrans;
    tf::TransformBroadcaster tfBroadcaster;
    
    PointType previousRobotPosPoint;
    PointType currentRobotPosPoint;

    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;


    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast; // corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast; // surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS; // downsampled corner featuer set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS; // downsampled surf featuer set from odoOptimization

    pcl::PointCloud<PointType>::Ptr laserCloudOutlierLast; // corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudOutlierLastDS; // corner feature set from odoOptimization

    pcl::PointCloud<PointType>::Ptr laserCloudSurfTotalLast; // surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfTotalLastDS; // downsampled corner featuer set from odoOptimization

    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;

    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;


    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;

    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterOutlier;

    double timeLaserCloudCornerLast;
    double timeLaserCloudSurfLast;
    double timeLaserOdometry;
    double timeLaserCloudOutlierLast;
    double timeLastGloalMapPublish;

    bool newLaserCloudCornerLast;
    bool newLaserCloudSurfLast;
    bool newLaserOdometry;
    bool newLaserCloudOutlierLast;


    float transformLast[6];
    float transformSum[6];
    float transformIncre[6];
    float transformTobeMapped[6];
    float transformBefMapped[6];
    float transformAftMapped[6];


    int imuPointerFront;
    int imuPointerLast;

    double imuTime[imuQueLength];
    float imuRoll[imuQueLength];
    float imuPitch[imuQueLength];

    double timeLastProcessing;

    PointType pointOri, pointSel, pointProj, coeff;

    // cv::Mat matA0;
    // cv::Mat matB0;
    // cv::Mat matX0;

    // cv::Mat matA1;
    // cv::Mat matD1;
    // cv::Mat matV1;

    bool isDegenerate;
    cv::Mat matP;

    int laserCloudCornerFromMapDSNum;
    int laserCloudSurfFromMapDSNum;
    int laserCloudCornerLastDSNum;
    int laserCloudSurfLastDSNum;
    int laserCloudOutlierLastDSNum;
    int laserCloudSurfTotalLastDSNum;


    float cRoll, sRoll, cPitch, sPitch, cYaw, sYaw, tX, tY, tZ;
    float ctRoll, stRoll, ctPitch, stPitch, ctYaw, stYaw, tInX, tInY, tInZ;
    //初始位姿
    float x_op, y_op, z_op, roll_op, pitch_op, yaw_op;

    std::mutex m;
    std::mutex init_mutex;
    bool restart;
    ros::Subscriber reset_point_sub;
    bool USECUTMAP = false;
public:
    
    

    mapOptimization():
        nh("~")
    {

        //pubKeyPoses = nh.advertise<sensor_msgs::PointCloud2>("/key_pose_origin", 2);
        pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> ("/aft_mapped_to_init", 5);

        openFlag = openFlag1 = openFlag2 = openFlag3 = false;
        openFlag1 = true;
        subSurfaceMap = nh.subscribe<sensor_msgs::PointCloud2>("/static_surface_map", 1, &mapOptimization::surfaceMapHandler, this);
        subCornerMap = nh.subscribe<sensor_msgs::PointCloud2>("/static_corner_map", 1, &mapOptimization::cornerMapHandler, this);
        subLaserCloudCornerLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 2, &mapOptimization::laserCloudCornerLastHandler, this);
        subLaserCloudSurfLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 2, &mapOptimization::laserCloudSurfLastHandler, this);
        subOutlierCloudLast = nh.subscribe<sensor_msgs::PointCloud2>("/outlier_cloud_last", 2, &mapOptimization::laserCloudOutlierLastHandler, this);
        subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/laser_odom_to_init", 5, &mapOptimization::laserOdometryHandler, this);
        subImu = nh.subscribe<sensor_msgs::Imu> (imuTopic, 50, &mapOptimization::imuHandler, this);
        reset_point_sub = nh.subscribe("/reset_point", 1, &mapOptimization::reset_point_callback, this);
        // reset_point_sub = nh.subscribe("/initialpose", 1, &mapOptimization::reset_point_callback, this);
        
        //subControlPose = nh.subscribe<nav_msgs::Odometry>("/control_pose", 2, &mapOptimization::controlPoseHandler, this);
        //pubRegisteredCloud = nh.advertise<sensor_msgs::PointCloud2>("/registered_cloud", 2);
        nh.param<float>("/x_op",x_op,0.0);
        nh.param<float>("/y_op",y_op,0.0);
        nh.param<float>("/z_op",z_op,0.0);
        nh.param<float>("/roll_op",roll_op,0.0);
        nh.param<float>("/pitch_op",pitch_op,0.0);
        nh.param<float>("/yaw_op",yaw_op,0.0);
        nh.param<bool>("/use_cut_map",USECUTMAP,false);

        std::cout << "\n-----------------------initial pose:------------------\n"
                  << x_op << ", " << y_op << ", " << z_op << ", " << yaw_op << std::endl;
        //private_nh.param<string>("cornerMapFilename",cornerMapFilename,"");
        
        downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);
        downSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);
        downSizeFilterOutlier.setLeafSize(0.4, 0.4, 0.4);

        // set by xinkai , frame_id = "/camera_init"
        odomAftMapped.header.frame_id = "/static_cloud_map";
        odomAftMapped.child_frame_id = "/aft_mapped";
        // set by xinkai , frame_id = "/camera_init"
        aftMappedTrans.frame_id_ = "/static_cloud_map";
        aftMappedTrans.child_frame_id_ = "/aft_mapped";
	nh.param<string>("/fileDirectory",fileDirectory,"/home/luoluolll");
	OpenFile.open(fileDirectory+"delta.txt");
	OpenFile <<"dR"<< " "<<"dT"<<endl;
        if(!USECUTMAP)
        {
            laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
            laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());
            cornerMap.reset(new pcl::PointCloud<PointType>());
            surfaceMap.reset(new pcl::PointCloud<PointType>());
            laserCloudCornerFromMapDSNum = 0;
            laserCloudSurfFromMapDSNum = 0;
            cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
            cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
        }
        restart = false;
        allocateMemory();

    }

    void allocateMemory(){

        if(USECUTMAP)
        {
            laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
            laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());
            cornerMap.reset(new pcl::PointCloud<PointType>());
            surfaceMap.reset(new pcl::PointCloud<PointType>());
            laserCloudCornerFromMapDSNum = 0;
            laserCloudSurfFromMapDSNum = 0;
            cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
            cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
        }

     

        laserCloudCornerLast.reset(new pcl::PointCloud<PointType>()); // corner feature set from odoOptimization
        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>()); // surf feature set from odoOptimization
        laserCloudCornerLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled corner featuer set from odoOptimization
        laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled surf featuer set from odoOptimization
        laserCloudOutlierLast.reset(new pcl::PointCloud<PointType>()); // corner feature set from odoOptimization
        laserCloudOutlierLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled corner feature set from odoOptimization
        laserCloudSurfTotalLast.reset(new pcl::PointCloud<PointType>()); // surf feature set from odoOptimization
        laserCloudSurfTotalLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled surf featuer set from odoOptimization

        laserCloudOri.reset(new pcl::PointCloud<PointType>());
        coeffSel.reset(new pcl::PointCloud<PointType>());

        kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

        timeLaserCloudCornerLast = 0;
        timeLaserCloudSurfLast = 0;
        timeLaserOdometry = 0;
        timeLaserCloudOutlierLast = 0;
        timeLastGloalMapPublish = 0;

        timeLastProcessing = -1;

        newLaserCloudCornerLast = false;
        newLaserCloudSurfLast = false;

        newLaserOdometry = false;
        newLaserCloudOutlierLast = false;

        transformSum[0] = pitch_op ;
        transformSum[1] = yaw_op ;
        transformSum[2] = roll_op ;
        transformSum[3] = y_op;
        transformSum[4] = z_op;
        transformSum[5] = x_op;

        for (int i = 0; i < 6; ++i){
            transformIncre[i] = 0;
            transformLast[i] = transformSum[i];
            transformTobeMapped[i] = transformSum[i];
            transformBefMapped[i] = transformSum[i];
            transformAftMapped[i] = transformSum[i];
        }

        imuPointerFront = 0;
        imuPointerLast = -1;

        for (int i = 0; i < imuQueLength; ++i){
            imuTime[i] = 0;
            imuRoll[i] = 0;
            imuPitch[i] = 0;
        }


        // matA0 = cv::Mat (5, 3, CV_32F, cv::Scalar::all(0));
        // matB0 = cv::Mat (5, 1, CV_32F, cv::Scalar::all(-1));
        // matX0 = cv::Mat (3, 1, CV_32F, cv::Scalar::all(0));

        // matA1 = cv::Mat (3, 3, CV_32F, cv::Scalar::all(0));
        // matD1 = cv::Mat (1, 3, CV_32F, cv::Scalar::all(0));
        // matV1 = cv::Mat (3, 3, CV_32F, cv::Scalar::all(0));

        isDegenerate = false;
        matP = cv::Mat (6, 6, CV_32F, cv::Scalar::all(0));


        laserCloudCornerLastDSNum = 0;
        laserCloudSurfLastDSNum = 0;
        laserCloudOutlierLastDSNum = 0;
        laserCloudSurfTotalLastDSNum = 0;

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
        transformTobeMapped[0] = -asin(srx);

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
        transformTobeMapped[1] = atan2(srycrx / cos(transformTobeMapped[0]), 
                                       crycrx / cos(transformTobeMapped[0]));
        
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
        transformTobeMapped[2] = atan2(srzcrx / cos(transformTobeMapped[0]), 
                                       crzcrx / cos(transformTobeMapped[0]));

        x1 = cos(transformTobeMapped[2]) * transformIncre[3] - sin(transformTobeMapped[2]) * transformIncre[4];
        y1 = sin(transformTobeMapped[2]) * transformIncre[3] + cos(transformTobeMapped[2]) * transformIncre[4];
        z1 = transformIncre[5];

        x2 = x1;
        y2 = cos(transformTobeMapped[0]) * y1 - sin(transformTobeMapped[0]) * z1;
        z2 = sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

        transformTobeMapped[3] = transformAftMapped[3] 
                               - (cos(transformTobeMapped[1]) * x2 + sin(transformTobeMapped[1]) * z2);
        transformTobeMapped[4] = transformAftMapped[4] - y2;
        transformTobeMapped[5] = transformAftMapped[5] 
                               - (-sin(transformTobeMapped[1]) * x2 + cos(transformTobeMapped[1]) * z2);

    }

    void transformUpdate()
    {
		if (imuPointerLast >= 0) {
		    float imuRollLast = 0, imuPitchLast = 0;
		    while (imuPointerFront != imuPointerLast) {
		        if (timeLaserOdometry + scanPeriod < imuTime[imuPointerFront]) {
		            break;
		        }
		        imuPointerFront = (imuPointerFront + 1) % imuQueLength;
		    }

		    if (timeLaserOdometry + scanPeriod > imuTime[imuPointerFront]) {
		        imuRollLast = imuRoll[imuPointerFront];
		        imuPitchLast = imuPitch[imuPointerFront];
		    } else {
		        int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
		        float ratioFront = (timeLaserOdometry + scanPeriod - imuTime[imuPointerBack]) 
		                         / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
		        float ratioBack = (imuTime[imuPointerFront] - timeLaserOdometry - scanPeriod) 
		                        / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);

		        imuRollLast = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;
		        imuPitchLast = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;
		    }

		    transformTobeMapped[0] = 0.998 * transformTobeMapped[0] + 0.002 * imuPitchLast;
		    transformTobeMapped[2] = 0.998 * transformTobeMapped[2] + 0.002 * imuRollLast;
		  }

		for (int i = 0; i < 6; i++) {
		    transformBefMapped[i] = transformSum[i];
		    transformAftMapped[i] = transformTobeMapped[i];
		}
    }

    void updatePointAssociateToMapSinCos(){
        cRoll = cos(transformTobeMapped[0]);
        sRoll = sin(transformTobeMapped[0]);

        cPitch = cos(transformTobeMapped[1]);
        sPitch = sin(transformTobeMapped[1]);

        cYaw = cos(transformTobeMapped[2]);
        sYaw = sin(transformTobeMapped[2]);

        tX = transformTobeMapped[3];
        tY = transformTobeMapped[4];
        tZ = transformTobeMapped[5];
    }

    void pointAssociateToMap(PointType const * const pi, PointType * const po)
    {
        float x1 = cYaw * pi->x - sYaw * pi->y;
        float y1 = sYaw * pi->x + cYaw * pi->y;
        float z1 = pi->z;

        float x2 = x1;
        float y2 = cRoll * y1 - sRoll * z1;
        float z2 = sRoll * y1 + cRoll * z1;

        po->x = cPitch * x2 + sPitch * z2 + tX;
        po->y = y2 + tY;
        po->z = -sPitch * x2 + cPitch * z2 + tZ;
        po->intensity = pi->intensity;
    }

    void updateTransformPointCloudSinCos(PointTypePose *tIn){

        ctRoll = cos(tIn->roll);
        stRoll = sin(tIn->roll);

        ctPitch = cos(tIn->pitch);
        stPitch = sin(tIn->pitch);

        ctYaw = cos(tIn->yaw);
        stYaw = sin(tIn->yaw);

        tInX = tIn->x;
        tInY = tIn->y;
        tInZ = tIn->z;
    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn){
	// !!! DO NOT use pcl for point cloud transformation, results are not accurate
        // Reason: unkown
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        PointType *pointFrom;
        PointType pointTo;

        int cloudSize = cloudIn->points.size();
        cloudOut->resize(cloudSize);

        for (int i = 0; i < cloudSize; ++i){

            pointFrom = &cloudIn->points[i];
            float x1 = ctYaw * pointFrom->x - stYaw * pointFrom->y;
            float y1 = stYaw * pointFrom->x + ctYaw* pointFrom->y;
            float z1 = pointFrom->z;

            float x2 = x1;
            float y2 = ctRoll * y1 - stRoll * z1;
            float z2 = stRoll * y1 + ctRoll* z1;

            pointTo.x = ctPitch * x2 + stPitch * z2 + tInX;
            pointTo.y = y2 + tInY;
            pointTo.z = -stPitch * x2 + ctPitch * z2 + tInZ;
            pointTo.intensity = pointFrom->intensity;

            cloudOut->points[i] = pointTo;
        }
        return cloudOut;
    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn){

        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        PointType *pointFrom;
        PointType pointTo;

        int cloudSize = cloudIn->points.size();
        cloudOut->resize(cloudSize);
        
        for (int i = 0; i < cloudSize; ++i){

            pointFrom = &cloudIn->points[i];
            float x1 = cos(transformIn->yaw) * pointFrom->x - sin(transformIn->yaw) * pointFrom->y;
            float y1 = sin(transformIn->yaw) * pointFrom->x + cos(transformIn->yaw)* pointFrom->y;
            float z1 = pointFrom->z;

            float x2 = x1;
            float y2 = cos(transformIn->roll) * y1 - sin(transformIn->roll) * z1;
            float z2 = sin(transformIn->roll) * y1 + cos(transformIn->roll)* z1;

            pointTo.x = cos(transformIn->pitch) * x2 + sin(transformIn->pitch) * z2 + transformIn->x;
            pointTo.y = y2 + transformIn->y;
            pointTo.z = -sin(transformIn->pitch) * x2 + cos(transformIn->pitch) * z2 + transformIn->z;
            pointTo.intensity = pointFrom->intensity;

            cloudOut->points[i] = pointTo;
        }
        return cloudOut;
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

    void surfaceMapHandler(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        if(openFlag2==true && !USECUTMAP)
            return;
        pcl::fromROSMsg(*msg, *surfaceMap);

        // Downsample the surrounding surf key frames (or map)
        downSizeFilterSurf.setInputCloud(surfaceMap);
        downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
        //*laserCloudSurfFromMapDS = *surfaceMap;
        laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->points.size();

        openFlag2 = true;
        openFlag = openFlag1 && openFlag2 && openFlag3;
    }

    void cornerMapHandler(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        if(openFlag3==true && !USECUTMAP)
            return;
        pcl::fromROSMsg(*msg, *cornerMap);

        // Downsample the surrounding corner key frames (or map)
        downSizeFilterCorner.setInputCloud(cornerMap);
        downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
        //*laserCloudCornerFromMapDS = *cornerMap;
        laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->points.size();
        
        openFlag3 = true;
        openFlag = openFlag1 && openFlag2 && openFlag3;
    }

    void controlPoseHandler(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // TODO 
    }

    void laserCloudOutlierLastHandler(const sensor_msgs::PointCloud2ConstPtr& msg){
        timeLaserCloudOutlierLast = msg->header.stamp.toSec();
        laserCloudOutlierLast->clear();
        pcl::fromROSMsg(*msg, *laserCloudOutlierLast);
        newLaserCloudOutlierLast = true;
    }

    void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr& msg){
        timeLaserCloudCornerLast = msg->header.stamp.toSec();
        laserCloudCornerLast->clear();
        pcl::fromROSMsg(*msg, *laserCloudCornerLast);
        newLaserCloudCornerLast = true;
    }

    void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr& msg){
        timeLaserCloudSurfLast = msg->header.stamp.toSec();
        laserCloudSurfLast->clear();
        pcl::fromROSMsg(*msg, *laserCloudSurfLast);
        newLaserCloudSurfLast = true;
    }

    void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry){
        timeLaserOdometry = laserOdometry->header.stamp.toSec();
        double roll, pitch, yaw;
        geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
        tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);
        // set by xinkai, init for original pose of robot
        transformSum[0] = -pitch + pitch_op ;
        transformSum[1] = -yaw + yaw_op ;
        transformSum[2] = roll + roll_op ;
        double dy = laserOdometry->pose.pose.position.x;
        double dz = laserOdometry->pose.pose.position.y;
        double dx = laserOdometry->pose.pose.position.z;
        transformSum[3] = y_op + dy * cos(yaw_op) + dx * sin(yaw_op);
        transformSum[4] = z_op + dz;
        transformSum[5] = x_op + dx * cos(yaw_op) - dy * sin(yaw_op);
        newLaserOdometry = true;
    }

    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn){
        double roll, pitch, yaw;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(imuIn->orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        imuPointerLast = (imuPointerLast + 1) % imuQueLength;
        imuTime[imuPointerLast] = imuIn->header.stamp.toSec();imuTime[imuPointerLast] = imuIn->header.stamp.toSec();
//TODO
        imuRoll[imuPointerLast] = roll;
        imuPitch[imuPointerLast] = pitch;

    }

    void publishTF(){

        geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw
                                  (transformAftMapped[2], -transformAftMapped[0], -transformAftMapped[1]);

        odomAftMapped.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        odomAftMapped.pose.pose.orientation.x = -geoQuat.y;
        odomAftMapped.pose.pose.orientation.y = -geoQuat.z;
        odomAftMapped.pose.pose.orientation.z = geoQuat.x;
        odomAftMapped.pose.pose.orientation.w = geoQuat.w;
        odomAftMapped.pose.pose.position.x = transformAftMapped[3];
        odomAftMapped.pose.pose.position.y = transformAftMapped[4];
        odomAftMapped.pose.pose.position.z = transformAftMapped[5];
        odomAftMapped.twist.twist.angular.x = transformBefMapped[0];
        odomAftMapped.twist.twist.angular.y = transformBefMapped[1];
        odomAftMapped.twist.twist.angular.z = transformBefMapped[2];
        odomAftMapped.twist.twist.linear.x = transformBefMapped[3];
        odomAftMapped.twist.twist.linear.y = transformBefMapped[4];
        odomAftMapped.twist.twist.linear.z = transformBefMapped[5];
        pubOdomAftMapped.publish(odomAftMapped);

        aftMappedTrans.stamp_ = ros::Time().fromSec(timeLaserOdometry);
        aftMappedTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
        aftMappedTrans.setOrigin(tf::Vector3(transformAftMapped[3], transformAftMapped[4], transformAftMapped[5]));
        tfBroadcaster.sendTransform(aftMappedTrans);
    }

    PointTypePose trans2PointTypePose(float transformIn[]){
        PointTypePose thisPose6D;
        thisPose6D.x = transformIn[3];
        thisPose6D.y = transformIn[4];
        thisPose6D.z = transformIn[5];
        thisPose6D.roll  = transformIn[0];
        thisPose6D.pitch = transformIn[1];
        thisPose6D.yaw   = transformIn[2];
        return thisPose6D;
    }

    void publishKeyPosesAndFrames(){

        /*
        if (pubKeyPoses.getNumSubscribers() != 0){
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*cloudKeyPoses3D, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
            // set by xinkai , frame_id = "/camera_init"
            cloudMsgTemp.header.frame_id = "/static_cloud_map";   
            pubKeyPoses.publish(cloudMsgTemp);
        }
        
        if (pubRegisteredCloud.getNumSubscribers() != 0){
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut += *transformPointCloud(laserCloudCornerLastDS,  &thisPose6D);
            *cloudOut += *transformPointCloud(laserCloudSurfTotalLast, &thisPose6D);
            
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*cloudOut, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry); 
            // set by xinkai , frame_id = "/camera_init"
            cloudMsgTemp.header.frame_id = "/static_cloud_map";
            pubRegisteredCloud.publish(cloudMsgTemp);
        } 
        */
        
    }

    void downsampleCurrentScan(){

        laserCloudCornerLastDS->clear();
        //*laserCloudCornerLastDS = *laserCloudCornerLast;
        downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
        downSizeFilterCorner.filter(*laserCloudCornerLastDS);
        laserCloudCornerLastDSNum = laserCloudCornerLastDS->points.size();

        laserCloudSurfLastDS->clear();
        //*laserCloudSurfLastDS = *laserCloudSurfLast;
        downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
        downSizeFilterSurf.filter(*laserCloudSurfLastDS);
        laserCloudSurfLastDSNum = laserCloudSurfLastDS->points.size();

        laserCloudOutlierLastDS->clear();
        //*laserCloudOutlierLastDS = *laserCloudOutlierLast;
        downSizeFilterOutlier.setInputCloud(laserCloudOutlierLast);
        downSizeFilterOutlier.filter(*laserCloudOutlierLastDS);
        laserCloudOutlierLastDSNum = laserCloudOutlierLastDS->points.size();

        laserCloudSurfTotalLast->clear();
        laserCloudSurfTotalLastDS->clear();
        *laserCloudSurfTotalLast += *laserCloudSurfLastDS;
        *laserCloudSurfTotalLast += *laserCloudOutlierLastDS;
        *laserCloudSurfTotalLastDS = *laserCloudSurfTotalLast;
        laserCloudSurfTotalLastDSNum = laserCloudSurfTotalLastDS->points.size();
    }

    void cornerOptimization(int iterCount){

        updatePointAssociateToMapSinCos();
#pragma omp parallel for num_threads(2)
        for (int i = 0; i < laserCloudCornerLastDSNum; i++) {
            PointType pointOri_cor = laserCloudCornerLastDS->points[i];
            PointType coeff_cor, pointSel_cor;
            pointAssociateToMap(&pointOri_cor, &pointSel_cor);
            std::vector<int> pointSearchInd_1;
            std::vector<float> pointSearchSqDis_1;
            cv::Mat matA1 = cv::Mat (3, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matD1 = cv::Mat (1, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matV1 = cv::Mat (3, 3, CV_32F, cv::Scalar::all(0));
            kdtreeCornerFromMap->nearestKSearch(pointSel_cor, 5, pointSearchInd_1, pointSearchSqDis_1);
            
            if (pointSearchSqDis_1[4] < 1.0) {
                float cx = 0, cy = 0, cz = 0;
                for (int j = 0; j < 5; j++) {
                    cx += laserCloudCornerFromMapDS->points[pointSearchInd_1[j]].x;
                    cy += laserCloudCornerFromMapDS->points[pointSearchInd_1[j]].y;
                    cz += laserCloudCornerFromMapDS->points[pointSearchInd_1[j]].z;
                }
                cx /= 5; cy /= 5;  cz /= 5;

                float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
                for (int j = 0; j < 5; j++) {
                    float ax = laserCloudCornerFromMapDS->points[pointSearchInd_1[j]].x - cx;
                    float ay = laserCloudCornerFromMapDS->points[pointSearchInd_1[j]].y - cy;
                    float az = laserCloudCornerFromMapDS->points[pointSearchInd_1[j]].z - cz;

                    a11 += ax * ax; a12 += ax * ay; a13 += ax * az;
                    a22 += ay * ay; a23 += ay * az;
                    a33 += az * az;
                }
                a11 /= 5; a12 /= 5; a13 /= 5; a22 /= 5; a23 /= 5; a33 /= 5;
            

                matA1.at<float>(0, 0) = a11; matA1.at<float>(0, 1) = a12; matA1.at<float>(0, 2) = a13;
                matA1.at<float>(1, 0) = a12; matA1.at<float>(1, 1) = a22; matA1.at<float>(1, 2) = a23;
                matA1.at<float>(2, 0) = a13; matA1.at<float>(2, 1) = a23; matA1.at<float>(2, 2) = a33;

                cv::eigen(matA1, matD1, matV1);

                if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) {

                    float x0 = pointSel_cor.x;
                    float y0 = pointSel_cor.y;
                    float z0 = pointSel_cor.z;
                    float x1 = cx + 0.1 * matV1.at<float>(0, 0);
                    float y1 = cy + 0.1 * matV1.at<float>(0, 1);
                    float z1 = cz + 0.1 * matV1.at<float>(0, 2);
                    float x2 = cx - 0.1 * matV1.at<float>(0, 0);
                    float y2 = cy - 0.1 * matV1.at<float>(0, 1);
                    float z2 = cz - 0.1 * matV1.at<float>(0, 2);

                    float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                    * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                                    + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                                    * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                                    + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
                                    * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

                    float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

                    float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                              + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

                    float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                               - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                    float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                               + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                    float ld2 = a012 / l12;

                    float s = 1 - 0.9 * fabs(ld2);

                    coeff_cor.x = s * la;
                    coeff_cor.y = s * lb;
                    coeff_cor.z = s * lc;
                    coeff_cor.intensity = s * ld2;
                    //std::cout<<"__1__"<<std::endl;
                    if (s > 0.1) {
                        m.lock();
                        laserCloudOri->push_back(pointOri_cor);
                        coeffSel->push_back(coeff_cor);
                        m.unlock();
                    //std::cout<<"__2__"<<std::endl;
                       
                    }
                    //printf("i = %d, I am Thread %d\n", i, omp_get_thread_num());

                }
            }
        }
    }

    void surfOptimization(int iterCount){

        updatePointAssociateToMapSinCos();
#pragma omp parallel for num_threads(2)
        for (int i = 0; i < laserCloudSurfTotalLastDSNum; i++) {
            PointType pointOri_s = laserCloudSurfTotalLastDS->points[i];
            PointType coeff_s, pointSel_s;
            pointAssociateToMap(&pointOri_s, &pointSel_s); 
            std::vector<int> pointSearchInd_2;
            std::vector<float> pointSearchSqDis_2;
            cv::Mat matA0 = cv::Mat (5, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matB0 = cv::Mat (5, 1, CV_32F, cv::Scalar::all(-1));
            cv::Mat matX0 = cv::Mat (3, 1, CV_32F, cv::Scalar::all(0));

            kdtreeSurfFromMap->nearestKSearch(pointSel_s, 5, pointSearchInd_2, pointSearchSqDis_2);

            if (pointSearchSqDis_2[4] < 1.0) {
                for (int j = 0; j < 5; j++) {
                    matA0.at<float>(j, 0) = laserCloudSurfFromMapDS->points[pointSearchInd_2[j]].x;
                    matA0.at<float>(j, 1) = laserCloudSurfFromMapDS->points[pointSearchInd_2[j]].y;
                    matA0.at<float>(j, 2) = laserCloudSurfFromMapDS->points[pointSearchInd_2[j]].z;
                }
                cv::solve(matA0, matB0, matX0, cv::DECOMP_QR);

                float pa = matX0.at<float>(0, 0);
                float pb = matX0.at<float>(1, 0);
                float pc = matX0.at<float>(2, 0);
                float pd = 1;

                float ps = sqrt(pa * pa + pb * pb + pc * pc);
                pa /= ps; pb /= ps; pc /= ps; pd /= ps;

                bool planeValid = true;
                for (int j = 0; j < 5; j++) {
                    if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd_2[j]].x +
                             pb * laserCloudSurfFromMapDS->points[pointSearchInd_2[j]].y +
                             pc * laserCloudSurfFromMapDS->points[pointSearchInd_2[j]].z + pd) > 0.2) {
                        planeValid = false;
                        break;
                    }
                }

                if (planeValid) {
                    float pd2 = pa * pointSel_s.x + pb * pointSel_s.y + pc * pointSel_s.z + pd;

                    float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel_s.x * pointSel_s.x
                            + pointSel_s.y * pointSel_s.y + pointSel_s.z * pointSel_s.z));

                    coeff_s.x = s * pa;
                    coeff_s.y = s * pb;
                    coeff_s.z = s * pc;
                    coeff_s.intensity = s * pd2;

                    if (s > 0.1) {
                        m.lock();
                        laserCloudOri->push_back(pointOri_s);
                        coeffSel->push_back(coeff_s);
                        m.unlock();
                    }
                }
            }
        }
    }

    bool LMOptimization(int iterCount){
        float srx = sin(transformTobeMapped[0]);
        float crx = cos(transformTobeMapped[0]);
        float sry = sin(transformTobeMapped[1]);
        float cry = cos(transformTobeMapped[1]);
        float srz = sin(transformTobeMapped[2]);
        float crz = cos(transformTobeMapped[2]);

        int laserCloudSelNum = laserCloudOri->points.size();
        if (laserCloudSelNum < 100) {
            return false;
        }

        cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
#pragma omp parallel for num_threads(2)
        for (int i = 0; i < laserCloudSelNum; i++) {
            PointType pointOri_1 = laserCloudOri->points[i];
            PointType coeff_1 = coeffSel->points[i];

            float arx = (crx*sry*srz*pointOri_1.x + crx*crz*sry*pointOri_1.y - srx*sry*pointOri_1.z) * coeff_1.x
                      + (-srx*srz*pointOri_1.x - crz*srx*pointOri_1.y - crx*pointOri_1.z) * coeff_1.y
                      + (crx*cry*srz*pointOri_1.x + crx*cry*crz*pointOri_1.y - cry*srx*pointOri_1.z) * coeff_1.z;

            float ary = ((cry*srx*srz - crz*sry)*pointOri_1.x 
                      + (sry*srz + cry*crz*srx)*pointOri_1.y + crx*cry*pointOri_1.z) * coeff_1.x
                      + ((-cry*crz - srx*sry*srz)*pointOri_1.x 
                      + (cry*srz - crz*srx*sry)*pointOri_1.y - crx*sry*pointOri_1.z) * coeff_1.z;

            float arz = ((crz*srx*sry - cry*srz)*pointOri_1.x + (-cry*crz-srx*sry*srz)*pointOri_1.y)*coeff_1.x
                      + (crx*crz*pointOri_1.x - crx*srz*pointOri_1.y) * coeff_1.y
                      + ((sry*srz + cry*crz*srx)*pointOri_1.x + (crz*sry-cry*srx*srz)*pointOri_1.y)*coeff_1.z;

            matA.at<float>(i, 0) = arx;
            matA.at<float>(i, 1) = ary;
            matA.at<float>(i, 2) = arz;
            matA.at<float>(i, 3) = coeff_1.x;
            matA.at<float>(i, 4) = coeff_1.y;
            matA.at<float>(i, 5) = coeff_1.z;
            matB.at<float>(i, 0) = -coeff_1.intensity;
           // std::cout<<i<<std::endl;
        }
        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        if (iterCount == 0) {
            cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

            isDegenerate = false;
            float eignThre[6] = {100, 100, 100, 100, 100, 100};
            for (int i = 5; i >= 0; i--) {
                if (matE.at<float>(0, i) < eignThre[i]) {
                    for (int j = 0; j < 6; j++) {
                        matV2.at<float>(i, j) = 0;
                    }
                    isDegenerate = true;
                } else {
                    break;
                }
            }
            matP = matV.inv() * matV2;
        }

        if (isDegenerate) {
            cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;
        }

        transformTobeMapped[0] += matX.at<float>(0, 0);
        transformTobeMapped[1] += matX.at<float>(1, 0);
        transformTobeMapped[2] += matX.at<float>(2, 0);
        transformTobeMapped[3] += matX.at<float>(3, 0);
        transformTobeMapped[4] += matX.at<float>(4, 0);
        transformTobeMapped[5] += matX.at<float>(5, 0);

        deltaR = sqrt(
                            pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
        deltaT = sqrt(
                            pow(matX.at<float>(3, 0) * 100, 2) +
                            pow(matX.at<float>(4, 0) * 100, 2) +
                            pow(matX.at<float>(5, 0) * 100, 2));

            
        if (deltaR < 0.05 && deltaT < 0.05) {
            return true;
        }
        return false;
    }

    void scan2MapOptimization(){
        if (laserCloudCornerFromMapDSNum > 10 && laserCloudSurfFromMapDSNum > 50) {

            kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
            kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

            for (int iterCount = 0; iterCount < 10; iterCount++) {

                laserCloudOri->clear();
                coeffSel->clear();

                cornerOptimization(iterCount);
                surfOptimization(iterCount);

                if (LMOptimization(iterCount) == true)
                    break;              
            }
            transformUpdate();
        }
        if(deltaR>0.1&&deltaT>0.5)
        {
            ROS_INFO("localization miss!");
        }
	OpenFile <<deltaR<<" "<< deltaT <<endl;
    }

    void saveKeyFramesAndFactor(){

        currentRobotPosPoint.x = transformAftMapped[3];
        currentRobotPosPoint.y = transformAftMapped[4];
        currentRobotPosPoint.z = transformAftMapped[5];

        bool saveThisKeyFrame = true;

        previousRobotPosPoint = currentRobotPosPoint;
        
        if (cloudKeyPoses3D->points.empty()){
            for (int i = 0; i < 6; ++i)
            	transformLast[i] = transformTobeMapped[i];
        }
        

        /**
         * save key poses
         */
        PointType thisPose3D;
        PointTypePose thisPose6D;

        thisPose3D.x = currentRobotPosPoint.x;
        thisPose3D.y = currentRobotPosPoint.y;
        thisPose3D.z = currentRobotPosPoint.z;
        thisPose3D.intensity = cloudKeyPoses3D->points.size(); // this can be used as index
        cloudKeyPoses3D->push_back(thisPose3D);

        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.intensity = thisPose3D.intensity; // this can be used as index
        thisPose6D.roll  = transformTobeMapped[0];
        thisPose6D.pitch = transformTobeMapped[1];
        thisPose6D.yaw   = transformTobeMapped[2]; // in camera frame
        thisPose6D.time = timeLaserOdometry;
        cloudKeyPoses6D->push_back(thisPose6D);

        // set by xinkai, output pose
        /*
        std::cout << "\n----------currentRobotPosePoint-----------\n"
                  << currentRobotPosPoint.z
                  << ",  " << currentRobotPosPoint.x
                  << ",  " << currentRobotPosPoint.y
                  << ", " << ros::Time::now()<< std::endl;
        // set by xinkai
        
        {
            std::cout << "\n-----------------curPose6D-----------------\n"
                      << thisPose6D.z
                      << ",  " << thisPose6D.x
                      << ",  " << thisPose6D.y
                      << ",  " << thisPose6D.pitch << std::endl;
        }
        */
        /**
         * save updated transform
         */
        if (cloudKeyPoses3D->points.size() > 1){
            for (int i = 0; i < 6; ++i){
            	transformLast[i] = transformAftMapped[i];
            	transformTobeMapped[i] = transformAftMapped[i];
            }
        }
    }

    void run(){
        double t1=ros::Time::now().toSec();
        if(restart)
        {
            std::lock_guard<std::mutex> lock(init_mutex);
            allocateMemory();
            ROS_INFO("**init_map_state**");
            restart = false;

        }


        if (newLaserCloudCornerLast  && std::abs(timeLaserCloudCornerLast  - timeLaserOdometry) < 0.005 &&
            newLaserCloudSurfLast    && std::abs(timeLaserCloudSurfLast    - timeLaserOdometry) < 0.005 &&
            newLaserCloudOutlierLast && std::abs(timeLaserCloudOutlierLast - timeLaserOdometry) < 0.005 &&
            newLaserOdometry)
        {

            newLaserCloudCornerLast = false; newLaserCloudSurfLast = false; newLaserCloudOutlierLast = false; newLaserOdometry = false;

            if (timeLaserOdometry - timeLastProcessing >= 0.08) {

                timeLastProcessing = timeLaserOdometry;

                transformAssociateToMap();

                downsampleCurrentScan();

                scan2MapOptimization();

                saveKeyFramesAndFactor();

                publishTF();

                // publishKeyPosesAndFrames();

            }
        }
        double t2=ros::Time::now().toSec();
        //std::cout<<"opt_time = "<<t2-t1<<", openflag = "<<openFlag<<std::endl;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lego_loc");

    ROS_INFO("\033[1;32m---->\033[0m Map Optimization Started.");

    mapOptimization MO;

    ros::Rate rate(50);
    while (ros::ok())
    {
        ros::spinOnce();

        MO.run();

        rate.sleep();
    }
    OpenFile.close();
    return 0;
}



