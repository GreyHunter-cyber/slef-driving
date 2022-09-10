#include "utilityLidar.h"
#include <pcl/filters/extract_indices.h>

class StaticMap
{
private:
    ros::NodeHandle nh;
    ros::Publisher pubWholeMap;
    ros::Publisher pubSurfaceMap;
    ros::Publisher pubCornerMap;
    ros::Subscriber MapOdometry_sub;
    bool openFlag;

    pcl::PointCloud<PointType>::Ptr wholeMap;
    pcl::PointCloud<PointType>::Ptr wholeMap_filtered;
    pcl::PointCloud<PointType>::Ptr surfaceMap;
    pcl::PointCloud<PointType>::Ptr cornerMap;

    pcl::PointCloud<PointType>::Ptr surfaceMapFiltered;
    pcl::PointCloud<PointType>::Ptr cornerMapFiltered;
    pcl::VoxelGrid<PointType> downSizeFilter;
    pcl::PointCloud<PointType>::Ptr surfaceMapDS;
    pcl::PointCloud<PointType>::Ptr cornerMapDS;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;
    PointType pos_point;

    sensor_msgs::PointCloud2 msgWholeMap;
    sensor_msgs::PointCloud2 msgSurfaceMap;
    sensor_msgs::PointCloud2 msgCornerMap;

    tf::StampedTransform map_2_camera_init_Trans;
    tf::TransformBroadcaster tfBroadcasterMap2CameraInit;
    string cornerMapFilename;
    string surfaceMapFilename;
    string finalMapFilename;
    string fileDirectory;
    //初始位姿
    float x_op, y_op, z_op, roll_op, pitch_op, yaw_op;
    int searchRadius = 60;
    bool USECUTMAP = false;
public:
    bool isOpenOK()
    {
        return openFlag;
    }

    StaticMap():
        nh("~")
        {
            openFlag = false;
            bool flag1 = false, flag2 = false, flag3 = false;
            pos_point.x = 0;
            pos_point.y = 0;
            pos_point.z = 0;
            wholeMap.reset(new pcl::PointCloud<PointType>());
            // wholeMap_filtered.reset(new pcl::PointCloud<PointType>());
            cornerMap.reset(new pcl::PointCloud<PointType>());
            surfaceMap.reset(new pcl::PointCloud<PointType>());

            cornerMapDS.reset(new pcl::PointCloud<PointType>());
            surfaceMapDS.reset(new pcl::PointCloud<PointType>());

            surfaceMapFiltered.reset(new pcl::PointCloud<PointType>());
            cornerMapFiltered.reset(new pcl::PointCloud<PointType>());
    
            kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
            kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());    

            nh.param<string>("/fileDirectory",fileDirectory,"/home/luoluolll");
            MapOdometry_sub = nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init", 10, &StaticMap::MapOdometryHandler, this);
            downSizeFilter.setLeafSize(0.5, 0.5, 0.5);

            nh.param<float>("/x_op",x_op,0.0);
            nh.param<float>("/y_op",y_op,0.0);
            nh.param<float>("/z_op",z_op,0.0);
            nh.param<float>("/roll_op",roll_op,0.0);
            nh.param<float>("/pitch_op",pitch_op,0.0);
            nh.param<float>("/yaw_op",yaw_op,0.0);
            nh.param<bool>("/use_cut_map",USECUTMAP,false);
            nh.param<int>("/search_radius",searchRadius,60);
            std::cout << "\n-----------------------initial pose:------------------\n"
                  << x_op << ", " << y_op << ", " << z_op << ", " << yaw_op << std::endl;
            
            getMapFilename();
            if (pcl::io::loadPCDFile<PointType>(finalMapFilename, *wholeMap) == -1)
                PCL_ERROR("open GridsCloud.pcd error\n");
            else
                flag1 = true;
            
            flag1 = true;

            
            std::cout << surfaceMapFilename << std::endl;
            std::cout << cornerMapFilename << std::endl;

            if (pcl::io::loadPCDFile<PointType>(surfaceMapFilename, *surfaceMap) == -1)
                PCL_ERROR("open surfaceMap.pcd error\n");
            else
                flag2 = true;

            if(pcl::io::loadPCDFile<PointType>(cornerMapFilename, *cornerMap) == -1)
                PCL_ERROR("open cornerMap.pcd error\n");
            else
                flag3 = true;

            openFlag = flag1 && flag2 && flag3;

            if(openFlag)
            {
                pubWholeMap = nh.advertise<sensor_msgs::PointCloud2>("/static_Grids_map", 1);
                pubSurfaceMap = nh.advertise<sensor_msgs::PointCloud2>("/static_surface_map", 1);
                pubCornerMap = nh.advertise<sensor_msgs::PointCloud2>("/static_corner_map", 1);

                // pcl::RadiusOutlierRemoval<PointType> pcFilter;  //创建滤波器对象
                // pcFilter.setInputCloud(wholeMap);             //设置待滤波的点云
                // pcFilter.setRadiusSearch(0.2);               // 设置搜索半径
                // pcFilter.setMinNeighborsInRadius(9);      // 设置一个内点最少的邻居数目
               // pcFilter.filter(*wholeMap_filtered);        //滤波结果存储到cloud_filtered
               pcl::toROSMsg(*wholeMap, msgWholeMap);
               msgWholeMap.header.frame_id = "/static_cloud_map";
                downSizeFilter.setInputCloud(surfaceMap);
                downSizeFilter.filter(*surfaceMapDS);
                downSizeFilter.setInputCloud(cornerMap);
                downSizeFilter.filter(*cornerMapDS);


            }
//TODO
            kdtreeCornerFromMap->setInputCloud(cornerMap);
            kdtreeSurfFromMap->setInputCloud(surfaceMap);

            // map_2_camera_init_Trans.frame_id_ = "/map";
            // map_2_camera_init_Trans.child_frame_id_ = "/camera_init";
            // tf::Quaternion q = tf::createQuaternionFromRPY( M_PI_2, 0 , M_PI_2);
            // map_2_camera_init_Trans.setOrigin(tf::Vector3(x_op, y_op, z_op));
            // map_2_camera_init_Trans.setRotation(q);

        }

    void getMapFilename()
    {
        //"/home/zxkj/lego_loc/maps/surfaceMap.pcd"
        surfaceMapFilename =fileDirectory + "surfaceMap.pcd";
        //"/home/zxkj/lego_loc/maps/cornerMap.pcd"
        cornerMapFilename =fileDirectory + "cornerMap.pcd";

       finalMapFilename =fileDirectory + "GridsCloud.pcd";
    }
    void MapOdometryHandler(const nav_msgs::Odometry::ConstPtr& MapOdometry)
    {
        pos_point.x = MapOdometry->pose.pose.position.x;
        pos_point.y = MapOdometry->pose.pose.position.y;
        pos_point.z = MapOdometry->pose.pose.position.z;
    }

    void extract_cloud_add(std::vector<int> Ind_, pcl::PointCloud<PointType>::Ptr &cloud, pcl::PointCloud<PointType>::Ptr &cloudFiltered)
    {
        boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(Ind_);
        // Create the filtering object
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        // Extract the inliers
        extract.setInputCloud (cloud);
        extract.setIndices (index_ptr);
        extract.setNegative (false);//如果设为true,可以提取指定index之外的点云
        extract.filter(*cloudFiltered);
    }

    void publishMapCloud()
    {

        // cornerMapFiltered.reset();
        // surfaceMapFiltered.reset();
        std::vector<int> cornerSearchInd_;
        std::vector<int> surfaceSearchInd_;
        std::vector<float> surfaceSearchDis;
        std::vector<float> cornerSearchDis;
    if(USECUTMAP)
    {
        kdtreeCornerFromMap->radiusSearch(pos_point, searchRadius, cornerSearchInd_,cornerSearchDis);
        kdtreeSurfFromMap->radiusSearch(pos_point, searchRadius, surfaceSearchInd_,surfaceSearchDis);

        extract_cloud_add(cornerSearchInd_, cornerMap, cornerMapFiltered);
        extract_cloud_add(surfaceSearchInd_, surfaceMap, surfaceMapFiltered);
    }else
    {
        cornerMapFiltered = cornerMap;
        surfaceMapFiltered = surfaceMap;
    }
    
        pcl::toROSMsg(*cornerMapFiltered, msgCornerMap);
        pcl::toROSMsg(*surfaceMapFiltered, msgSurfaceMap);
        msgSurfaceMap.header.frame_id = "/static_cloud_map";
        msgCornerMap.header.frame_id = "/static_cloud_map";            

        ros::Time t = ros::Time::now();
        msgWholeMap.header.stamp = t;
        msgSurfaceMap.header.stamp = t;
        msgCornerMap.header.stamp = t;
        
        pubWholeMap.publish(msgWholeMap);
        pubSurfaceMap.publish(msgSurfaceMap);
        pubCornerMap.publish(msgCornerMap);

        // map_2_camera_init_Trans.stamp_ = ros::Time::now();
        // tfBroadcasterMap2CameraInit.sendTransform(map_2_camera_init_Trans);

    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lego_loc");

    ROS_INFO("\033[1;32m---->\033[0m Static Maps Started.");

    StaticMap SM;
    if(!SM.isOpenOK())
        return 0;

    ros::Rate rate(2);
    while(ros::ok())
    {
        ros::spinOnce();
        SM.publishMapCloud();
        rate.sleep();
    }
    return 0;
}
