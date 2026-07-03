// This is an advanced implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014. 

// Modifier: Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk


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

#include <cmath>  // 头文件依赖
#include <nav_msgs/Odometry.h>  // 头文件依赖
#include <nav_msgs/Path.h>  // 头文件依赖
#include <geometry_msgs/PoseStamped.h>  // 头文件依赖
#include <pcl/point_cloud.h>  // 头文件依赖
#include <pcl/point_types.h>  // 头文件依赖
#include <pcl/filters/voxel_grid.h>  // 头文件依赖
#include <pcl/kdtree/kdtree_flann.h>  // 头文件依赖
#include <pcl_conversions/pcl_conversions.h>  // 头文件依赖
#include <ros/ros.h>  // 头文件依赖
#include <sensor_msgs/Imu.h>  // 头文件依赖
#include <sensor_msgs/PointCloud2.h>  // 头文件依赖
#include <tf/transform_datatypes.h>  // 头文件依赖
#include <tf/transform_broadcaster.h>  // 头文件依赖
#include <eigen3/Eigen/Dense>  // 头文件依赖
#include <mutex>  // 头文件依赖
#include <queue>  // 头文件依赖

#include "aloam_velodyne/common.h"  // 头文件依赖
#include "aloam_velodyne/tic_toc.h"  // 头文件依赖
#include "lidarFactor.hpp"  // 头文件依赖

#define DISTORTION 0  // 是否启用运动畸变补偿


int corner_correspondence = 0, plane_correspondence = 0;  // 语句:赋值/初始化

constexpr double SCAN_PERIOD = 0.1;  // 扫描周期(s)
constexpr double DISTANCE_SQ_THRESHOLD = 25;  // 匹配距离阈值(平方)
constexpr double NEARBY_SCAN = 2.5;  // 允许的相邻线束跨度

int skipFrameNum = 5;  // 初始化跳帧数
bool systemInited = false;  // 系统初始化标志

double timeCornerPointsSharp = 0;  // 语句:赋值/初始化
double timeCornerPointsLessSharp = 0;  // 语句:赋值/初始化
double timeSurfPointsFlat = 0;  // 语句:赋值/初始化
double timeSurfPointsLessFlat = 0;  // 语句:赋值/初始化
double timeLaserCloudFullRes = 0;  // 语句:赋值/初始化

pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());  // 上一帧角点KD树
pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());  // 上一帧面点KD树

pcl::PointCloud<PointType>::Ptr cornerPointsSharp(new pcl::PointCloud<PointType>());  // 角点(最尖锐)点云
pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp(new pcl::PointCloud<PointType>());  // 角点(次尖锐)点云
pcl::PointCloud<PointType>::Ptr surfPointsFlat(new pcl::PointCloud<PointType>());  // 面点(最平坦)点云
pcl::PointCloud<PointType>::Ptr surfPointsLessFlat(new pcl::PointCloud<PointType>());  // 面点(次平坦)点云

pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());  // 上一帧角点点云
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());  // 上一帧面点点云
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());  // 全分辨率点云

int laserCloudCornerLastNum = 0;  // 上一帧角点点云
int laserCloudSurfLastNum = 0;  // 上一帧面点点云

// Transformation from current frame to world frame
Eigen::Quaterniond q_w_curr(1, 0, 0, 0);  // 当前位姿(世界系)四元数
Eigen::Vector3d t_w_curr(0, 0, 0);  // 当前位置(世界系)平移

// q_curr_last(x, y, z, w), t_curr_last
double para_q[4] = {0, 0, 0, 1};  // 优化参数:四元数(x,y,z,w)
double para_t[3] = {0, 0, 0};  // 优化参数:平移(x,y,z)

Eigen::Map<Eigen::Quaterniond> q_last_curr(para_q);  // 优化参数:四元数(x,y,z,w)
Eigen::Map<Eigen::Vector3d> t_last_curr(para_t);  // 优化参数:平移(x,y,z)

std::queue<sensor_msgs::PointCloud2ConstPtr> cornerSharpBuf;  // 角点sharp缓存队列
std::queue<sensor_msgs::PointCloud2ConstPtr> cornerLessSharpBuf;  // 语句:声明/调用
std::queue<sensor_msgs::PointCloud2ConstPtr> surfFlatBuf;  // 面点flat缓存队列
std::queue<sensor_msgs::PointCloud2ConstPtr> surfLessFlatBuf;  // 语句:声明/调用
std::queue<sensor_msgs::PointCloud2ConstPtr> fullPointsBuf;  // 语句:声明/调用
std::mutex mBuf;  // 消息队列互斥锁

// undistort lidar point
void TransformToStart(PointType const *const pi, PointType *const po)  // 点去畸变:变换到扫描起始
{  // 代码块开始
    //interpolation ratio
    double s;  // 语句:声明/调用
    if (DISTORTION)  // 是否启用运动畸变补偿
        s = (pi->intensity - int(pi->intensity)) / SCAN_PERIOD;  // 扫描周期(s)
    else  // 否则分支
        s = 1.0;  // 语句:赋值/初始化
    //s = 1;
    Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, q_last_curr);  // 上一帧->当前帧旋转(Eigen::Map)
    Eigen::Vector3d t_point_last = s * t_last_curr;  // 上一帧->当前帧平移(Eigen::Map)
    Eigen::Vector3d point(pi->x, pi->y, pi->z);  // 语句:声明/调用
    Eigen::Vector3d un_point = q_point_last * point + t_point_last;  // 语句:赋值/初始化

    po->x = un_point.x();  // 语句:赋值/初始化
    po->y = un_point.y();  // 语句:赋值/初始化
    po->z = un_point.z();  // 语句:赋值/初始化
    po->intensity = pi->intensity;  // 语句:赋值/初始化
}  // 代码块结束

// transform all lidar points to the start of the next frame

void TransformToEnd(PointType const *const pi, PointType *const po)  // 点去畸变:变换到扫描结束
{  // 代码块开始
    // undistort point first
    pcl::PointXYZI un_point_tmp;  // 语句:声明/调用
    TransformToStart(pi, &un_point_tmp);  // 点去畸变:变换到扫描起始

    Eigen::Vector3d un_point(un_point_tmp.x, un_point_tmp.y, un_point_tmp.z);  // 语句:声明/调用
    Eigen::Vector3d point_end = q_last_curr.inverse() * (un_point - t_last_curr);  // 上一帧->当前帧旋转(Eigen::Map)

    po->x = point_end.x();  // 语句:赋值/初始化
    po->y = point_end.y();  // 语句:赋值/初始化
    po->z = point_end.z();  // 语句:赋值/初始化

    //Remove distortion time info
    po->intensity = int(pi->intensity);  // 语句:赋值/初始化
}  // 代码块结束

void laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsSharp2)  // 角点(最尖锐)点云
{  // 代码块开始
    mBuf.lock();  // 消息队列互斥锁
    cornerSharpBuf.push(cornerPointsSharp2);  // 角点(最尖锐)点云
    mBuf.unlock();  // 消息队列互斥锁
}  // 代码块结束

void laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsLessSharp2)  // 角点(次尖锐)点云
{  // 代码块开始
    mBuf.lock();  // 消息队列互斥锁
    cornerLessSharpBuf.push(cornerPointsLessSharp2);  // 角点(次尖锐)点云
    mBuf.unlock();  // 消息队列互斥锁
}  // 代码块结束

void laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsFlat2)  // 面点(最平坦)点云
{  // 代码块开始
    mBuf.lock();  // 消息队列互斥锁
    surfFlatBuf.push(surfPointsFlat2);  // 面点(最平坦)点云
    mBuf.unlock();  // 消息队列互斥锁
}  // 代码块结束

void laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsLessFlat2)  // 面点(次平坦)点云
{  // 代码块开始
    mBuf.lock();  // 消息队列互斥锁
    surfLessFlatBuf.push(surfPointsLessFlat2);  // 面点(次平坦)点云
    mBuf.unlock();  // 消息队列互斥锁
}  // 代码块结束

//receive all point cloud
void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2)  // 全分辨率点云
{  // 代码块开始
    mBuf.lock();  // 消息队列互斥锁
    fullPointsBuf.push(laserCloudFullRes2);  // 全分辨率点云
    mBuf.unlock();  // 消息队列互斥锁
}  // 代码块结束

int main(int argc, char **argv)  // 语句
{  // 代码块开始
    ros::init(argc, argv, "laserOdometry");  // 语句:声明/调用
    ros::NodeHandle nh;  // 语句:声明/调用

    nh.param<int>("mapping_skip_frame", skipFrameNum, 2);  // 初始化跳帧数

    printf("Mapping %d Hz \n", 10 / skipFrameNum);  // 初始化跳帧数

    ros::Subscriber subCornerPointsSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100, laserCloudSharpHandler);  // ROS 订阅器

    ros::Subscriber subCornerPointsLessSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100, laserCloudLessSharpHandler);  // ROS 订阅器

    ros::Subscriber subSurfPointsFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100, laserCloudFlatHandler);  // ROS 订阅器

    ros::Subscriber subSurfPointsLessFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100, laserCloudLessFlatHandler);  // ROS 订阅器

    ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100, laserCloudFullResHandler);  // 全分辨率点云

    ros::Publisher pubLaserCloudCornerLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 100);  // ROS 发布器

    ros::Publisher pubLaserCloudSurfLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 100);  // ROS 发布器

    ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 100);  // ROS 发布器

    ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 100);  // ROS 发布器

    ros::Publisher pubLaserPath = nh.advertise<nav_msgs::Path>("/laser_odom_path", 100);  // ROS 发布器

    nav_msgs::Path laserPath;  // 语句:声明/调用

    int frameCount = 0;  // 语句:赋值/初始化
    ros::Rate rate(100);  // 语句:声明/调用

    while (ros::ok())  // while 循环
    {  // 代码块开始
        ros::spinOnce();  // 语句:声明/调用

        if (!cornerSharpBuf.empty() && !cornerLessSharpBuf.empty() &&  // 角点sharp缓存队列
            !surfFlatBuf.empty() && !surfLessFlatBuf.empty() &&  // 面点flat缓存队列
            !fullPointsBuf.empty())  // 语句
        {  // 代码块开始
            timeCornerPointsSharp = cornerSharpBuf.front()->header.stamp.toSec();  // 角点sharp缓存队列
            timeCornerPointsLessSharp = cornerLessSharpBuf.front()->header.stamp.toSec();  // 语句:赋值/初始化
            timeSurfPointsFlat = surfFlatBuf.front()->header.stamp.toSec();  // 面点flat缓存队列
            timeSurfPointsLessFlat = surfLessFlatBuf.front()->header.stamp.toSec();  // 语句:赋值/初始化
            timeLaserCloudFullRes = fullPointsBuf.front()->header.stamp.toSec();  // 语句:赋值/初始化

            if (timeCornerPointsSharp != timeLaserCloudFullRes ||  // 条件判断
                timeCornerPointsLessSharp != timeLaserCloudFullRes ||  // 语句
                timeSurfPointsFlat != timeLaserCloudFullRes ||  // 语句
                timeSurfPointsLessFlat != timeLaserCloudFullRes)  // 语句
            {  // 代码块开始
                printf("unsync messeage!");  // 语句:声明/调用
                ROS_BREAK();  // 语句:声明/调用
            }  // 代码块结束

            mBuf.lock();  // 消息队列互斥锁
            cornerPointsSharp->clear();  // 角点(最尖锐)点云
            pcl::fromROSMsg(*cornerSharpBuf.front(), *cornerPointsSharp);  // 角点(最尖锐)点云
            cornerSharpBuf.pop();  // 角点sharp缓存队列

            cornerPointsLessSharp->clear();  // 角点(次尖锐)点云
            pcl::fromROSMsg(*cornerLessSharpBuf.front(), *cornerPointsLessSharp);  // 角点(次尖锐)点云
            cornerLessSharpBuf.pop();  // 语句:声明/调用

            surfPointsFlat->clear();  // 面点(最平坦)点云
            pcl::fromROSMsg(*surfFlatBuf.front(), *surfPointsFlat);  // 面点(最平坦)点云
            surfFlatBuf.pop();  // 面点flat缓存队列

            surfPointsLessFlat->clear();  // 面点(次平坦)点云
            pcl::fromROSMsg(*surfLessFlatBuf.front(), *surfPointsLessFlat);  // 面点(次平坦)点云
            surfLessFlatBuf.pop();  // 语句:声明/调用

            laserCloudFullRes->clear();  // 全分辨率点云
            pcl::fromROSMsg(*fullPointsBuf.front(), *laserCloudFullRes);  // 全分辨率点云
            fullPointsBuf.pop();  // 语句:声明/调用
            mBuf.unlock();  // 消息队列互斥锁

            TicToc t_whole;  // 计时(统计耗时)
            // initializing
            if (!systemInited)  // 系统初始化标志
            {  // 代码块开始
                systemInited = true;  // 系统初始化标志
                std::cout << "Initialization finished \n";  // 语句:声明/调用
            }  // 代码块结束
            else  // 否则分支
            {  // 代码块开始
                int cornerPointsSharpNum = cornerPointsSharp->points.size();  // 角点(最尖锐)点云
                int surfPointsFlatNum = surfPointsFlat->points.size();  // 面点(最平坦)点云

                TicToc t_opt;  // 计时(统计耗时)
                for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter)  // for 循环
                {  // 代码块开始
                    corner_correspondence = 0;  // 语句:赋值/初始化
                    plane_correspondence = 0;  // 语句:赋值/初始化

                    //ceres::LossFunction *loss_function = NULL;
                    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);  // 语句:赋值/初始化
                    ceres::LocalParameterization *q_parameterization =  // 语句
                        new ceres::EigenQuaternionParameterization();  // 语句:声明/调用
                    ceres::Problem::Options problem_options;  // 语句:声明/调用

                    ceres::Problem problem(problem_options);  // 语句:声明/调用
                    problem.AddParameterBlock(para_q, 4, q_parameterization);  // 优化参数:四元数(x,y,z,w)
                    problem.AddParameterBlock(para_t, 3);  // 优化参数:平移(x,y,z)

                    pcl::PointXYZI pointSel;  // 语句:声明/调用
                    std::vector<int> pointSearchInd;  // 语句:声明/调用
                    std::vector<float> pointSearchSqDis;  // 语句:声明/调用

                    TicToc t_data;  // 计时(统计耗时)
                    // find correspondence for corner features
                    for (int i = 0; i < cornerPointsSharpNum; ++i)  // 角点(最尖锐)点云
                    {  // 代码块开始
                        TransformToStart(&(cornerPointsSharp->points[i]), &pointSel);  // 角点(最尖锐)点云
                        kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);  // 上一帧角点KD树

                        int closestPointInd = -1, minPointInd2 = -1;  // 语句:赋值/初始化
                        if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)  // 匹配距离阈值(平方)
                        {  // 代码块开始
                            closestPointInd = pointSearchInd[0];  // 语句:赋值/初始化
                            int closestPointScanID = int(laserCloudCornerLast->points[closestPointInd].intensity);  // 上一帧角点点云

                            double minPointSqDis2 = DISTANCE_SQ_THRESHOLD;  // 匹配距离阈值(平方)
                            // search in the direction of increasing scan line
                            for (int j = closestPointInd + 1; j < (int)laserCloudCornerLast->points.size(); ++j)  // 上一帧角点点云
                            {  // 代码块开始
                                // if in the same scan line, continue
                                if (int(laserCloudCornerLast->points[j].intensity) <= closestPointScanID)  // 上一帧角点点云
                                    continue;  // 继续下一轮

                                // if not in nearby scans, end the loop
                                if (int(laserCloudCornerLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))  // 允许的相邻线束跨度
                                    break;  // 跳出循环/分支

                                double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *  // 上一帧角点点云
                                                        (laserCloudCornerLast->points[j].x - pointSel.x) +  // 上一帧角点点云
                                                    (laserCloudCornerLast->points[j].y - pointSel.y) *  // 上一帧角点点云
                                                        (laserCloudCornerLast->points[j].y - pointSel.y) +  // 上一帧角点点云
                                                    (laserCloudCornerLast->points[j].z - pointSel.z) *  // 上一帧角点点云
                                                        (laserCloudCornerLast->points[j].z - pointSel.z);  // 上一帧角点点云

                                if (pointSqDis < minPointSqDis2)  // 条件判断
                                {  // 代码块开始
                                    // find nearer point
                                    minPointSqDis2 = pointSqDis;  // 语句:赋值/初始化
                                    minPointInd2 = j;  // 语句:赋值/初始化
                                }  // 代码块结束
                            }  // 代码块结束

                            // search in the direction of decreasing scan line
                            for (int j = closestPointInd - 1; j >= 0; --j)  // for 循环
                            {  // 代码块开始
                                // if in the same scan line, continue
                                if (int(laserCloudCornerLast->points[j].intensity) >= closestPointScanID)  // 上一帧角点点云
                                    continue;  // 继续下一轮

                                // if not in nearby scans, end the loop
                                if (int(laserCloudCornerLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))  // 允许的相邻线束跨度
                                    break;  // 跳出循环/分支

                                double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *  // 上一帧角点点云
                                                        (laserCloudCornerLast->points[j].x - pointSel.x) +  // 上一帧角点点云
                                                    (laserCloudCornerLast->points[j].y - pointSel.y) *  // 上一帧角点点云
                                                        (laserCloudCornerLast->points[j].y - pointSel.y) +  // 上一帧角点点云
                                                    (laserCloudCornerLast->points[j].z - pointSel.z) *  // 上一帧角点点云
                                                        (laserCloudCornerLast->points[j].z - pointSel.z);  // 上一帧角点点云

                                if (pointSqDis < minPointSqDis2)  // 条件判断
                                {  // 代码块开始
                                    // find nearer point
                                    minPointSqDis2 = pointSqDis;  // 语句:赋值/初始化
                                    minPointInd2 = j;  // 语句:赋值/初始化
                                }  // 代码块结束
                            }  // 代码块结束
                        }  // 代码块结束
                        if (minPointInd2 >= 0) // both closestPointInd and minPointInd2 is valid
                        {  // 代码块开始
                            Eigen::Vector3d curr_point(cornerPointsSharp->points[i].x,  // 角点(最尖锐)点云
                                                       cornerPointsSharp->points[i].y,  // 角点(最尖锐)点云
                                                       cornerPointsSharp->points[i].z);  // 角点(最尖锐)点云
                            Eigen::Vector3d last_point_a(laserCloudCornerLast->points[closestPointInd].x,  // 上一帧角点点云
                                                         laserCloudCornerLast->points[closestPointInd].y,  // 上一帧角点点云
                                                         laserCloudCornerLast->points[closestPointInd].z);  // 上一帧角点点云
                            Eigen::Vector3d last_point_b(laserCloudCornerLast->points[minPointInd2].x,  // 上一帧角点点云
                                                         laserCloudCornerLast->points[minPointInd2].y,  // 上一帧角点点云
                                                         laserCloudCornerLast->points[minPointInd2].z);  // 上一帧角点点云

                            double s;  // 语句:声明/调用
                            if (DISTORTION)  // 是否启用运动畸变补偿
                                s = (cornerPointsSharp->points[i].intensity - int(cornerPointsSharp->points[i].intensity)) / SCAN_PERIOD;  // 扫描周期(s)
                            else  // 否则分支
                                s = 1.0;  // 语句:赋值/初始化
                            ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, last_point_a, last_point_b, s);  // 语句:赋值/初始化
                            problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);  // 优化参数:四元数(x,y,z,w)
                            corner_correspondence++;  // 语句:声明/调用
                        }  // 代码块结束
                    }  // 代码块结束

                    // find correspondence for plane features
                    for (int i = 0; i < surfPointsFlatNum; ++i)  // 面点(最平坦)点云
                    {  // 代码块开始
                        TransformToStart(&(surfPointsFlat->points[i]), &pointSel);  // 面点(最平坦)点云
                        kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);  // 上一帧面点KD树

                        int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;  // 语句:赋值/初始化
                        if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)  // 匹配距离阈值(平方)
                        {  // 代码块开始
                            closestPointInd = pointSearchInd[0];  // 语句:赋值/初始化

                            // get closest point's scan ID
                            int closestPointScanID = int(laserCloudSurfLast->points[closestPointInd].intensity);  // 上一帧面点点云
                            double minPointSqDis2 = DISTANCE_SQ_THRESHOLD, minPointSqDis3 = DISTANCE_SQ_THRESHOLD;  // 匹配距离阈值(平方)

                            // search in the direction of increasing scan line
                            for (int j = closestPointInd + 1; j < (int)laserCloudSurfLast->points.size(); ++j)  // 上一帧面点点云
                            {  // 代码块开始
                                // if not in nearby scans, end the loop
                                if (int(laserCloudSurfLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))  // 允许的相邻线束跨度
                                    break;  // 跳出循环/分支

                                double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *  // 上一帧面点点云
                                                        (laserCloudSurfLast->points[j].x - pointSel.x) +  // 上一帧面点点云
                                                    (laserCloudSurfLast->points[j].y - pointSel.y) *  // 上一帧面点点云
                                                        (laserCloudSurfLast->points[j].y - pointSel.y) +  // 上一帧面点点云
                                                    (laserCloudSurfLast->points[j].z - pointSel.z) *  // 上一帧面点点云
                                                        (laserCloudSurfLast->points[j].z - pointSel.z);  // 上一帧面点点云

                                // if in the same or lower scan line
                                if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScanID && pointSqDis < minPointSqDis2)  // 上一帧面点点云
                                {  // 代码块开始
                                    minPointSqDis2 = pointSqDis;  // 语句:赋值/初始化
                                    minPointInd2 = j;  // 语句:赋值/初始化
                                }  // 代码块结束
                                // if in the higher scan line
                                else if (int(laserCloudSurfLast->points[j].intensity) > closestPointScanID && pointSqDis < minPointSqDis3)  // 上一帧面点点云
                                {  // 代码块开始
                                    minPointSqDis3 = pointSqDis;  // 语句:赋值/初始化
                                    minPointInd3 = j;  // 语句:赋值/初始化
                                }  // 代码块结束
                            }  // 代码块结束

                            // search in the direction of decreasing scan line
                            for (int j = closestPointInd - 1; j >= 0; --j)  // for 循环
                            {  // 代码块开始
                                // if not in nearby scans, end the loop
                                if (int(laserCloudSurfLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))  // 允许的相邻线束跨度
                                    break;  // 跳出循环/分支

                                double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *  // 上一帧面点点云
                                                        (laserCloudSurfLast->points[j].x - pointSel.x) +  // 上一帧面点点云
                                                    (laserCloudSurfLast->points[j].y - pointSel.y) *  // 上一帧面点点云
                                                        (laserCloudSurfLast->points[j].y - pointSel.y) +  // 上一帧面点点云
                                                    (laserCloudSurfLast->points[j].z - pointSel.z) *  // 上一帧面点点云
                                                        (laserCloudSurfLast->points[j].z - pointSel.z);  // 上一帧面点点云

                                // if in the same or higher scan line
                                if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScanID && pointSqDis < minPointSqDis2)  // 上一帧面点点云
                                {  // 代码块开始
                                    minPointSqDis2 = pointSqDis;  // 语句:赋值/初始化
                                    minPointInd2 = j;  // 语句:赋值/初始化
                                }  // 代码块结束
                                else if (int(laserCloudSurfLast->points[j].intensity) < closestPointScanID && pointSqDis < minPointSqDis3)  // 上一帧面点点云
                                {  // 代码块开始
                                    // find nearer point
                                    minPointSqDis3 = pointSqDis;  // 语句:赋值/初始化
                                    minPointInd3 = j;  // 语句:赋值/初始化
                                }  // 代码块结束
                            }  // 代码块结束

                            if (minPointInd2 >= 0 && minPointInd3 >= 0)  // 条件判断
                            {  // 代码块开始

                                Eigen::Vector3d curr_point(surfPointsFlat->points[i].x,  // 面点(最平坦)点云
                                                            surfPointsFlat->points[i].y,  // 面点(最平坦)点云
                                                            surfPointsFlat->points[i].z);  // 面点(最平坦)点云
                                Eigen::Vector3d last_point_a(laserCloudSurfLast->points[closestPointInd].x,  // 上一帧面点点云
                                                                laserCloudSurfLast->points[closestPointInd].y,  // 上一帧面点点云
                                                                laserCloudSurfLast->points[closestPointInd].z);  // 上一帧面点点云
                                Eigen::Vector3d last_point_b(laserCloudSurfLast->points[minPointInd2].x,  // 上一帧面点点云
                                                                laserCloudSurfLast->points[minPointInd2].y,  // 上一帧面点点云
                                                                laserCloudSurfLast->points[minPointInd2].z);  // 上一帧面点点云
                                Eigen::Vector3d last_point_c(laserCloudSurfLast->points[minPointInd3].x,  // 上一帧面点点云
                                                                laserCloudSurfLast->points[minPointInd3].y,  // 上一帧面点点云
                                                                laserCloudSurfLast->points[minPointInd3].z);  // 上一帧面点点云

                                double s;  // 语句:声明/调用
                                if (DISTORTION)  // 是否启用运动畸变补偿
                                    s = (surfPointsFlat->points[i].intensity - int(surfPointsFlat->points[i].intensity)) / SCAN_PERIOD;  // 扫描周期(s)
                                else  // 否则分支
                                    s = 1.0;  // 语句:赋值/初始化
                                ceres::CostFunction *cost_function = LidarPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c, s);  // 语句:赋值/初始化
                                problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);  // 优化参数:四元数(x,y,z,w)
                                plane_correspondence++;  // 语句:声明/调用
                            }  // 代码块结束
                        }  // 代码块结束
                    }  // 代码块结束

                    //printf("coner_correspondance %d, plane_correspondence %d \n", corner_correspondence, plane_correspondence);
                    printf("data association time %f ms \n", t_data.toc());  // 语句:声明/调用

                    if ((corner_correspondence + plane_correspondence) < 10)  // 条件判断
                    {  // 代码块开始
                        printf("less correspondence! *************************************************\n");  // 语句:声明/调用
                    }  // 代码块结束

                    TicToc t_solver;  // 计时(统计耗时)
                    ceres::Solver::Options options;  // 语句:声明/调用
                    options.linear_solver_type = ceres::DENSE_QR;  // 语句:赋值/初始化
                    options.max_num_iterations = 4;  // 语句:赋值/初始化
                    options.minimizer_progress_to_stdout = false;  // 语句:赋值/初始化
                    ceres::Solver::Summary summary;  // 语句:声明/调用
                    ceres::Solve(options, &problem, &summary);  // 语句:声明/调用
                    printf("solver time %f ms \n", t_solver.toc());  // 语句:声明/调用
                }  // 代码块结束
                printf("optimization twice time %f \n", t_opt.toc());  // 语句:声明/调用

                t_w_curr = t_w_curr + q_w_curr * t_last_curr;  // 当前位姿(世界系)四元数
                q_w_curr = q_w_curr * q_last_curr;  // 当前位姿(世界系)四元数
            }  // 代码块结束

            TicToc t_pub;  // 计时(统计耗时)

            // publish odometry
            nav_msgs::Odometry laserOdometry;  // 语句:声明/调用
            laserOdometry.header.frame_id = "camera_init";  // 语句:赋值/初始化
            laserOdometry.child_frame_id = "laser_odom";  // 语句:赋值/初始化
            laserOdometry.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);  // 语句:赋值/初始化
            laserOdometry.pose.pose.orientation.x = q_w_curr.x();  // 当前位姿(世界系)四元数
            laserOdometry.pose.pose.orientation.y = q_w_curr.y();  // 当前位姿(世界系)四元数
            laserOdometry.pose.pose.orientation.z = q_w_curr.z();  // 当前位姿(世界系)四元数
            laserOdometry.pose.pose.orientation.w = q_w_curr.w();  // 当前位姿(世界系)四元数
            laserOdometry.pose.pose.position.x = t_w_curr.x();  // 当前位置(世界系)平移
            laserOdometry.pose.pose.position.y = t_w_curr.y();  // 当前位置(世界系)平移
            laserOdometry.pose.pose.position.z = t_w_curr.z();  // 当前位置(世界系)平移
            pubLaserOdometry.publish(laserOdometry);  // 发布消息

            geometry_msgs::PoseStamped laserPose;  // 语句:声明/调用
            laserPose.header = laserOdometry.header;  // 语句:赋值/初始化
            laserPose.pose = laserOdometry.pose.pose;  // 语句:赋值/初始化
            laserPath.header.stamp = laserOdometry.header.stamp;  // 语句:赋值/初始化
            laserPath.poses.push_back(laserPose);  // 语句:声明/调用
            laserPath.header.frame_id = "camera_init";  // 语句:赋值/初始化
            pubLaserPath.publish(laserPath);  // 发布消息

            // transform corner features and plane features to the scan end point
            if (0)  // 条件判断
            {  // 代码块开始
                int cornerPointsLessSharpNum = cornerPointsLessSharp->points.size();  // 角点(次尖锐)点云
                for (int i = 0; i < cornerPointsLessSharpNum; i++)  // 角点(次尖锐)点云
                {  // 代码块开始
                    TransformToEnd(&cornerPointsLessSharp->points[i], &cornerPointsLessSharp->points[i]);  // 角点(次尖锐)点云
                }  // 代码块结束

                int surfPointsLessFlatNum = surfPointsLessFlat->points.size();  // 面点(次平坦)点云
                for (int i = 0; i < surfPointsLessFlatNum; i++)  // 面点(次平坦)点云
                {  // 代码块开始
                    TransformToEnd(&surfPointsLessFlat->points[i], &surfPointsLessFlat->points[i]);  // 面点(次平坦)点云
                }  // 代码块结束

                int laserCloudFullResNum = laserCloudFullRes->points.size();  // 全分辨率点云
                for (int i = 0; i < laserCloudFullResNum; i++)  // 全分辨率点云
                {  // 代码块开始
                    TransformToEnd(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);  // 全分辨率点云
                }  // 代码块结束
            }  // 代码块结束

            pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;  // 角点(次尖锐)点云
            cornerPointsLessSharp = laserCloudCornerLast;  // 角点(次尖锐)点云
            laserCloudCornerLast = laserCloudTemp;  // 上一帧角点点云

            laserCloudTemp = surfPointsLessFlat;  // 面点(次平坦)点云
            surfPointsLessFlat = laserCloudSurfLast;  // 面点(次平坦)点云
            laserCloudSurfLast = laserCloudTemp;  // 上一帧面点点云

            laserCloudCornerLastNum = laserCloudCornerLast->points.size();  // 上一帧角点点云
            laserCloudSurfLastNum = laserCloudSurfLast->points.size();  // 上一帧面点点云
            if (laserCloudCornerLastNum == 0 || laserCloudSurfLastNum == 0) {  // 上一帧角点点云
                ROS_WARN("Not enough features, reset systemInited");  // 系统初始化标志
                systemInited = false;  // 系统初始化标志
                continue;  // 继续下一轮
            }  // 代码块结束
            // std::cout << "the size of corner last is " << laserCloudCornerLastNum << ", and the size of surf last is " << laserCloudSurfLastNum << '\n';

            kdtreeCornerLast->setInputCloud(laserCloudCornerLast);  // 上一帧角点KD树
            kdtreeSurfLast->setInputCloud(laserCloudSurfLast);  // 上一帧面点KD树

            if (frameCount % skipFrameNum == 0)  // 初始化跳帧数
            {  // 代码块开始
                frameCount = 0;  // 语句:赋值/初始化

                sensor_msgs::PointCloud2 laserCloudCornerLast2;  // 上一帧角点点云
                pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);  // 上一帧角点点云
                laserCloudCornerLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);  // 上一帧角点点云
                laserCloudCornerLast2.header.frame_id = "camera";  // 上一帧角点点云
                pubLaserCloudCornerLast.publish(laserCloudCornerLast2);  // 上一帧角点点云

                sensor_msgs::PointCloud2 laserCloudSurfLast2;  // 上一帧面点点云
                pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);  // 上一帧面点点云
                laserCloudSurfLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);  // 上一帧面点点云
                laserCloudSurfLast2.header.frame_id = "camera";  // 上一帧面点点云
                pubLaserCloudSurfLast.publish(laserCloudSurfLast2);  // 上一帧面点点云

                sensor_msgs::PointCloud2 laserCloudFullRes3;  // 全分辨率点云
                pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);  // 全分辨率点云
                laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);  // 全分辨率点云
                laserCloudFullRes3.header.frame_id = "camera";  // 全分辨率点云
                pubLaserCloudFullRes.publish(laserCloudFullRes3);  // 全分辨率点云
            }  // 代码块结束
            printf("publication time %f ms \n", t_pub.toc());  // 语句:声明/调用
            printf("whole laserOdometry time %f ms \n \n", t_whole.toc());  // 语句:声明/调用
            if(t_whole.toc() > 100)  // 条件判断
                ROS_WARN("odometry process over 100ms");  // 语句:声明/调用

            frameCount++;  // 语句:声明/调用
        }  // 代码块结束
        rate.sleep();  // 语句:声明/调用
    }  // 代码块结束
    return 0;  // 返回
}  // 代码块结束
