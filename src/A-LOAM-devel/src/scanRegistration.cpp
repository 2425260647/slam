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
#include <vector>  // 头文件依赖
#include <string>  // 头文件依赖
#include "aloam_velodyne/common.h"  // 头文件依赖
#include "aloam_velodyne/tic_toc.h"  // 头文件依赖
#include <nav_msgs/Odometry.h>  // 头文件依赖
#include <pcl_conversions/pcl_conversions.h>  // 头文件依赖
#include <pcl/point_cloud.h>  // 头文件依赖
#include <pcl/point_types.h>  // 头文件依赖
#include <pcl/filters/voxel_grid.h>  // 头文件依赖
#include <pcl/kdtree/kdtree_flann.h>  // 头文件依赖
#include <ros/ros.h>  // 头文件依赖
#include <sensor_msgs/Imu.h>  // 头文件依赖
#include <sensor_msgs/PointCloud2.h>  // 头文件依赖
#include <tf/transform_datatypes.h>  // 头文件依赖
#include <tf/transform_broadcaster.h>  // 头文件依赖

using std::atan2;  // 类型别名/引入符号
using std::cos;  // 类型别名/引入符号
using std::sin;  // 类型别名/引入符号

const double scanPeriod = 0.1;  // 一圈扫描周期(s)

const int systemDelay = 0;   // 系统延时帧数
int systemInitCount = 0;  // 初始化计数
bool systemInited = false;  // 初始化完成标志
int N_SCANS = 0;  // 雷达线数(16/32/64)
float cloudCurvature[400000];  // 点曲率(特征提取)
int cloudSortInd[400000];  // 曲率排序索引
int cloudNeighborPicked[400000];  // 邻域屏蔽/已选标记
int cloudLabel[400000];  // 点标签(角/面/忽略)

bool comp (int i,int j) { return (cloudCurvature[i]<cloudCurvature[j]); }  // 点曲率(特征提取)

ros::Publisher pubLaserCloud;  // ROS 发布器
ros::Publisher pubCornerPointsSharp;  // ROS 发布器
ros::Publisher pubCornerPointsLessSharp;  // ROS 发布器
ros::Publisher pubSurfPointsFlat;  // ROS 发布器
ros::Publisher pubSurfPointsLessFlat;  // ROS 发布器
ros::Publisher pubRemovePoints;  // ROS 发布器
std::vector<ros::Publisher> pubEachScan;  // 语句:声明/调用

bool PUB_EACH_LINE = false;  // 是否发布每条scan点云

double MINIMUM_RANGE = 0.1;   // 最小距离阈值(剔除近点)

template <typename PointT>  // 语句
void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,  // 去除近距离点
                              pcl::PointCloud<PointT> &cloud_out, float thres)  // 语句
{  // 代码块开始
    if (&cloud_in != &cloud_out)  // 条件判断
    {  // 代码块开始
        cloud_out.header = cloud_in.header;  // 语句:赋值/初始化
        cloud_out.points.resize(cloud_in.points.size());  // 语句:声明/调用
    }  // 代码块结束

    size_t j = 0;  // 语句:赋值/初始化

    for (size_t i = 0; i < cloud_in.points.size(); ++i)  // for 循环
    {  // 代码块开始
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)  // 条件判断
            continue;  // 继续下一轮
        cloud_out.points[j] = cloud_in.points[i];  // 语句:赋值/初始化
        j++;  // 语句:声明/调用
    }  // 代码块结束
    if (j != cloud_in.points.size())  // 条件判断
    {  // 代码块开始
        cloud_out.points.resize(j);  // 语句:声明/调用
    }  // 代码块结束

    cloud_out.height = 1;  // 语句:赋值/初始化
    cloud_out.width = static_cast<uint32_t>(j);  // 语句:赋值/初始化
    cloud_out.is_dense = true;  // 语句:赋值/初始化
}  // 代码块结束

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)  // 点云回调入口
{  // 代码块开始
    if (!systemInited)  // 初始化完成标志
    {   // 代码块开始
        systemInitCount++;  // 初始化计数
        if (systemInitCount >= systemDelay)  // 系统延时帧数
        {  // 代码块开始
            systemInited = true;  // 初始化完成标志
        }  // 代码块结束
        else  // 否则分支
            return;  // 返回
    }  // 代码块结束

    TicToc t_whole;  // 计时(统计耗时)
    TicToc t_prepare;  // 计时(统计耗时)
    std::vector<int> scanStartInd(N_SCANS, 0);  // 雷达线数(16/32/64)
    std::vector<int> scanEndInd(N_SCANS, 0);  // 雷达线数(16/32/64)

    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;  // 语句:声明/调用
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);  // ROS->PCL 点云转换
    std::vector<int> indices;  // 语句:声明/调用

    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);  // 语句:声明/调用
    removeClosedPointCloud(laserCloudIn, laserCloudIn, MINIMUM_RANGE);  // 最小距离阈值(剔除近点)


    int cloudSize = laserCloudIn.points.size();  // 点云点数
    float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);  // 起始方位角
    float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y,  // 结束方位角
                          laserCloudIn.points[cloudSize - 1].x) +  // 点云点数
                   2 * M_PI;  // 语句:声明/调用

    if (endOri - startOri > 3 * M_PI)  // 起始方位角
    {  // 代码块开始
        endOri -= 2 * M_PI;  // 结束方位角
    }  // 代码块结束
    else if (endOri - startOri < M_PI)  // 起始方位角
    {  // 代码块开始
        endOri += 2 * M_PI;  // 结束方位角
    }  // 代码块结束
    //printf("end Ori %f\n", endOri);

    bool halfPassed = false;  // 是否跨过半圈标志
    int count = cloudSize;  // 点云点数
    PointType point;  // 语句:声明/调用
    std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);  // 雷达线数(16/32/64)
    for (int i = 0; i < cloudSize; i++)  // 点云点数
    {  // 代码块开始
        point.x = laserCloudIn.points[i].x;  // 语句:赋值/初始化
        point.y = laserCloudIn.points[i].y;  // 语句:赋值/初始化
        point.z = laserCloudIn.points[i].z;  // 语句:赋值/初始化

        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;  // 语句:赋值/初始化
        int scanID = 0;  // 线束编号

        if (N_SCANS == 16)  // 雷达线数(16/32/64)
        {  // 代码块开始
            scanID = int((angle + 15) / 2 + 0.5);  // 线束编号
            if (scanID > (N_SCANS - 1) || scanID < 0)  // 雷达线数(16/32/64)
            {  // 代码块开始
                count--;  // 语句:声明/调用
                continue;  // 继续下一轮
            }  // 代码块结束
        }  // 代码块结束
        else if (N_SCANS == 32)  // 雷达线数(16/32/64)
        {  // 代码块开始
            scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);  // 线束编号
            if (scanID > (N_SCANS - 1) || scanID < 0)  // 雷达线数(16/32/64)
            {  // 代码块开始
                count--;  // 语句:声明/调用
                continue;  // 继续下一轮
            }  // 代码块结束
        }  // 代码块结束
        else if (N_SCANS == 64)  // 雷达线数(16/32/64)
        {     // 代码块开始
            if (angle >= -8.83)  // 条件判断
                scanID = int((2 - angle) * 3.0 + 0.5);  // 线束编号
            else  // 否则分支
                scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);  // 雷达线数(16/32/64)

            // use [0 50]  > 50 remove outlies 
            if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)  // 线束编号
            {  // 代码块开始
                count--;  // 语句:声明/调用
                continue;  // 继续下一轮
            }  // 代码块结束
        }  // 代码块结束
        else  // 否则分支
        {  // 代码块开始
            printf("wrong scan number\n");  // 语句:声明/调用
            ROS_BREAK();  // 语句:声明/调用
        }  // 代码块结束
        //printf("angle %f scanID %d \n", angle, scanID);

        float ori = -atan2(point.y, point.x);  // 语句:赋值/初始化
        if (!halfPassed)  // 是否跨过半圈标志
        {   // 代码块开始
            if (ori < startOri - M_PI / 2)  // 起始方位角
            {  // 代码块开始
                ori += 2 * M_PI;  // 语句:赋值/初始化
            }  // 代码块结束
            else if (ori > startOri + M_PI * 3 / 2)  // 起始方位角
            {  // 代码块开始
                ori -= 2 * M_PI;  // 语句:赋值/初始化
            }  // 代码块结束

            if (ori - startOri > M_PI)  // 起始方位角
            {  // 代码块开始
                halfPassed = true;  // 是否跨过半圈标志
            }  // 代码块结束
        }  // 代码块结束
        else  // 否则分支
        {  // 代码块开始
            ori += 2 * M_PI;  // 语句:赋值/初始化
            if (ori < endOri - M_PI * 3 / 2)  // 结束方位角
            {  // 代码块开始
                ori += 2 * M_PI;  // 语句:赋值/初始化
            }  // 代码块结束
            else if (ori > endOri + M_PI / 2)  // 结束方位角
            {  // 代码块开始
                ori -= 2 * M_PI;  // 语句:赋值/初始化
            }  // 代码块结束
        }  // 代码块结束

        float relTime = (ori - startOri) / (endOri - startOri);  // 起始方位角
        point.intensity = scanID + scanPeriod * relTime;  // 一圈扫描周期(s)
        laserCloudScans[scanID].push_back(point);   // 线束编号
    }  // 代码块结束
    
    cloudSize = count;  // 点云点数
    printf("points size %d \n", cloudSize);  // 点云点数

    pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());  // 语句:声明/调用
    for (int i = 0; i < N_SCANS; i++)  // 雷达线数(16/32/64)
    {   // 代码块开始
        scanStartInd[i] = laserCloud->size() + 5;  // 每线起始索引
        *laserCloud += laserCloudScans[i];  // 语句:赋值/初始化
        scanEndInd[i] = laserCloud->size() - 6;  // 每线结束索引
    }  // 代码块结束

    printf("prepare time %f \n", t_prepare.toc());  // 语句:声明/调用

    for (int i = 5; i < cloudSize - 5; i++)  // 点云点数
    {   // 代码块开始
        float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;  // 语句:赋值/初始化
        float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;  // 语句:赋值/初始化
        float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;  // 语句:赋值/初始化

        cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;  // 点曲率(特征提取)
        cloudSortInd[i] = i;  // 曲率排序索引
        cloudNeighborPicked[i] = 0;  // 邻域屏蔽/已选标记
        cloudLabel[i] = 0;  // 点标签(角/面/忽略)
    }  // 代码块结束


    TicToc t_pts;  // 计时(统计耗时)

    pcl::PointCloud<PointType> cornerPointsSharp;  // 语句:声明/调用
    pcl::PointCloud<PointType> cornerPointsLessSharp;  // 语句:声明/调用
    pcl::PointCloud<PointType> surfPointsFlat;  // 语句:声明/调用
    pcl::PointCloud<PointType> surfPointsLessFlat;  // 语句:声明/调用

    float t_q_sort = 0;  // 语句:赋值/初始化
    for (int i = 0; i < N_SCANS; i++)  // 雷达线数(16/32/64)
    {  // 代码块开始
        if( scanEndInd[i] - scanStartInd[i] < 6)  // 每线起始索引
            continue;  // 继续下一轮
        pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);  // 语句:声明/调用
        for (int j = 0; j < 6; j++)  // for 循环
        {  // 代码块开始
            int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6;   // 每线起始索引
            int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;  // 每线起始索引

            TicToc t_tmp;  // 计时(统计耗时)
            std::sort (cloudSortInd + sp, cloudSortInd + ep + 1, comp);  // 曲率排序索引
            t_q_sort += t_tmp.toc();  // 语句:赋值/初始化

            int largestPickedNum = 0;  // 语句:赋值/初始化
            for (int k = ep; k >= sp; k--)  // for 循环
            {  // 代码块开始
                int ind = cloudSortInd[k];   // 曲率排序索引

                if (cloudNeighborPicked[ind] == 0 &&  // 邻域屏蔽/已选标记
                    cloudCurvature[ind] > 0.1)  // 点曲率(特征提取)
                {  // 代码块开始

                    largestPickedNum++;  // 语句:声明/调用
                    if (largestPickedNum <= 2)  // 条件判断
                    {                          // 代码块开始
                        cloudLabel[ind] = 2;  // 点标签(角/面/忽略)
                        cornerPointsSharp.push_back(laserCloud->points[ind]);  // 语句:声明/调用
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);  // 语句:声明/调用
                    }  // 代码块结束
                    else if (largestPickedNum <= 20)  // 否则分支
                    {                          // 代码块开始
                        cloudLabel[ind] = 1;   // 点标签(角/面/忽略)
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);  // 语句:声明/调用
                    }  // 代码块结束
                    else  // 否则分支
                    {  // 代码块开始
                        break;  // 跳出循环/分支
                    }  // 代码块结束

                    cloudNeighborPicked[ind] = 1;   // 邻域屏蔽/已选标记

                    for (int l = 1; l <= 5; l++)  // for 循环
                    {  // 代码块开始
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;  // 语句:赋值/初始化
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;  // 语句:赋值/初始化
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;  // 语句:赋值/初始化
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)  // 条件判断
                        {  // 代码块开始
                            break;  // 跳出循环/分支
                        }  // 代码块结束

                        cloudNeighborPicked[ind + l] = 1;  // 邻域屏蔽/已选标记
                    }  // 代码块结束
                    for (int l = -1; l >= -5; l--)  // for 循环
                    {  // 代码块开始
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;  // 语句:赋值/初始化
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;  // 语句:赋值/初始化
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;  // 语句:赋值/初始化
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)  // 条件判断
                        {  // 代码块开始
                            break;  // 跳出循环/分支
                        }  // 代码块结束

                        cloudNeighborPicked[ind + l] = 1;  // 邻域屏蔽/已选标记
                    }  // 代码块结束
                }  // 代码块结束
            }  // 代码块结束

            int smallestPickedNum = 0;  // 语句:赋值/初始化
            for (int k = sp; k <= ep; k++)  // for 循环
            {  // 代码块开始
                int ind = cloudSortInd[k];  // 曲率排序索引

                if (cloudNeighborPicked[ind] == 0 &&  // 邻域屏蔽/已选标记
                    cloudCurvature[ind] < 0.1)  // 点曲率(特征提取)
                {  // 代码块开始

                    cloudLabel[ind] = -1;   // 点标签(角/面/忽略)
                    surfPointsFlat.push_back(laserCloud->points[ind]);  // 语句:声明/调用

                    smallestPickedNum++;  // 语句:声明/调用
                    if (smallestPickedNum >= 4)  // 条件判断
                    {   // 代码块开始
                        break;  // 跳出循环/分支
                    }  // 代码块结束

                    cloudNeighborPicked[ind] = 1;  // 邻域屏蔽/已选标记
                    for (int l = 1; l <= 5; l++)  // for 循环
                    {   // 代码块开始
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;  // 语句:赋值/初始化
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;  // 语句:赋值/初始化
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;  // 语句:赋值/初始化
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)  // 条件判断
                        {  // 代码块开始
                            break;  // 跳出循环/分支
                        }  // 代码块结束

                        cloudNeighborPicked[ind + l] = 1;  // 邻域屏蔽/已选标记
                    }  // 代码块结束
                    for (int l = -1; l >= -5; l--)  // for 循环
                    {  // 代码块开始
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;  // 语句:赋值/初始化
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;  // 语句:赋值/初始化
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;  // 语句:赋值/初始化
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)  // 条件判断
                        {  // 代码块开始
                            break;  // 跳出循环/分支
                        }  // 代码块结束

                        cloudNeighborPicked[ind + l] = 1;  // 邻域屏蔽/已选标记
                    }  // 代码块结束
                }  // 代码块结束
            }  // 代码块结束

            for (int k = sp; k <= ep; k++)  // for 循环
            {  // 代码块开始
                if (cloudLabel[k] <= 0)  // 点标签(角/面/忽略)
                {  // 代码块开始
                    surfPointsLessFlatScan->push_back(laserCloud->points[k]);  // 语句:声明/调用
                }  // 代码块结束
            }  // 代码块结束
        }  // 代码块结束

        pcl::PointCloud<PointType> surfPointsLessFlatScanDS;  // 语句:声明/调用
        pcl::VoxelGrid<PointType> downSizeFilter;  // 语句:声明/调用
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);  // 语句:声明/调用
        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);  // 语句:声明/调用
        downSizeFilter.filter(surfPointsLessFlatScanDS);  // 语句:声明/调用

        surfPointsLessFlat += surfPointsLessFlatScanDS;  // 语句:赋值/初始化
    }  // 代码块结束
    printf("sort q time %f \n", t_q_sort);  // 语句:声明/调用
    printf("seperate points time %f \n", t_pts.toc());  // 语句:声明/调用


    sensor_msgs::PointCloud2 laserCloudOutMsg;  // 语句:声明/调用
    pcl::toROSMsg(*laserCloud, laserCloudOutMsg);  // PCL->ROS 点云转换
    laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;  // 语句:赋值/初始化
    laserCloudOutMsg.header.frame_id = "camera_init";  // 语句:赋值/初始化
    pubLaserCloud.publish(laserCloudOutMsg);  // 发布消息

    sensor_msgs::PointCloud2 cornerPointsSharpMsg;  // 语句:声明/调用
    pcl::toROSMsg(cornerPointsSharp, cornerPointsSharpMsg);  // PCL->ROS 点云转换
    cornerPointsSharpMsg.header.stamp = laserCloudMsg->header.stamp;  // 语句:赋值/初始化
    cornerPointsSharpMsg.header.frame_id = "camera_init";  // 语句:赋值/初始化
    pubCornerPointsSharp.publish(cornerPointsSharpMsg);  // 发布消息

    sensor_msgs::PointCloud2 cornerPointsLessSharpMsg;  // 语句:声明/调用
    pcl::toROSMsg(cornerPointsLessSharp, cornerPointsLessSharpMsg);  // PCL->ROS 点云转换
    cornerPointsLessSharpMsg.header.stamp = laserCloudMsg->header.stamp;  // 语句:赋值/初始化
    cornerPointsLessSharpMsg.header.frame_id = "camera_init";  // 语句:赋值/初始化
    pubCornerPointsLessSharp.publish(cornerPointsLessSharpMsg);  // 发布消息

    sensor_msgs::PointCloud2 surfPointsFlat2;  // 语句:声明/调用
    pcl::toROSMsg(surfPointsFlat, surfPointsFlat2);  // PCL->ROS 点云转换
    surfPointsFlat2.header.stamp = laserCloudMsg->header.stamp;  // 语句:赋值/初始化
    surfPointsFlat2.header.frame_id = "camera_init";  // 语句:赋值/初始化
    pubSurfPointsFlat.publish(surfPointsFlat2);  // 发布消息

    sensor_msgs::PointCloud2 surfPointsLessFlat2;  // 语句:声明/调用
    pcl::toROSMsg(surfPointsLessFlat, surfPointsLessFlat2);  // PCL->ROS 点云转换
    surfPointsLessFlat2.header.stamp = laserCloudMsg->header.stamp;  // 语句:赋值/初始化
    surfPointsLessFlat2.header.frame_id = "camera_init";  // 语句:赋值/初始化
    pubSurfPointsLessFlat.publish(surfPointsLessFlat2);  // 发布消息

    // pub each scam
    if(PUB_EACH_LINE)  // 是否发布每条scan点云
    {  // 代码块开始
        for(int i = 0; i< N_SCANS; i++)  // 雷达线数(16/32/64)
        {  // 代码块开始
            sensor_msgs::PointCloud2 scanMsg;  // 语句:声明/调用
            pcl::toROSMsg(laserCloudScans[i], scanMsg);  // PCL->ROS 点云转换
            scanMsg.header.stamp = laserCloudMsg->header.stamp;  // 语句:赋值/初始化
            scanMsg.header.frame_id = "camera_init";  // 语句:赋值/初始化
            pubEachScan[i].publish(scanMsg);  // 发布消息
        }  // 代码块结束
    }  // 代码块结束

    printf("scan registration time %f ms *************\n", t_whole.toc());  // 语句:声明/调用
    if(t_whole.toc() > 100)  // 条件判断
        ROS_WARN("scan registration process over 100ms");  // 语句:声明/调用
}  // 代码块结束

int main(int argc, char **argv)  // 语句
{  // 代码块开始
    ros::init(argc, argv, "scanRegistration");  // 语句:声明/调用
    ros::NodeHandle nh;  // 语句:声明/调用

    nh.param<int>("scan_line", N_SCANS, 16);  // 雷达线数(16/32/64)

    nh.param<double>("minimum_range", MINIMUM_RANGE, 0.1);  // 最小距离阈值(剔除近点)

    printf("scan line number %d \n", N_SCANS);  // 雷达线数(16/32/64)

    if(N_SCANS != 16 && N_SCANS != 32 && N_SCANS != 64)  // 雷达线数(16/32/64)
    {  // 代码块开始
        printf("only support velodyne with 16, 32 or 64 scan line!");  // 语句:声明/调用
        return 0;  // 返回
    }  // 代码块结束

    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, laserCloudHandler);  // 点云回调入口

    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100);  // 语句:赋值/初始化

    pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100);  // 语句:赋值/初始化

    pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100);  // 语句:赋值/初始化

    pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100);  // 语句:赋值/初始化

    pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100);  // 语句:赋值/初始化

    pubRemovePoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_remove_points", 100);  // 语句:赋值/初始化

    if(PUB_EACH_LINE)  // 是否发布每条scan点云
    {  // 代码块开始
        for(int i = 0; i < N_SCANS; i++)  // 雷达线数(16/32/64)
        {  // 代码块开始
            ros::Publisher tmp = nh.advertise<sensor_msgs::PointCloud2>("/laser_scanid_" + std::to_string(i), 100);  // ROS 发布器
            pubEachScan.push_back(tmp);  // 语句:声明/调用
        }  // 代码块结束
    }  // 代码块结束
    ros::spin();  // 语句:声明/调用

    return 0;  // 返回
}  // 代码块结束
