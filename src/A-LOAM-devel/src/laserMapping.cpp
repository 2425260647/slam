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

#include <math.h>  // 头文件依赖
#include <vector>  // 头文件依赖
#include <aloam_velodyne/common.h>  // 头文件依赖
#include <nav_msgs/Odometry.h>  // 头文件依赖
#include <nav_msgs/Path.h>  // 头文件依赖
#include <geometry_msgs/PoseStamped.h>  // 头文件依赖
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
#include <eigen3/Eigen/Dense>  // 头文件依赖
#include <ceres/ceres.h>  // 头文件依赖
#include <mutex>  // 头文件依赖
#include <queue>  // 头文件依赖
#include <thread>  // 头文件依赖
#include <iostream>  // 头文件依赖
#include <string>  // 头文件依赖

#include "lidarFactor.hpp"  // 头文件依赖
#include "aloam_velodyne/common.h"  // 头文件依赖
#include "aloam_velodyne/tic_toc.h"  // 头文件依赖


int frameCount = 0;  // 帧计数

double timeLaserCloudCornerLast = 0;  // 语句:赋值/初始化
double timeLaserCloudSurfLast = 0;  // 语句:赋值/初始化
double timeLaserCloudFullRes = 0;  // 语句:赋值/初始化
double timeLaserOdometry = 0;  // 语句:赋值/初始化


int laserCloudCenWidth = 10;  // 地图立方格中心索引(W)
int laserCloudCenHeight = 10;  // 地图立方格中心索引(H)
int laserCloudCenDepth = 5;  // 地图立方格中心索引(D)
const int laserCloudWidth = 21;  // 地图立方格宽度
const int laserCloudHeight = 21;  // 地图立方格高度
const int laserCloudDepth = 11;  // 地图立方格深度


const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth; //4851


int laserCloudValidInd[125];  // 有效立方格索引缓存
int laserCloudSurroundInd[125];  // 周围立方格索引缓存

// input: from odom
pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());  // 来自里程计:角点
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());  // 来自里程计:面点

// ouput: all visualble cube points
pcl::PointCloud<PointType>::Ptr laserCloudSurround(new pcl::PointCloud<PointType>());  // 周围可视化点云

// surround points in map to build tree
pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap(new pcl::PointCloud<PointType>());  // 地图角点(用于匹配)
pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap(new pcl::PointCloud<PointType>());  // 地图面点(用于匹配)

//input & output: points in one frame. local --> global
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());  // 全分辨率点云(局部->全局)

// points in every cube
pcl::PointCloud<PointType>::Ptr laserCloudCornerArray[laserCloudNum];  // 立方格总数
pcl::PointCloud<PointType>::Ptr laserCloudSurfArray[laserCloudNum];  // 立方格总数

//kd-tree
pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap(new pcl::KdTreeFLANN<PointType>());  // 地图角点KD树
pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>());  // 地图面点KD树

double parameters[7] = {0, 0, 0, 1, 0, 0, 0};  // 优化参数(q,t)
Eigen::Map<Eigen::Quaterniond> q_w_curr(parameters);  // 优化参数(q,t)
Eigen::Map<Eigen::Vector3d> t_w_curr(parameters + 4);  // 优化参数(q,t)

// wmap_T_odom * odom_T_curr = wmap_T_curr;
// transformation between odom's world and map's world frame
Eigen::Quaterniond q_wmap_wodom(1, 0, 0, 0);  // odom系->map系旋转
Eigen::Vector3d t_wmap_wodom(0, 0, 0);  // odom系->map系平移

Eigen::Quaterniond q_wodom_curr(1, 0, 0, 0);  // 语句:声明/调用
Eigen::Vector3d t_wodom_curr(0, 0, 0);  // 语句:声明/调用


std::queue<sensor_msgs::PointCloud2ConstPtr> cornerLastBuf;  // 语句:声明/调用
std::queue<sensor_msgs::PointCloud2ConstPtr> surfLastBuf;  // 语句:声明/调用
std::queue<sensor_msgs::PointCloud2ConstPtr> fullResBuf;  // 语句:声明/调用
std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf;  // 语句:声明/调用
std::mutex mBuf;  // 语句:声明/调用

pcl::VoxelGrid<PointType> downSizeFilterCorner;  // 语句:声明/调用
pcl::VoxelGrid<PointType> downSizeFilterSurf;  // 语句:声明/调用

std::vector<int> pointSearchInd;  // 语句:声明/调用
std::vector<float> pointSearchSqDis;  // 语句:声明/调用

PointType pointOri, pointSel;  // 语句:声明/调用

ros::Publisher pubLaserCloudSurround, pubLaserCloudMap, pubLaserCloudFullRes, pubLaserCloudFullResLocal, pubOdomAftMapped, pubOdomAftMappedHighFrec, pubLaserAfterMappedPath;  // ROS 发布器

nav_msgs::Path laserAfterMappedPath;  // 语句:声明/调用

// set initial guess
void transformAssociateToMap()  // 语句
{  // 代码块开始
	q_w_curr = q_wmap_wodom * q_wodom_curr;  // 当前位姿(地图系)四元数
	t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;  // 当前位置(地图系)平移
}  // 代码块结束

void transformUpdate()  // 语句
{  // 代码块开始
	q_wmap_wodom = q_w_curr * q_wodom_curr.inverse();  // 当前位姿(地图系)四元数
	t_wmap_wodom = t_w_curr - q_wmap_wodom * t_wodom_curr;  // 当前位置(地图系)平移
}  // 代码块结束

void pointAssociateToMap(PointType const *const pi, PointType *const po)  // 语句
{  // 代码块开始
	Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);  // 语句:声明/调用
	Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;  // 当前位姿(地图系)四元数
	po->x = point_w.x();  // 语句:赋值/初始化
	po->y = point_w.y();  // 语句:赋值/初始化
	po->z = point_w.z();  // 语句:赋值/初始化
	po->intensity = pi->intensity;  // 语句:赋值/初始化
	//po->intensity = 1.0;
}  // 代码块结束

void pointAssociateTobeMapped(PointType const *const pi, PointType *const po)  // 语句
{  // 代码块开始
	Eigen::Vector3d point_w(pi->x, pi->y, pi->z);  // 语句:声明/调用
	Eigen::Vector3d point_curr = q_w_curr.inverse() * (point_w - t_w_curr);  // 当前位姿(地图系)四元数
	po->x = point_curr.x();  // 语句:赋值/初始化
	po->y = point_curr.y();  // 语句:赋值/初始化
	po->z = point_curr.z();  // 语句:赋值/初始化
	po->intensity = pi->intensity;  // 语句:赋值/初始化
}  // 代码块结束

void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudCornerLast2)  // 来自里程计:角点
{  // 代码块开始
	mBuf.lock();  // 语句:声明/调用
	cornerLastBuf.push(laserCloudCornerLast2);  // 来自里程计:角点
	mBuf.unlock();  // 语句:声明/调用
}  // 代码块结束

void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudSurfLast2)  // 来自里程计:面点
{  // 代码块开始
	mBuf.lock();  // 语句:声明/调用
	surfLastBuf.push(laserCloudSurfLast2);  // 来自里程计:面点
	mBuf.unlock();  // 语句:声明/调用
}  // 代码块结束

void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2)  // 全分辨率点云(局部->全局)
{  // 代码块开始
	mBuf.lock();  // 语句:声明/调用
	fullResBuf.push(laserCloudFullRes2);  // 全分辨率点云(局部->全局)
	mBuf.unlock();  // 语句:声明/调用
}  // 代码块结束

//receive odomtry
void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &laserOdometry)  // 语句
{  // 代码块开始
	mBuf.lock();  // 语句:声明/调用
	odometryBuf.push(laserOdometry);  // 语句:声明/调用
	mBuf.unlock();  // 语句:声明/调用

	// high frequence publish
	Eigen::Quaterniond q_wodom_curr;  // 语句:声明/调用
	Eigen::Vector3d t_wodom_curr;  // 语句:声明/调用
	q_wodom_curr.x() = laserOdometry->pose.pose.orientation.x;  // 语句:赋值/初始化
	q_wodom_curr.y() = laserOdometry->pose.pose.orientation.y;  // 语句:赋值/初始化
	q_wodom_curr.z() = laserOdometry->pose.pose.orientation.z;  // 语句:赋值/初始化
	q_wodom_curr.w() = laserOdometry->pose.pose.orientation.w;  // 语句:赋值/初始化
	t_wodom_curr.x() = laserOdometry->pose.pose.position.x;  // 语句:赋值/初始化
	t_wodom_curr.y() = laserOdometry->pose.pose.position.y;  // 语句:赋值/初始化
	t_wodom_curr.z() = laserOdometry->pose.pose.position.z;  // 语句:赋值/初始化

	Eigen::Quaterniond q_w_curr = q_wmap_wodom * q_wodom_curr;  // 当前位姿(地图系)四元数
	Eigen::Vector3d t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;   // 当前位置(地图系)平移

	nav_msgs::Odometry odomAftMapped;  // 语句:声明/调用
	odomAftMapped.header.frame_id = "camera_init";  // 语句:赋值/初始化
	odomAftMapped.child_frame_id = "aft_mapped";  // 语句:赋值/初始化
	odomAftMapped.header.stamp = laserOdometry->header.stamp;  // 语句:赋值/初始化
	odomAftMapped.pose.pose.orientation.x = q_w_curr.x();  // 当前位姿(地图系)四元数
	odomAftMapped.pose.pose.orientation.y = q_w_curr.y();  // 当前位姿(地图系)四元数
	odomAftMapped.pose.pose.orientation.z = q_w_curr.z();  // 当前位姿(地图系)四元数
	odomAftMapped.pose.pose.orientation.w = q_w_curr.w();  // 当前位姿(地图系)四元数
	odomAftMapped.pose.pose.position.x = t_w_curr.x();  // 当前位置(地图系)平移
	odomAftMapped.pose.pose.position.y = t_w_curr.y();  // 当前位置(地图系)平移
	odomAftMapped.pose.pose.position.z = t_w_curr.z();  // 当前位置(地图系)平移
	pubOdomAftMappedHighFrec.publish(odomAftMapped);  // 发布消息
}  // 代码块结束

void process()  // 语句
{  // 代码块开始
	while(1)  // while 循环
	{  // 代码块开始
		while (!cornerLastBuf.empty() && !surfLastBuf.empty() &&  // while 循环
			!fullResBuf.empty() && !odometryBuf.empty())  // 语句
		{  // 代码块开始
			mBuf.lock();  // 语句:声明/调用
			while (!odometryBuf.empty() && odometryBuf.front()->header.stamp.toSec() < cornerLastBuf.front()->header.stamp.toSec())  // while 循环
				odometryBuf.pop();  // 语句:声明/调用
			if (odometryBuf.empty())  // 条件判断
			{  // 代码块开始
				mBuf.unlock();  // 语句:声明/调用
				break;  // 跳出循环/分支
			}  // 代码块结束

			while (!surfLastBuf.empty() && surfLastBuf.front()->header.stamp.toSec() < cornerLastBuf.front()->header.stamp.toSec())  // while 循环
				surfLastBuf.pop();  // 语句:声明/调用
			if (surfLastBuf.empty())  // 条件判断
			{  // 代码块开始
				mBuf.unlock();  // 语句:声明/调用
				break;  // 跳出循环/分支
			}  // 代码块结束

			while (!fullResBuf.empty() && fullResBuf.front()->header.stamp.toSec() < cornerLastBuf.front()->header.stamp.toSec())  // while 循环
				fullResBuf.pop();  // 语句:声明/调用
			if (fullResBuf.empty())  // 条件判断
			{  // 代码块开始
				mBuf.unlock();  // 语句:声明/调用
				break;  // 跳出循环/分支
			}  // 代码块结束

			timeLaserCloudCornerLast = cornerLastBuf.front()->header.stamp.toSec();  // 语句:赋值/初始化
			timeLaserCloudSurfLast = surfLastBuf.front()->header.stamp.toSec();  // 语句:赋值/初始化
			timeLaserCloudFullRes = fullResBuf.front()->header.stamp.toSec();  // 语句:赋值/初始化
			timeLaserOdometry = odometryBuf.front()->header.stamp.toSec();  // 语句:赋值/初始化

			if (timeLaserCloudCornerLast != timeLaserOdometry ||  // 条件判断
				timeLaserCloudSurfLast != timeLaserOdometry ||  // 语句
				timeLaserCloudFullRes != timeLaserOdometry)  // 语句
			{  // 代码块开始
				printf("time corner %f surf %f full %f odom %f \n", timeLaserCloudCornerLast, timeLaserCloudSurfLast, timeLaserCloudFullRes, timeLaserOdometry);  // 语句:声明/调用
				printf("unsync messeage!");  // 语句:声明/调用
				mBuf.unlock();  // 语句:声明/调用
				break;  // 跳出循环/分支
			}  // 代码块结束

			laserCloudCornerLast->clear();  // 来自里程计:角点
			pcl::fromROSMsg(*cornerLastBuf.front(), *laserCloudCornerLast);  // 来自里程计:角点
			cornerLastBuf.pop();  // 语句:声明/调用

			laserCloudSurfLast->clear();  // 来自里程计:面点
			pcl::fromROSMsg(*surfLastBuf.front(), *laserCloudSurfLast);  // 来自里程计:面点
			surfLastBuf.pop();  // 语句:声明/调用

			laserCloudFullRes->clear();  // 全分辨率点云(局部->全局)
			pcl::fromROSMsg(*fullResBuf.front(), *laserCloudFullRes);  // 全分辨率点云(局部->全局)
			fullResBuf.pop();  // 语句:声明/调用

			q_wodom_curr.x() = odometryBuf.front()->pose.pose.orientation.x;  // 语句:赋值/初始化
			q_wodom_curr.y() = odometryBuf.front()->pose.pose.orientation.y;  // 语句:赋值/初始化
			q_wodom_curr.z() = odometryBuf.front()->pose.pose.orientation.z;  // 语句:赋值/初始化
			q_wodom_curr.w() = odometryBuf.front()->pose.pose.orientation.w;  // 语句:赋值/初始化
			t_wodom_curr.x() = odometryBuf.front()->pose.pose.position.x;  // 语句:赋值/初始化
			t_wodom_curr.y() = odometryBuf.front()->pose.pose.position.y;  // 语句:赋值/初始化
			t_wodom_curr.z() = odometryBuf.front()->pose.pose.position.z;  // 语句:赋值/初始化
			odometryBuf.pop();  // 语句:声明/调用

			while(!cornerLastBuf.empty())  // while 循环
			{  // 代码块开始
				cornerLastBuf.pop();  // 语句:声明/调用
				printf("drop lidar frame in mapping for real time performance \n");  // 语句:声明/调用
			}  // 代码块结束

			mBuf.unlock();  // 语句:声明/调用

			TicToc t_whole;  // 计时(统计耗时)

			transformAssociateToMap();  // 语句:声明/调用

			TicToc t_shift;  // 计时(统计耗时)
			int centerCubeI = int((t_w_curr.x() + 25.0) / 50.0) + laserCloudCenWidth;  // 地图立方格中心索引(W)
			int centerCubeJ = int((t_w_curr.y() + 25.0) / 50.0) + laserCloudCenHeight;  // 地图立方格中心索引(H)
			int centerCubeK = int((t_w_curr.z() + 25.0) / 50.0) + laserCloudCenDepth;  // 地图立方格中心索引(D)

			if (t_w_curr.x() + 25.0 < 0)  // 当前位置(地图系)平移
				centerCubeI--;  // 语句:声明/调用
			if (t_w_curr.y() + 25.0 < 0)  // 当前位置(地图系)平移
				centerCubeJ--;  // 语句:声明/调用
			if (t_w_curr.z() + 25.0 < 0)  // 当前位置(地图系)平移
				centerCubeK--;  // 语句:声明/调用

			while (centerCubeI < 3)  // while 循环
			{  // 代码块开始
				for (int j = 0; j < laserCloudHeight; j++)  // 地图立方格高度
				{  // 代码块开始
					for (int k = 0; k < laserCloudDepth; k++)  // 地图立方格深度
					{   // 代码块开始
						int i = laserCloudWidth - 1;  // 地图立方格宽度
						pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =  // 语句
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];   // 地图立方格宽度
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =  // 语句
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];  // 地图立方格宽度
						for (; i >= 1; i--)  // for 循环
						{  // 代码块开始
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =  // 地图立方格宽度
								laserCloudCornerArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];  // 地图立方格宽度
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =  // 地图立方格宽度
								laserCloudSurfArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];  // 地图立方格宽度
						}  // 代码块结束
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =  // 地图立方格宽度
							laserCloudCubeCornerPointer;  // 语句:声明/调用
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =  // 地图立方格宽度
							laserCloudCubeSurfPointer;  // 语句:声明/调用
						laserCloudCubeCornerPointer->clear();  // 语句:声明/调用
						laserCloudCubeSurfPointer->clear();  // 语句:声明/调用
					}  // 代码块结束
				}  // 代码块结束

				centerCubeI++;  // 语句:声明/调用
				laserCloudCenWidth++;  // 地图立方格中心索引(W)
			}  // 代码块结束

			while (centerCubeI >= laserCloudWidth - 3)  // 地图立方格宽度
			{   // 代码块开始
				for (int j = 0; j < laserCloudHeight; j++)  // 地图立方格高度
				{  // 代码块开始
					for (int k = 0; k < laserCloudDepth; k++)  // 地图立方格深度
					{  // 代码块开始
						int i = 0;  // 语句:赋值/初始化
						pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =  // 语句
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];  // 地图立方格宽度
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =  // 语句
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];  // 地图立方格宽度
						for (; i < laserCloudWidth - 1; i++)  // 地图立方格宽度
						{  // 代码块开始
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =  // 地图立方格宽度
								laserCloudCornerArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];  // 地图立方格宽度
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =  // 地图立方格宽度
								laserCloudSurfArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];  // 地图立方格宽度
						}  // 代码块结束
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =  // 地图立方格宽度
							laserCloudCubeCornerPointer;  // 语句:声明/调用
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =  // 地图立方格宽度
							laserCloudCubeSurfPointer;  // 语句:声明/调用
						laserCloudCubeCornerPointer->clear();  // 语句:声明/调用
						laserCloudCubeSurfPointer->clear();  // 语句:声明/调用
					}  // 代码块结束
				}  // 代码块结束

				centerCubeI--;  // 语句:声明/调用
				laserCloudCenWidth--;  // 地图立方格中心索引(W)
			}  // 代码块结束

			while (centerCubeJ < 3)  // while 循环
			{  // 代码块开始
				for (int i = 0; i < laserCloudWidth; i++)  // 地图立方格宽度
				{  // 代码块开始
					for (int k = 0; k < laserCloudDepth; k++)  // 地图立方格深度
					{  // 代码块开始
						int j = laserCloudHeight - 1;  // 地图立方格高度
						pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =  // 语句
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];  // 地图立方格宽度
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =  // 语句
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];  // 地图立方格宽度
						for (; j >= 1; j--)  // for 循环
						{  // 代码块开始
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =  // 地图立方格宽度
								laserCloudCornerArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];  // 地图立方格宽度
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =  // 地图立方格宽度
								laserCloudSurfArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];  // 地图立方格宽度
						}  // 代码块结束
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =  // 地图立方格宽度
							laserCloudCubeCornerPointer;  // 语句:声明/调用
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =  // 地图立方格宽度
							laserCloudCubeSurfPointer;  // 语句:声明/调用
						laserCloudCubeCornerPointer->clear();  // 语句:声明/调用
						laserCloudCubeSurfPointer->clear();  // 语句:声明/调用
					}  // 代码块结束
				}  // 代码块结束

				centerCubeJ++;  // 语句:声明/调用
				laserCloudCenHeight++;  // 地图立方格中心索引(H)
			}  // 代码块结束

			while (centerCubeJ >= laserCloudHeight - 3)  // 地图立方格高度
			{  // 代码块开始
				for (int i = 0; i < laserCloudWidth; i++)  // 地图立方格宽度
				{  // 代码块开始
					for (int k = 0; k < laserCloudDepth; k++)  // 地图立方格深度
					{  // 代码块开始
						int j = 0;  // 语句:赋值/初始化
						pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =  // 语句
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];  // 地图立方格宽度
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =  // 语句
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];  // 地图立方格宽度
						for (; j < laserCloudHeight - 1; j++)  // 地图立方格高度
						{  // 代码块开始
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =  // 地图立方格宽度
								laserCloudCornerArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];  // 地图立方格宽度
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =  // 地图立方格宽度
								laserCloudSurfArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];  // 地图立方格宽度
						}  // 代码块结束
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =  // 地图立方格宽度
							laserCloudCubeCornerPointer;  // 语句:声明/调用
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =  // 地图立方格宽度
							laserCloudCubeSurfPointer;  // 语句:声明/调用
						laserCloudCubeCornerPointer->clear();  // 语句:声明/调用
						laserCloudCubeSurfPointer->clear();  // 语句:声明/调用
					}  // 代码块结束
				}  // 代码块结束

				centerCubeJ--;  // 语句:声明/调用
				laserCloudCenHeight--;  // 地图立方格中心索引(H)
			}  // 代码块结束

			while (centerCubeK < 3)  // while 循环
			{  // 代码块开始
				for (int i = 0; i < laserCloudWidth; i++)  // 地图立方格宽度
				{  // 代码块开始
					for (int j = 0; j < laserCloudHeight; j++)  // 地图立方格高度
					{  // 代码块开始
						int k = laserCloudDepth - 1;  // 地图立方格深度
						pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =  // 语句
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];  // 地图立方格宽度
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =  // 语句
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];  // 地图立方格宽度
						for (; k >= 1; k--)  // for 循环
						{  // 代码块开始
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =  // 地图立方格宽度
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];  // 地图立方格宽度
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =  // 地图立方格宽度
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];  // 地图立方格宽度
						}  // 代码块结束
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =  // 地图立方格宽度
							laserCloudCubeCornerPointer;  // 语句:声明/调用
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =  // 地图立方格宽度
							laserCloudCubeSurfPointer;  // 语句:声明/调用
						laserCloudCubeCornerPointer->clear();  // 语句:声明/调用
						laserCloudCubeSurfPointer->clear();  // 语句:声明/调用
					}  // 代码块结束
				}  // 代码块结束

				centerCubeK++;  // 语句:声明/调用
				laserCloudCenDepth++;  // 地图立方格中心索引(D)
			}  // 代码块结束

			while (centerCubeK >= laserCloudDepth - 3)  // 地图立方格深度
			{  // 代码块开始
				for (int i = 0; i < laserCloudWidth; i++)  // 地图立方格宽度
				{  // 代码块开始
					for (int j = 0; j < laserCloudHeight; j++)  // 地图立方格高度
					{  // 代码块开始
						int k = 0;  // 语句:赋值/初始化
						pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =  // 语句
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];  // 地图立方格宽度
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =  // 语句
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];  // 地图立方格宽度
						for (; k < laserCloudDepth - 1; k++)  // 地图立方格深度
						{  // 代码块开始
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =  // 地图立方格宽度
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];  // 地图立方格宽度
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =  // 地图立方格宽度
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];  // 地图立方格宽度
						}  // 代码块结束
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =  // 地图立方格宽度
							laserCloudCubeCornerPointer;  // 语句:声明/调用
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =  // 地图立方格宽度
							laserCloudCubeSurfPointer;  // 语句:声明/调用
						laserCloudCubeCornerPointer->clear();  // 语句:声明/调用
						laserCloudCubeSurfPointer->clear();  // 语句:声明/调用
					}  // 代码块结束
				}  // 代码块结束

				centerCubeK--;  // 语句:声明/调用
				laserCloudCenDepth--;  // 地图立方格中心索引(D)
			}  // 代码块结束

			int laserCloudValidNum = 0;  // 语句:赋值/初始化
			int laserCloudSurroundNum = 0;  // 周围可视化点云

			for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++)  // for 循环
			{  // 代码块开始
				for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++)  // for 循环
				{  // 代码块开始
					for (int k = centerCubeK - 1; k <= centerCubeK + 1; k++)  // for 循环
					{  // 代码块开始
						if (i >= 0 && i < laserCloudWidth &&  // 地图立方格宽度
							j >= 0 && j < laserCloudHeight &&  // 地图立方格高度
							k >= 0 && k < laserCloudDepth)  // 地图立方格深度
						{   // 代码块开始
							laserCloudValidInd[laserCloudValidNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;  // 地图立方格宽度
							laserCloudValidNum++;  // 语句:声明/调用
							laserCloudSurroundInd[laserCloudSurroundNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;  // 地图立方格宽度
							laserCloudSurroundNum++;  // 周围可视化点云
						}  // 代码块结束
					}  // 代码块结束
				}  // 代码块结束
			}  // 代码块结束

			laserCloudCornerFromMap->clear();  // 地图角点(用于匹配)
			laserCloudSurfFromMap->clear();  // 地图面点(用于匹配)
			for (int i = 0; i < laserCloudValidNum; i++)  // for 循环
			{  // 代码块开始
				*laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];  // 有效立方格索引缓存
				*laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];  // 有效立方格索引缓存
			}  // 代码块结束
			int laserCloudCornerFromMapNum = laserCloudCornerFromMap->points.size();  // 地图角点(用于匹配)
			int laserCloudSurfFromMapNum = laserCloudSurfFromMap->points.size();  // 地图面点(用于匹配)


			pcl::PointCloud<PointType>::Ptr laserCloudCornerStack(new pcl::PointCloud<PointType>());  // 语句:声明/调用
			downSizeFilterCorner.setInputCloud(laserCloudCornerLast);  // 来自里程计:角点
			downSizeFilterCorner.filter(*laserCloudCornerStack);  // 语句:声明/调用
			int laserCloudCornerStackNum = laserCloudCornerStack->points.size();  // 语句:赋值/初始化

			pcl::PointCloud<PointType>::Ptr laserCloudSurfStack(new pcl::PointCloud<PointType>());  // 语句:声明/调用
			downSizeFilterSurf.setInputCloud(laserCloudSurfLast);  // 来自里程计:面点
			downSizeFilterSurf.filter(*laserCloudSurfStack);  // 语句:声明/调用
			int laserCloudSurfStackNum = laserCloudSurfStack->points.size();  // 语句:赋值/初始化

			printf("map prepare time %f ms\n", t_shift.toc());  // 语句:声明/调用
			printf("map corner num %d  surf num %d \n", laserCloudCornerFromMapNum, laserCloudSurfFromMapNum);  // 地图角点(用于匹配)
			if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 50)  // 地图角点(用于匹配)
			{  // 代码块开始
				TicToc t_opt;  // 计时(统计耗时)
				TicToc t_tree;  // 计时(统计耗时)
				kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMap);  // 地图角点(用于匹配)
				kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMap);  // 地图面点(用于匹配)
				printf("build tree time %f ms \n", t_tree.toc());  // 语句:声明/调用

				for (int iterCount = 0; iterCount < 2; iterCount++)  // for 循环
				{  // 代码块开始
					//ceres::LossFunction *loss_function = NULL;
					ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);  // Ceres 优化相关
					ceres::LocalParameterization *q_parameterization =  // Ceres 优化相关
						new ceres::EigenQuaternionParameterization();  // Ceres 优化相关
					ceres::Problem::Options problem_options;  // Ceres 优化相关

					ceres::Problem problem(problem_options);  // Ceres 优化相关
					problem.AddParameterBlock(parameters, 4, q_parameterization);  // 优化参数(q,t)
					problem.AddParameterBlock(parameters + 4, 3);  // 优化参数(q,t)

					TicToc t_data;  // 计时(统计耗时)
					int corner_num = 0;  // 语句:赋值/初始化

					for (int i = 0; i < laserCloudCornerStackNum; i++)  // for 循环
					{  // 代码块开始
						pointOri = laserCloudCornerStack->points[i];  // 语句:赋值/初始化
						//double sqrtDis = pointOri.x * pointOri.x + pointOri.y * pointOri.y + pointOri.z * pointOri.z;
						pointAssociateToMap(&pointOri, &pointSel);  // 语句:声明/调用
						kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);   // 地图角点KD树

						if (pointSearchSqDis[4] < 1.0)  // 条件判断
						{   // 代码块开始
							std::vector<Eigen::Vector3d> nearCorners;  // 语句:声明/调用
							Eigen::Vector3d center(0, 0, 0);  // 语句:声明/调用
							for (int j = 0; j < 5; j++)  // for 循环
							{  // 代码块开始
								Eigen::Vector3d tmp(laserCloudCornerFromMap->points[pointSearchInd[j]].x,  // 地图角点(用于匹配)
													laserCloudCornerFromMap->points[pointSearchInd[j]].y,  // 地图角点(用于匹配)
													laserCloudCornerFromMap->points[pointSearchInd[j]].z);  // 地图角点(用于匹配)
								center = center + tmp;  // 语句:赋值/初始化
								nearCorners.push_back(tmp);  // 语句:声明/调用
							}  // 代码块结束
							center = center / 5.0;  // 语句:赋值/初始化

							Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();  // 语句:赋值/初始化
							for (int j = 0; j < 5; j++)  // for 循环
							{  // 代码块开始
								Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;  // 语句:赋值/初始化
								covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();  // 语句:赋值/初始化
							}  // 代码块结束

							Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);  // 语句:声明/调用

							// if is indeed line feature
							// note Eigen library sort eigenvalues in increasing order
							Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);  // 语句:赋值/初始化
							Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);  // 语句:声明/调用
							if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])  // 条件判断
							{   // 代码块开始
								Eigen::Vector3d point_on_line = center;  // 语句:赋值/初始化
								Eigen::Vector3d point_a, point_b;  // 语句:声明/调用
								point_a = 0.1 * unit_direction + point_on_line;  // 语句:赋值/初始化
								point_b = -0.1 * unit_direction + point_on_line;  // 语句:赋值/初始化

								ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, point_a, point_b, 1.0);  // Ceres 优化相关
								problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);  // 优化参数(q,t)
								corner_num++;	  // 语句:声明/调用
							}							  // 代码块结束
						}  // 代码块结束
						/*
						else if(pointSearchSqDis[4] < 0.01 * sqrtDis)
						{
							Eigen::Vector3d center(0, 0, 0);
							for (int j = 0; j < 5; j++)
							{
								Eigen::Vector3d tmp(laserCloudCornerFromMap->points[pointSearchInd[j]].x,
													laserCloudCornerFromMap->points[pointSearchInd[j]].y,
													laserCloudCornerFromMap->points[pointSearchInd[j]].z);
								center = center + tmp;
							}
							center = center / 5.0;	
							Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
							ceres::CostFunction *cost_function = LidarDistanceFactor::Create(curr_point, center);
							problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
						}
						*/
					}  // 代码块结束

					int surf_num = 0;  // 语句:赋值/初始化
					for (int i = 0; i < laserCloudSurfStackNum; i++)  // for 循环
					{  // 代码块开始
						pointOri = laserCloudSurfStack->points[i];  // 语句:赋值/初始化
						//double sqrtDis = pointOri.x * pointOri.x + pointOri.y * pointOri.y + pointOri.z * pointOri.z;
						pointAssociateToMap(&pointOri, &pointSel);  // 语句:声明/调用
						kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);  // 地图面点KD树

						Eigen::Matrix<double, 5, 3> matA0;  // 语句:声明/调用
						Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();  // 语句:赋值/初始化
						if (pointSearchSqDis[4] < 1.0)  // 条件判断
						{  // 代码块开始
							
							for (int j = 0; j < 5; j++)  // for 循环
							{  // 代码块开始
								matA0(j, 0) = laserCloudSurfFromMap->points[pointSearchInd[j]].x;  // 地图面点(用于匹配)
								matA0(j, 1) = laserCloudSurfFromMap->points[pointSearchInd[j]].y;  // 地图面点(用于匹配)
								matA0(j, 2) = laserCloudSurfFromMap->points[pointSearchInd[j]].z;  // 地图面点(用于匹配)
								//printf(" pts %f %f %f ", matA0(j, 0), matA0(j, 1), matA0(j, 2));
							}  // 代码块结束
							// find the norm of plane
							Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);  // 语句:赋值/初始化
							double negative_OA_dot_norm = 1 / norm.norm();  // 语句:赋值/初始化
							norm.normalize();  // 语句:声明/调用

							// Here n(pa, pb, pc) is unit norm of plane
							bool planeValid = true;  // 语句:赋值/初始化
							for (int j = 0; j < 5; j++)  // for 循环
							{  // 代码块开始
								// if OX * n > 0.2, then plane is not fit well
								if (fabs(norm(0) * laserCloudSurfFromMap->points[pointSearchInd[j]].x +  // 地图面点(用于匹配)
										 norm(1) * laserCloudSurfFromMap->points[pointSearchInd[j]].y +  // 地图面点(用于匹配)
										 norm(2) * laserCloudSurfFromMap->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)  // 地图面点(用于匹配)
								{  // 代码块开始
									planeValid = false;  // 语句:赋值/初始化
									break;  // 跳出循环/分支
								}  // 代码块结束
							}  // 代码块结束
							Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);  // 语句:声明/调用
							if (planeValid)  // 条件判断
							{  // 代码块开始
								ceres::CostFunction *cost_function = LidarPlaneNormFactor::Create(curr_point, norm, negative_OA_dot_norm);  // Ceres 优化相关
								problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);  // 优化参数(q,t)
								surf_num++;  // 语句:声明/调用
							}  // 代码块结束
						}  // 代码块结束
						/*
						else if(pointSearchSqDis[4] < 0.01 * sqrtDis)
						{
							Eigen::Vector3d center(0, 0, 0);
							for (int j = 0; j < 5; j++)
							{
								Eigen::Vector3d tmp(laserCloudSurfFromMap->points[pointSearchInd[j]].x,
													laserCloudSurfFromMap->points[pointSearchInd[j]].y,
													laserCloudSurfFromMap->points[pointSearchInd[j]].z);
								center = center + tmp;
							}
							center = center / 5.0;	
							Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
							ceres::CostFunction *cost_function = LidarDistanceFactor::Create(curr_point, center);
							problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
						}
						*/
					}  // 代码块结束

					//printf("corner num %d used corner num %d \n", laserCloudCornerStackNum, corner_num);
					//printf("surf num %d used surf num %d \n", laserCloudSurfStackNum, surf_num);

					printf("mapping data assosiation time %f ms \n", t_data.toc());  // 语句:声明/调用

					TicToc t_solver;  // 计时(统计耗时)
					ceres::Solver::Options options;  // Ceres 优化相关
					options.linear_solver_type = ceres::DENSE_QR;  // Ceres 优化相关
					options.max_num_iterations = 4;  // 语句:赋值/初始化
					options.minimizer_progress_to_stdout = false;  // 语句:赋值/初始化
					options.check_gradients = false;  // 语句:赋值/初始化
					options.gradient_check_relative_precision = 1e-4;  // 语句:赋值/初始化
					ceres::Solver::Summary summary;  // Ceres 优化相关
					ceres::Solve(options, &problem, &summary);  // Ceres 优化相关
					printf("mapping solver time %f ms \n", t_solver.toc());  // 语句:声明/调用

					//printf("time %f \n", timeLaserOdometry);
					//printf("corner factor num %d surf factor num %d\n", corner_num, surf_num);
					//printf("result q %f %f %f %f result t %f %f %f\n", parameters[3], parameters[0], parameters[1], parameters[2],
					//	   parameters[4], parameters[5], parameters[6]);
				}  // 代码块结束
				printf("mapping optimization time %f \n", t_opt.toc());  // 语句:声明/调用
			}  // 代码块结束
			else  // 否则分支
			{  // 代码块开始
				ROS_WARN("time Map corner and surf num are not enough");  // 语句:声明/调用
			}  // 代码块结束
			transformUpdate();  // 语句:声明/调用

			TicToc t_add;  // 计时(统计耗时)
			for (int i = 0; i < laserCloudCornerStackNum; i++)  // for 循环
			{  // 代码块开始
				pointAssociateToMap(&laserCloudCornerStack->points[i], &pointSel);  // 语句:声明/调用

				int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;  // 地图立方格中心索引(W)
				int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;  // 地图立方格中心索引(H)
				int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;  // 地图立方格中心索引(D)

				if (pointSel.x + 25.0 < 0)  // 条件判断
					cubeI--;  // 语句:声明/调用
				if (pointSel.y + 25.0 < 0)  // 条件判断
					cubeJ--;  // 语句:声明/调用
				if (pointSel.z + 25.0 < 0)  // 条件判断
					cubeK--;  // 语句:声明/调用

				if (cubeI >= 0 && cubeI < laserCloudWidth &&  // 地图立方格宽度
					cubeJ >= 0 && cubeJ < laserCloudHeight &&  // 地图立方格高度
					cubeK >= 0 && cubeK < laserCloudDepth)  // 地图立方格深度
				{  // 代码块开始
					int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;  // 地图立方格宽度
					laserCloudCornerArray[cubeInd]->push_back(pointSel);  // 每立方格角点集合
				}  // 代码块结束
			}  // 代码块结束

			for (int i = 0; i < laserCloudSurfStackNum; i++)  // for 循环
			{  // 代码块开始
				pointAssociateToMap(&laserCloudSurfStack->points[i], &pointSel);  // 语句:声明/调用

				int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;  // 地图立方格中心索引(W)
				int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;  // 地图立方格中心索引(H)
				int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;  // 地图立方格中心索引(D)

				if (pointSel.x + 25.0 < 0)  // 条件判断
					cubeI--;  // 语句:声明/调用
				if (pointSel.y + 25.0 < 0)  // 条件判断
					cubeJ--;  // 语句:声明/调用
				if (pointSel.z + 25.0 < 0)  // 条件判断
					cubeK--;  // 语句:声明/调用

				if (cubeI >= 0 && cubeI < laserCloudWidth &&  // 地图立方格宽度
					cubeJ >= 0 && cubeJ < laserCloudHeight &&  // 地图立方格高度
					cubeK >= 0 && cubeK < laserCloudDepth)  // 地图立方格深度
				{  // 代码块开始
					int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;  // 地图立方格宽度
					laserCloudSurfArray[cubeInd]->push_back(pointSel);  // 每立方格面点集合
				}  // 代码块结束
			}  // 代码块结束
			printf("add points time %f ms\n", t_add.toc());  // 语句:声明/调用

			
			TicToc t_filter;  // 计时(统计耗时)
			for (int i = 0; i < laserCloudValidNum; i++)  // for 循环
			{  // 代码块开始
				int ind = laserCloudValidInd[i];  // 有效立方格索引缓存

				pcl::PointCloud<PointType>::Ptr tmpCorner(new pcl::PointCloud<PointType>());  // 语句:声明/调用
				downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);  // 每立方格角点集合
				downSizeFilterCorner.filter(*tmpCorner);  // 语句:声明/调用
				laserCloudCornerArray[ind] = tmpCorner;  // 每立方格角点集合

				pcl::PointCloud<PointType>::Ptr tmpSurf(new pcl::PointCloud<PointType>());  // 语句:声明/调用
				downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);  // 每立方格面点集合
				downSizeFilterSurf.filter(*tmpSurf);  // 语句:声明/调用
				laserCloudSurfArray[ind] = tmpSurf;  // 每立方格面点集合
			}  // 代码块结束
			printf("filter time %f ms \n", t_filter.toc());  // 语句:声明/调用
			
			TicToc t_pub;  // 计时(统计耗时)
			//publish surround map for every 5 frame
			if (frameCount % 5 == 0)  // 帧计数
			{  // 代码块开始
				laserCloudSurround->clear();  // 周围可视化点云
				for (int i = 0; i < laserCloudSurroundNum; i++)  // 周围可视化点云
				{  // 代码块开始
					int ind = laserCloudSurroundInd[i];  // 周围立方格索引缓存
					*laserCloudSurround += *laserCloudCornerArray[ind];  // 周围可视化点云
					*laserCloudSurround += *laserCloudSurfArray[ind];  // 周围可视化点云
				}  // 代码块结束

				sensor_msgs::PointCloud2 laserCloudSurround3;  // 周围可视化点云
				pcl::toROSMsg(*laserCloudSurround, laserCloudSurround3);  // 周围可视化点云
				laserCloudSurround3.header.stamp = ros::Time().fromSec(timeLaserOdometry);  // 周围可视化点云
				laserCloudSurround3.header.frame_id = "camera_init";  // 周围可视化点云
				pubLaserCloudSurround.publish(laserCloudSurround3);  // 周围可视化点云
				pcl::io::savePCDFileBinary(
   				 	"/home/slam/map.pcd",
    				*laserCloudSurround);
			}  // 代码块结束

			if (frameCount % 20 == 0)  // 帧计数
			{  // 代码块开始
				pcl::PointCloud<PointType> laserCloudMap;  // 语句:声明/调用
				for (int i = 0; i < 4851; i++)  // for 循环
				{  // 代码块开始
					laserCloudMap += *laserCloudCornerArray[i];  // 每立方格角点集合
					laserCloudMap += *laserCloudSurfArray[i];  // 每立方格面点集合
				}  // 代码块结束
				sensor_msgs::PointCloud2 laserCloudMsg;  // 语句:声明/调用
				pcl::toROSMsg(laserCloudMap, laserCloudMsg);  // PCL->ROS 点云转换
				laserCloudMsg.header.stamp = ros::Time().fromSec(timeLaserOdometry);  // 语句:赋值/初始化
				laserCloudMsg.header.frame_id = "camera_init";  // 语句:赋值/初始化
				pubLaserCloudMap.publish(laserCloudMsg);  // 发布消息
			}  // 代码块结束

			sensor_msgs::PointCloud2 laserCloudFullResLocalMsg;
			pcl::toROSMsg(*laserCloudFullRes, laserCloudFullResLocalMsg);
			laserCloudFullResLocalMsg.header.stamp = ros::Time().fromSec(timeLaserOdometry);
			laserCloudFullResLocalMsg.header.frame_id = "camera_init";
			pubLaserCloudFullResLocal.publish(laserCloudFullResLocalMsg);

			int laserCloudFullResNum = laserCloudFullRes->points.size();  // 全分辨率点云(局部->全局)
			for (int i = 0; i < laserCloudFullResNum; i++)  // 全分辨率点云(局部->全局)
			{  // 代码块开始
				pointAssociateToMap(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);  // 全分辨率点云(局部->全局)
			}  // 代码块结束

			sensor_msgs::PointCloud2 laserCloudFullRes3;  // 全分辨率点云(局部->全局)
			pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);  // 全分辨率点云(局部->全局)
			laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeLaserOdometry);  // 全分辨率点云(局部->全局)
			laserCloudFullRes3.header.frame_id = "camera_init";  // 全分辨率点云(局部->全局)
			pubLaserCloudFullRes.publish(laserCloudFullRes3);  // 全分辨率点云(局部->全局)

			printf("mapping pub time %f ms \n", t_pub.toc());  // 语句:声明/调用

			printf("whole mapping time %f ms +++++\n", t_whole.toc());  // 语句:声明/调用

			nav_msgs::Odometry odomAftMapped;  // 语句:声明/调用
			odomAftMapped.header.frame_id = "camera_init";  // 全局坐标系名字
			odomAftMapped.child_frame_id = "aft_mapped";  //机器人当前位资的坐标系
			odomAftMapped.header.stamp = ros::Time().fromSec(timeLaserOdometry);  // 语句:赋值/初始化
			odomAftMapped.pose.pose.orientation.x = q_w_curr.x();  // 当前位姿(地图系)四元数
			odomAftMapped.pose.pose.orientation.y = q_w_curr.y();  // 当前位姿(地图系)四元数
			odomAftMapped.pose.pose.orientation.z = q_w_curr.z();  // 当前位姿(地图系)四元数
			odomAftMapped.pose.pose.orientation.w = q_w_curr.w();  // 当前位姿(地图系)四元数
			odomAftMapped.pose.pose.position.x = t_w_curr.x();  // 当前位置(地图系)平移
			odomAftMapped.pose.pose.position.y = t_w_curr.y();  // 当前位置(地图系)平移
			odomAftMapped.pose.pose.position.z = t_w_curr.z();  // 当前位置(地图系)平移
			pubOdomAftMapped.publish(odomAftMapped);  // 发布消息

			geometry_msgs::PoseStamped laserAfterMappedPose;  // 语句:声明/调用
			laserAfterMappedPose.header = odomAftMapped.header;  // 语句:赋值/初始化
			laserAfterMappedPose.pose = odomAftMapped.pose.pose;  // 语句:赋值/初始化
			laserAfterMappedPath.header.stamp = odomAftMapped.header.stamp;  // 语句:赋值/初始化
			laserAfterMappedPath.header.frame_id = "camera_init";  // 语句:赋值/初始化
			laserAfterMappedPath.poses.push_back(laserAfterMappedPose);  // 语句:声明/调用
			pubLaserAfterMappedPath.publish(laserAfterMappedPath);  // 发布消息

			static tf::TransformBroadcaster br;  // 语句:声明/调用
			tf::Transform transform;  // 语句:声明/调用
			tf::Quaternion q;  // 语句:声明/调用
			transform.setOrigin(tf::Vector3(t_w_curr(0),  // 当前位置(地图系)平移
											t_w_curr(1),  // 当前位置(地图系)平移
											t_w_curr(2)));  // 当前位置(地图系)平移
			q.setW(q_w_curr.w());  // 当前位姿(地图系)四元数
			q.setX(q_w_curr.x());  // 当前位姿(地图系)四元数
			q.setY(q_w_curr.y());  // 当前位姿(地图系)四元数
			q.setZ(q_w_curr.z());  // 当前位姿(地图系)四元数
			transform.setRotation(q);  // 语句:声明/调用
			br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "camera_init", "aft_mapped"));  // 语句:声明/调用

			frameCount++;  // 帧计数
		}  // 代码块结束
		std::chrono::milliseconds dura(2);  // 语句:声明/调用
        std::this_thread::sleep_for(dura);  // 语句:声明/调用
	}  // 代码块结束
}  // 代码块结束

int main(int argc, char **argv)  // 语句
{  // 代码块开始
	ros::init(argc, argv, "laserMapping");  // 语句:声明/调用
	ros::NodeHandle nh;  // 语句:声明/调用

	float lineRes = 0;  // 语句:赋值/初始化
	float planeRes = 0;  // 语句:赋值/初始化
	nh.param<float>("mapping_line_resolution", lineRes, 0.4);  // 语句:声明/调用
	nh.param<float>("mapping_plane_resolution", planeRes, 0.8);  // 语句:声明/调用
	printf("line resolution %f plane resolution %f \n", lineRes, planeRes);  // 语句:声明/调用
	downSizeFilterCorner.setLeafSize(lineRes, lineRes,lineRes);  // 语句:声明/调用
	downSizeFilterSurf.setLeafSize(planeRes, planeRes, planeRes);  // 语句:声明/调用

	ros::Subscriber subLaserCloudCornerLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 100, laserCloudCornerLastHandler);  // 来自里程计:角点

	ros::Subscriber subLaserCloudSurfLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 100, laserCloudSurfLastHandler);  // 来自里程计:面点

	ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/laser_odom_to_init", 100, laserOdometryHandler);  // ROS 订阅器

	ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 100, laserCloudFullResHandler);  // 全分辨率点云(局部->全局)

	pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 100);  // 语句:赋值/初始化

	pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_map", 100);  // 语句:赋值/初始化

	pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_registered", 100);  // 语句:赋值/初始化

	pubLaserCloudFullResLocal = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_registered_local", 100);

	pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 100);  // 语句:赋值/初始化

	pubOdomAftMappedHighFrec = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init_high_frec", 100);  // 语句:赋值/初始化

	pubLaserAfterMappedPath = nh.advertise<nav_msgs::Path>("/aft_mapped_path", 100);  // 语句:赋值/初始化

	for (int i = 0; i < laserCloudNum; i++)  // 立方格总数
	{  // 代码块开始
		laserCloudCornerArray[i].reset(new pcl::PointCloud<PointType>());  // 每立方格角点集合
		laserCloudSurfArray[i].reset(new pcl::PointCloud<PointType>());  // 每立方格面点集合
	}  // 代码块结束

	std::thread mapping_process{process};  // 语句:声明/调用

	ros::spin();  // 语句:声明/调用

	return 0;  // 返回
}  // 代码块结束
