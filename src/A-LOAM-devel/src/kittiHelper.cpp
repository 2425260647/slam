// Author:   Tong Qin               qintonguav@gmail.com  // 作者信息（原始实现作者）
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk // 作者信息（共同作者）

#include <iostream>                       // 标准输出：std::cout
#include <fstream>                        // 文件读写：std::ifstream
#include <iterator>                       // 迭代器工具（本文件中可能是历史遗留）
#include <string>                         // 字符串：std::string / std::getline / std::stof
#include <vector>                         // 动态数组：std::vector
#include <opencv2/opencv.hpp>             // OpenCV 主头（包含 Mat、imread 等常用接口）
#include <image_transport/image_transport.h> // ROS 图像发布/订阅的 image_transport
#include <opencv2/highgui/highgui.hpp>    // OpenCV HighGUI（图像读写/显示相关，主要为了 imread）
#include <nav_msgs/Odometry.h>            // ROS 消息：里程计 nav_msgs::Odometry
#include <nav_msgs/Path.h>                // ROS 消息：轨迹 nav_msgs::Path
#include <ros/ros.h>                      // ROS 基础：init / NodeHandle / Publisher / Rate 等
#include <rosbag/bag.h>                   // rosbag：写 bag 文件
#include <geometry_msgs/PoseStamped.h>    // ROS 消息：带时间戳/坐标系的位姿 PoseStamped
#include <cv_bridge/cv_bridge.h>          // OpenCV <-> ROS Image 的桥接
#include <sensor_msgs/image_encodings.h>  // ROS 图像编码字符串（例如 mono8）
#include <eigen3/Eigen/Dense>             // Eigen 矩阵/向量/四元数
#include <pcl/point_cloud.h>              // PCL 点云容器 PointCloud
#include <pcl/point_types.h>              // PCL 点类型（PointXYZI 等）
#include <pcl_conversions/pcl_conversions.h> // PCL <-> ROS PointCloud2 转换
#include <sensor_msgs/PointCloud2.h>      // ROS 点云消息：sensor_msgs::PointCloud2

std::vector<float> read_lidar_data(const std::string lidar_data_path) // 读取 KITTI Velodyne 的 .bin（二进制 float32：x y z intensity 반복）
{                                                                     // 函数体开始
    std::ifstream lidar_data_file(lidar_data_path, std::ifstream::in | std::ifstream::binary); // 以二进制模式打开点云文件
    lidar_data_file.seekg(0, std::ios::end);                                                   // 文件指针移到末尾（用于计算文件大小）
    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);                        // 以 float 为单位的元素个数
    lidar_data_file.seekg(0, std::ios::beg);                                                    // 文件指针回到开头（准备读取）

    std::vector<float> lidar_data_buffer(num_elements);                                         // 预分配缓冲区：保存所有 float
    lidar_data_file.read(reinterpret_cast<char*>(&lidar_data_buffer[0]), num_elements * sizeof(float)); // 按字节读取到 float 缓冲区
    return lidar_data_buffer;                                                                   // 返回：按 [x,y,z,intensity,x,y,z,intensity,...] 排列
}                                                                                               // 函数结束

int main(int argc, char** argv)                                                                 // 程序入口：ROS 节点主函数
{                                                                                               // main 函数体开始
    ros::init(argc, argv, "kitti_helper");                                                     // 初始化 ROS 节点（节点名：kitti_helper）
    ros::NodeHandle n("~");                                                                   // 私有命名空间句柄（读取 ~param）
    std::string dataset_folder, sequence_number, output_bag_file;                               // 数据集根目录、序列号、输出 bag 路径
    n.getParam("dataset_folder", dataset_folder);                                             // 参数：数据集根目录（一般以 / 结尾，或在拼接时自己保证）
    n.getParam("sequence_number", sequence_number);                                           // 参数：序列号（例如 00/01/...）
    std::cout << "Reading sequence " << sequence_number << " from " << dataset_folder << '\n'; // 打印提示：当前读取哪个序列
    bool to_bag;                                                                                // 是否把数据写入 rosbag
    n.getParam("to_bag", to_bag);                                                             // 参数：to_bag（true/false）
    if (to_bag)                                                                                 // 如果需要写 bag
        n.getParam("output_bag_file", output_bag_file);                                       // 参数：输出 bag 文件路径
    int publish_delay;                                                                          // 发布节拍“降速”系数（>1 表示更慢）
    n.getParam("publish_delay", publish_delay);                                               // 参数：publish_delay
    publish_delay = publish_delay <= 0 ? 1 : publish_delay;                                     // 防御：publish_delay 非法时强制为 1

    ros::Publisher pub_laser_cloud = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 2); // 发布点云：话题 /velodyne_points，队列长度 2

    image_transport::ImageTransport it(n);                                                       // image_transport 需要一个 NodeHandle
    image_transport::Publisher pub_image_left = it.advertise("/image_left", 2);                // 发布左目图像：话题 /image_left
    image_transport::Publisher pub_image_right = it.advertise("/image_right", 2);              // 发布右目图像：话题 /image_right

    ros::Publisher pubOdomGT = n.advertise<nav_msgs::Odometry>("/odometry_gt", 5);             // 发布真值里程计：话题 /odometry_gt
    nav_msgs::Odometry odomGT;                                                                   // 真值里程计消息缓存（每帧复用）
    odomGT.header.frame_id = "camera_init";                                                   // 坐标系：与 A-LOAM 常用的 /camera_init 对齐
    odomGT.child_frame_id = "ground_truth";                                                   // 子坐标系名：ground_truth（仅用于标识）

    ros::Publisher pubPathGT = n.advertise<nav_msgs::Path>("/path_gt", 5);                     // 发布真值轨迹：话题 /path_gt
    nav_msgs::Path pathGT;                                                                       // 真值轨迹消息（逐帧追加 PoseStamped）
    pathGT.header.frame_id = "camera_init";                                                   // 轨迹坐标系同样设为 /camera_init

    std::string timestamp_path = "sequences/" + sequence_number + "/times.txt";              // KITTI 时间戳文件相对路径：sequences/XX/times.txt
    std::ifstream timestamp_file(dataset_folder + timestamp_path, std::ifstream::in);           // 打开时间戳文件（逐行读取，每行一个时间）

    std::string ground_truth_path = "results/" + sequence_number + ".txt";                   // 真值位姿文件相对路径：results/XX.txt（3x4 矩阵每行一帧）
    std::ifstream ground_truth_file(dataset_folder + ground_truth_path, std::ifstream::in);     // 打开真值文件（逐行读取）

    rosbag::Bag bag_out;                                                                         // rosbag 句柄（可选写入）
    if (to_bag)                                                                                  // 如果需要写入 bag
        bag_out.open(output_bag_file, rosbag::bagmode::Write);                                   // 以写模式打开 bag
                                                                                                 // （注意：这里未做 open 是否成功的检查）
    Eigen::Matrix3d R_transform;                                                                 // 坐标系旋转变换（3x3）
    R_transform << 0, 0, 1, -1, 0, 0, 0, -1, 0;                                                  // 固定旋转矩阵：把 KITTI 真值坐标系旋到 /camera_init 约定
    Eigen::Quaterniond q_transform(R_transform);                                                 // 将旋转矩阵转为四元数（便于旋转叠乘）

    std::string line;                                                                            // 临时字符串：读取一整行文本
    std::size_t line_num = 0;                                                                    // 帧序号（同时用于拼文件名：000000.png/bin）

    ros::Rate r(10.0 / publish_delay);                                                           // 循环频率：默认 10Hz，再按 publish_delay 降速
    while (std::getline(timestamp_file, line) && ros::ok())                                      // 主循环：逐帧读时间戳；ROS 正常则继续
    {                                                                                             // while 循环体开始
        float timestamp = stof(line);                                                            // 将时间戳字符串转为 float 秒（ROS 用 fromSec）
        std::stringstream left_image_path, right_image_path;                                     // 用 stringstream 方便拼接路径并做宽度/填充
        left_image_path << dataset_folder << "sequences/" + sequence_number + "/image_0/"     // 左目图像目录：image_0
                        << std::setfill('0') << std::setw(6) << line_num << ".png";            // 文件名：6 位补零（例如 000123.png）
        cv::Mat left_image = cv::imread(left_image_path.str(), cv::IMREAD_GRAYSCALE);         // 读取左目图像（灰度）；失败时 Mat 为空
        right_image_path << dataset_folder << "sequences/" + sequence_number + "/image_1/"    // 右目图像目录：image_1
                         << std::setfill('0') << std::setw(6) << line_num << ".png";           // 文件名同样按帧号补零
        cv::Mat right_image = cv::imread(left_image_path.str(), cv::IMREAD_GRAYSCALE);        // 读取右目图像（灰度）——注意：这里用的是 left_image_path，疑似笔误

        std::getline(ground_truth_file, line);                                                    // 读取对应帧的真值位姿一行（3x4 展开为 12 个数）
        std::stringstream pose_stream(line);                                                     // 把这一行包装成流，便于按空格切 token
        std::string s;                                                                           // 临时 token 字符串
        Eigen::Matrix<double, 3, 4> gt_pose;                                                     // 真值位姿矩阵：前三列 R，最后一列 t
        for (std::size_t i = 0; i < 3; ++i)                                                      // 逐行读取 3 行
        {                                                                                         // 外层 for 开始
            for (std::size_t j = 0; j < 4; ++j)                                                  // 每行 4 列
            {                                                                                     // 内层 for 开始
                std::getline(pose_stream, s, ' ');                                               // 按空格读取一个数字字符串（连续空格会产生空 token，需注意数据格式）
                gt_pose(i, j) = stof(s);                                                         // 转成 double 并写入矩阵
            }                                                                                     // 内层 for 结束
        }                                                                                         // 外层 for 结束

        Eigen::Quaterniond q_w_i(gt_pose.topLeftCorner<3, 3>());                                 // 从 3x3 旋转矩阵构造四元数（世界->相机/IMU 视约定而定）
        Eigen::Quaterniond q = q_transform * q_w_i;                                              // 叠加坐标变换：先 q_w_i，再应用固定旋转 q_transform
        q.normalize();                                                                           // 归一化，避免数值误差导致四元数不单位
        Eigen::Vector3d t = q_transform * gt_pose.topRightCorner<3, 1>();                        // 平移向量也需要同样旋转到目标坐标系

        odomGT.header.stamp = ros::Time().fromSec(timestamp);                                    // 设置消息时间戳（用数据集时间而非 now）
        odomGT.pose.pose.orientation.x = q.x();                                                  // 写入四元数 x
        odomGT.pose.pose.orientation.y = q.y();                                                  // 写入四元数 y
        odomGT.pose.pose.orientation.z = q.z();                                                  // 写入四元数 z
        odomGT.pose.pose.orientation.w = q.w();                                                  // 写入四元数 w
        odomGT.pose.pose.position.x = t(0);                                                      // 写入平移 x
        odomGT.pose.pose.position.y = t(1);                                                      // 写入平移 y
        odomGT.pose.pose.position.z = t(2);                                                      // 写入平移 z
        pubOdomGT.publish(odomGT);                                                               // 发布真值里程计

        geometry_msgs::PoseStamped poseGT;                                                       // 单帧位姿（用于拼 Path）
        poseGT.header = odomGT.header;                                                           // 复用同一 header（时间戳与 frame_id）
        poseGT.pose = odomGT.pose.pose;                                                          // 位姿内容与里程计一致
        pathGT.header.stamp = odomGT.header.stamp;                                               // Path 自身 header 的时间戳更新为当前帧
        pathGT.poses.push_back(poseGT);                                                          // 将当前帧位姿追加到轨迹末尾
        pubPathGT.publish(pathGT);                                                               // 发布真值轨迹

        // read lidar point cloud                                                           // 读 KITTI Velodyne 点云（二进制 .bin）
        std::stringstream lidar_data_path;                                                       // 拼接点云文件路径
        lidar_data_path << dataset_folder << "velodyne/sequences/" + sequence_number + "/velodyne/" // 点云目录：velodyne/sequences/XX/velodyne/
                        << std::setfill('0') << std::setw(6) << line_num << ".bin";            // 文件名：6 位补零，扩展名 .bin
        std::vector<float> lidar_data = read_lidar_data(lidar_data_path.str());                  // 读出所有 float（长度应为 4*N）
        std::cout << "totally " << lidar_data.size() / 4.0 << " points in this lidar frame \n"; // 打印点数（每点 4 个 float）

        std::vector<Eigen::Vector3d> lidar_points;                                               // 点坐标列表（Eigen 形式；本文件后续未使用，可能用于调试/扩展）
        std::vector<float> lidar_intensities;                                                    // 强度列表（同样后续未使用）
        pcl::PointCloud<pcl::PointXYZI> laser_cloud;                                             // PCL 点云（x,y,z,intensity）
        for (std::size_t i = 0; i < lidar_data.size(); i += 4)                                   // 遍历每个点：每 4 个 float 为一组
        {                                                                                         // for 循环体开始
            lidar_points.emplace_back(lidar_data[i], lidar_data[i + 1], lidar_data[i + 2]);       // 保存点坐标（x,y,z）
            lidar_intensities.push_back(lidar_data[i + 3]);                                      // 保存强度 intensity

            pcl::PointXYZI point;                                                                // 构造一个 PCL 点
            point.x = lidar_data[i];                                                             // 点的 x
            point.y = lidar_data[i + 1];                                                         // 点的 y
            point.z = lidar_data[i + 2];                                                         // 点的 z
            point.intensity = lidar_data[i + 3];                                                 // 点的强度
            laser_cloud.push_back(point);                                                        // 追加到 PCL 点云
        }                                                                                         // for 循环体结束

        sensor_msgs::PointCloud2 laser_cloud_msg;                                                // ROS 点云消息容器
        pcl::toROSMsg(laser_cloud, laser_cloud_msg);                                             // PCL 点云 -> ROS PointCloud2
        laser_cloud_msg.header.stamp = ros::Time().fromSec(timestamp);                           // 点云时间戳：使用数据集时间
        laser_cloud_msg.header.frame_id = "camera_init";                                      // 点云坐标系：/camera_init
        pub_laser_cloud.publish(laser_cloud_msg);                                                // 发布点云

        sensor_msgs::ImagePtr image_left_msg = cv_bridge::CvImage(laser_cloud_msg.header, "mono8", left_image).toImageMsg();   // Mat -> ROS Image（左目，mono8）
        sensor_msgs::ImagePtr image_right_msg = cv_bridge::CvImage(laser_cloud_msg.header, "mono8", right_image).toImageMsg(); // Mat -> ROS Image（右目，mono8）
        pub_image_left.publish(image_left_msg);                                                  // 发布左目图像
        pub_image_right.publish(image_right_msg);                                                // 发布右目图像

        if (to_bag)                                                                              // 如果需要写入 bag
        {                                                                                         // if 代码块开始
            bag_out.write("/image_left", ros::Time::now(), image_left_msg);                     // 写入左目图像（注意：这里使用 now，而不是数据集 timestamp）
            bag_out.write("/image_right", ros::Time::now(), image_right_msg);                   // 写入右目图像
            bag_out.write("/velodyne_points", ros::Time::now(), laser_cloud_msg);               // 写入点云
            bag_out.write("/path_gt", ros::Time::now(), pathGT);                                // 写入真值轨迹
            bag_out.write("/odometry_gt", ros::Time::now(), odomGT);                            // 写入真值里程计
        }                                                                                         // if 代码块结束

        line_num++;                                                                              // 帧号 +1（下一帧文件名/索引）
        r.sleep();                                                                               // 按设定频率 sleep（控制发布节奏）
    }                                                                                             // while 循环结束
    bag_out.close();                                                                             // 关闭 bag（若未 open，具体行为取决于 rosbag 实现）
    std::cout << "Done \n";                                                                   // 打印结束提示


    return 0;                                                                                    // 正常退出
}                                                                                                 // main 结束
