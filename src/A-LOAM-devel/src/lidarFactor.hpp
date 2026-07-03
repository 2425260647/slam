// Author:   Tong Qin               qintonguav@gmail.com  // 作者信息（原始实现作者）
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk // 作者信息（共同作者）

#include <ceres/ceres.h>                // Ceres Solver：非线性最小二乘优化框架
#include <ceres/rotation.h>             // Ceres：旋转相关工具（此文件中可能未直接使用）
#include <eigen3/Eigen/Dense>           // Eigen：矩阵/向量/四元数
#include <pcl/point_cloud.h>            // PCL：点云容器
#include <pcl/point_types.h>            // PCL：点类型（PointXYZI 等）
#include <pcl/kdtree/kdtree_flann.h>    // PCL：KD-Tree（此文件中可能未直接使用）
#include <pcl_conversions/pcl_conversions.h> // PCL <-> ROS 转换（此文件中可能未直接使用）

struct LidarEdgeFactor                   // 边缘特征残差：点到线（由两点定义）的距离（3 维 residual）
{                                        // struct 开始
	LidarEdgeFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_,          // 构造：当前帧点、上一帧线段端点 a
					Eigen::Vector3d last_point_b_, double s_)                                   // 构造：上一帧线段端点 b、插值系数 s（去畸变用）
		: curr_point(curr_point_), last_point_a(last_point_a_), last_point_b(last_point_b_), s(s_) {} // 成员初始化列表

	template <typename T>                  // 模板：支持 Ceres 的 AutoDiff（T 可能是 double 或 Jet）
	bool operator()(const T *q, const T *t, T *residual) const                            // Ceres 残差计算：参数为四元数 q(4) 与平移 t(3)
	{                                      // operator() 开始

		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())}; // 当前帧点（转换为 T 类型）
		Eigen::Matrix<T, 3, 1> lpa{T(last_point_a.x()), T(last_point_a.y()), T(last_point_a.z())}; // 上一帧线段端点 a
		Eigen::Matrix<T, 3, 1> lpb{T(last_point_b.x()), T(last_point_b.y()), T(last_point_b.z())}; // 上一帧线段端点 b

		//Eigen::Quaternion<T> q_last_curr{q[3], T(s) * q[0], T(s) * q[1], T(s) * q[2]};    // 旧写法：直接缩放四元数虚部（这里注释掉）
		Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};                           // 从参数数组构造四元数（参数存储为 [x,y,z,w]，构造需传入 [w,x,y,z]）
		Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};                            // 单位四元数（无旋转）
		q_last_curr = q_identity.slerp(T(s), q_last_curr);                                  // 按 s 做球面线性插值：实现运动去畸变（从单位旋转插值到估计旋转）
		Eigen::Matrix<T, 3, 1> t_last_curr{T(s) * t[0], T(s) * t[1], T(s) * t[2]};          // 平移同样按 s 线性插值

		Eigen::Matrix<T, 3, 1> lp;                                                          // lp：把当前点变换到“上一帧/起始时刻”坐标系后的点
		lp = q_last_curr * cp + t_last_curr;                                                // 刚体变换：旋转 + 平移

		Eigen::Matrix<T, 3, 1> nu = (lp - lpa).cross(lp - lpb);                              // 叉乘：与线方向相关的面积向量（|nu| = |(lp-lpa) x (lp-lpb)|）
		Eigen::Matrix<T, 3, 1> de = lpa - lpb;                                               // 线方向向量（a-b）

		residual[0] = nu.x() / de.norm();                                                    // 点到直线距离向量分量（除以线长得到距离尺度）
		residual[1] = nu.y() / de.norm();                                                    // residual y 分量
		residual[2] = nu.z() / de.norm();                                                    // residual z 分量

		return true;                                                                         // 告诉 Ceres 计算成功
	}                                      // operator() 结束

	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_a_, // 工厂函数：创建 Ceres CostFunction
									   const Eigen::Vector3d last_point_b_, const double s_)            // 输入与构造参数一致
	{                                      // Create 开始
		return (new ceres::AutoDiffCostFunction<                                             // 使用 AutoDiff 自动求导
				LidarEdgeFactor, 3, 4, 3>(                                                     // residual 维度 3；参数块大小：q=4, t=3
			new LidarEdgeFactor(curr_point_, last_point_a_, last_point_b_, s_)));             // 传入 functor 实例
	}                                      // Create 结束

	Eigen::Vector3d curr_point, last_point_a, last_point_b;                                 // 成员：当前点、上一帧线段端点 a/b
	double s;                                                                               // 成员：插值系数（去畸变比例，通常来自点的时间戳）
};                                        // struct LidarEdgeFactor 结束

struct LidarPlaneFactor                  // 平面特征残差：点到平面（由三点确定）的距离（1 维 residual）
{                                        // struct 开始
	LidarPlaneFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_j_,           // 构造：当前点、上一帧平面点 j
					 Eigen::Vector3d last_point_l_, Eigen::Vector3d last_point_m_, double s_)     // 构造：上一帧平面点 l/m、插值系数 s
		: curr_point(curr_point_), last_point_j(last_point_j_), last_point_l(last_point_l_), // 初始化：保存输入点
		  last_point_m(last_point_m_), s(s_)                                                 // 初始化：保存输入点与 s
	{                                      // 构造函数体开始
		ljm_norm = (last_point_j - last_point_l).cross(last_point_j - last_point_m);        // 用两条边叉乘求平面法向（未归一化）
		ljm_norm.normalize();                                                               // 归一化得到单位法向
	}                                      // 构造函数体结束

	template <typename T>                  // 模板：支持 AutoDiff
	bool operator()(const T *q, const T *t, T *residual) const                             // 计算点到平面的有符号距离残差
	{                                      // operator() 开始

		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())}; // 当前帧点（T 类型）
		Eigen::Matrix<T, 3, 1> lpj{T(last_point_j.x()), T(last_point_j.y()), T(last_point_j.z())}; // 平面上的参考点 j
		//Eigen::Matrix<T, 3, 1> lpl{T(last_point_l.x()), T(last_point_l.y()), T(last_point_l.z())}; // 平面点 l（此实现中不再需要）
		//Eigen::Matrix<T, 3, 1> lpm{T(last_point_m.x()), T(last_point_m.y()), T(last_point_m.z())}; // 平面点 m（此实现中不再需要）
		Eigen::Matrix<T, 3, 1> ljm{T(ljm_norm.x()), T(ljm_norm.y()), T(ljm_norm.z())};       // 单位法向（T 类型）

		//Eigen::Quaternion<T> q_last_curr{q[3], T(s) * q[0], T(s) * q[1], T(s) * q[2]};     // 旧写法：直接缩放四元数虚部（注释掉）
		Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};                            // 参数数组 -> 四元数（[w,x,y,z]）
		Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};                             // 单位四元数
		q_last_curr = q_identity.slerp(T(s), q_last_curr);                                   // 旋转插值：用于去畸变
		Eigen::Matrix<T, 3, 1> t_last_curr{T(s) * t[0], T(s) * t[1], T(s) * t[2]};           // 平移插值：用于去畸变

		Eigen::Matrix<T, 3, 1> lp;                                                           // 变换后的点（上一帧/起始时刻坐标系）
		lp = q_last_curr * cp + t_last_curr;                                                 // 刚体变换：旋转 + 平移

		residual[0] = (lp - lpj).dot(ljm);                                                   // 点到平面的有符号距离：法向 · (点-平面点)

		return true;                                                                          // 计算成功
	}                                      // operator() 结束

	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_j_, // 工厂函数：创建点到平面的残差项
									   const Eigen::Vector3d last_point_l_, const Eigen::Vector3d last_point_m_, // 平面三点
									   const double s_)                                                   // 插值系数 s
	{                                      // Create 开始
		return (new ceres::AutoDiffCostFunction<                                              // AutoDiff 自动求导
				LidarPlaneFactor, 1, 4, 3>(                                                      // residual 维度 1；参数块大小：q=4, t=3
			new LidarPlaneFactor(curr_point_, last_point_j_, last_point_l_, last_point_m_, s_))); // 传入 functor
	}                                      // Create 结束

	Eigen::Vector3d curr_point, last_point_j, last_point_l, last_point_m;                   // 成员：当前点与平面三点
	Eigen::Vector3d ljm_norm;                                                               // 成员：单位法向量
	double s;                                                                               // 成员：插值系数（去畸变比例）
};                                        // struct LidarPlaneFactor 结束

struct LidarPlaneNormFactor              // 平面残差（已知平面单位法向 + 偏置）：点到平面距离（1 维 residual）
{                                        // struct 开始

	LidarPlaneNormFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d plane_unit_norm_,    // 构造：当前点、单位法向
					 double negative_OA_dot_norm_) : curr_point(curr_point_), plane_unit_norm(plane_unit_norm_), // 构造：保存点与法向
										 negative_OA_dot_norm(negative_OA_dot_norm_) {}                   // 构造：保存平面偏置项 d（这里用 -OA·n 的形式）

	template <typename T>                  // 模板：支持 AutoDiff
	bool operator()(const T *q, const T *t, T *residual) const                             // 计算点到平面距离：n·(R p + t) + d
	{                                      // operator() 开始
		Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};                                // 参数数组 -> 四元数（世界到当前的旋转）
		Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};                                   // 平移向量（世界到当前的平移）
		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};  // 当前点（T 类型）
		Eigen::Matrix<T, 3, 1> point_w;                                                      // point_w：点变换到世界坐标系后的坐标
		point_w = q_w_curr * cp + t_w_curr;                                                  // 刚体变换：R p + t

		Eigen::Matrix<T, 3, 1> norm(T(plane_unit_norm.x()), T(plane_unit_norm.y()), T(plane_unit_norm.z())); // 平面单位法向（T 类型）
		residual[0] = norm.dot(point_w) + T(negative_OA_dot_norm);                            // 点到平面的有符号距离（平面方程：n·x + d = 0）
		return true;                                                                         // 计算成功
	}                                      // operator() 结束

	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d plane_unit_norm_, // 工厂函数：创建已知平面的残差项
									   const double negative_OA_dot_norm_)                                // 平面偏置 d
	{                                      // Create 开始
		return (new ceres::AutoDiffCostFunction<                                              // AutoDiff 自动求导
				LidarPlaneNormFactor, 1, 4, 3>(                                                  // residual 维度 1；参数块大小：q=4, t=3
			new LidarPlaneNormFactor(curr_point_, plane_unit_norm_, negative_OA_dot_norm_)));  // 传入 functor
	}                                      // Create 结束

	Eigen::Vector3d curr_point;                                                             // 成员：当前点
	Eigen::Vector3d plane_unit_norm;                                                        // 成员：平面单位法向
	double negative_OA_dot_norm;                                                            // 成员：平面偏置项（d）
};                                        // struct LidarPlaneNormFactor 结束


struct LidarDistanceFactor               // 距离/对齐残差：点到对应点的三维差（3 维 residual）
{                                        // struct 开始

	LidarDistanceFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d closed_point_)        // 构造：当前点与其最近邻/对应点
						: curr_point(curr_point_), closed_point(closed_point_){}                   // 成员初始化

	template <typename T>                  // 模板：支持 AutoDiff
	bool operator()(const T *q, const T *t, T *residual) const                             // 计算点到点的差：R p + t - p_ref
	{                                      // operator() 开始
		Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};                                // 参数数组 -> 四元数
		Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};                                   // 平移向量
		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};  // 当前点（T 类型）
		Eigen::Matrix<T, 3, 1> point_w;                                                      // point_w：变换到世界坐标系的点
		point_w = q_w_curr * cp + t_w_curr;                                                  // 刚体变换：R p + t


		residual[0] = point_w.x() - T(closed_point.x());                                     // x 方向差值
		residual[1] = point_w.y() - T(closed_point.y());                                     // y 方向差值
		residual[2] = point_w.z() - T(closed_point.z());                                     // z 方向差值
		return true;                                                                          // 计算成功
	}                                      // operator() 结束

	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d closed_point_) // 工厂函数：创建点到点残差项
	{                                      // Create 开始
		return (new ceres::AutoDiffCostFunction<                                              // AutoDiff 自动求导
				LidarDistanceFactor, 3, 4, 3>(                                                   // residual 维度 3；参数块大小：q=4, t=3
			new LidarDistanceFactor(curr_point_, closed_point_)));                             // 传入 functor
	}                                      // Create 结束

	Eigen::Vector3d curr_point;                                                             // 成员：当前点
	Eigen::Vector3d closed_point;                                                           // 成员：对应/最近邻点
};                                        // struct LidarDistanceFactor 结束