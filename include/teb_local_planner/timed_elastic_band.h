/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 *********************************************************************/

#ifndef TIMED_ELASTIC_BAND_H_
#define TIMED_ELASTIC_BAND_H_

#include <ros/ros.h>
#include <ros/assert.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>

#include <complex>
#include <iterator>

#include <teb_local_planner/obstacles.h>

// G2O Types
#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/vertex_timediff.h>

//#include <turtlebot_trajectory_testing/turtlebot_trajectory_tester.h>

namespace teb_local_planner
{

//! 表示轨迹空间部分的姿势容器
typedef std::vector<VertexPose*> PoseSequence;
//! 定义轨迹时间的时差容器
typedef std::vector<VertexTimeDiff*> TimeDiffSequence;


/**
 * @class TimedElasticBand
 * @brief 类，它定义了一个轨迹，该轨迹被建模为具有增强时间信息的弹性带。
 * 
 * 所有轨迹相关的方法（初始化，修改，...）都在这个类中实现。 \n
 * Let \f$ Q = \lbrace \mathbf{s}_i \rbrace_{i=0...n},\ n \in \mathbb{N} \f$ be a sequence of poses, \n
 * in which \f$ \mathbf{s}_i = [x_i, y_i, \beta_i]^T \in \mathbb{R}^2 \times S^1 \f$ denotes a single pose of the robot. \n
 * 定时弹力带 (TEB) 通过结合时间间隔来增强这个姿势序列
 * two consecutive poses, resuting in a sequence of \c n-1 time intervals \f$ \Delta T_i \f$: \n
 * \f$ \tau = \lbrace \Delta T_i \rbrace_{i=0...n-1} \f$. \n
 * 每个时间间隔（time diff）表示机器人从当前配置过渡到下一个配置所需的时间。
 * 两个序列的元组定义了基础轨迹。
 * 
 * 姿势和时间差异被包装到 g2o::Vertex 类中，以便在 TebOptimalPlanner 中实现高效优化。 \n
 * TebOptimalPlanner 利用这个 Timed_Elastic_band 类来表示可优化的轨迹。
 * 
 * @todo 移动决定是否应将开始或目标状态标记为固定或不固定以优化到 TebOptimalPlanner 类。
 */
class TimedElasticBand
{
public:

  /**
   * @brief 构造类
   */
  TimedElasticBand();
  
  /**
   * @brief 销毁类
   */
  virtual ~TimedElasticBand();

  
  
  /** @name 访问姿势和时间差异序列 */
  //@{
  
  /**
   * @brief 访问完整的姿势序列 
   * @return 对姿势序列的引用
   */
  PoseSequence& poses() {return pose_vec_;};
  
   /**
   * @brief 访问完整的姿势序列（只读）
   * @return 对姿势序列的常量引用
   */
  const PoseSequence& poses() const {return pose_vec_;};
  
  /**
   * @brief 访问完整的 timediff 序列 
   * @return 对 dimediff 序列的引用
   */
  TimeDiffSequence& timediffs() {return timediff_vec_;};
  
  /**
   * @brief 访问完整的 timediff 序列 
   * @return 对 dimediff 序列的引用
   */
  const TimeDiffSequence& timediffs() const {return timediff_vec_;};  
  
  /**
   * @brief 获取时间序列的 pos \c 索引处的时差
   * @param index 内部 TimeDiffSequence 内的索引元素位置
   * @return 对 pos \c 索引处时差的引用
   */
  double& TimeDiff(int index)
  {
    ROS_ASSERT(index<sizeTimeDiffs()); 
    return timediff_vec_.at(index)->dt();
  }
  
  /**
   * @brief 访问时间序列的 pos \c 索引处的时差（只读）
   * @param index 内部 TimeDiffSequence 内的索引元素位置
   * @return const 常量引用 pos \c 索引处的时差
   */
  const double& TimeDiff(int index) const
  {
    ROS_ASSERT(index<sizeTimeDiffs()); 
    return timediff_vec_.at(index)->dt();
  }
  
  /**
   * @brief 在姿势序列的 pos \c 索引处访问姿势
   * @param index 内部 PoseSequence 内的索引元素位置
   * @return 对 pos \c 索引处姿势的引用
   */
  PoseSE2& Pose(int index) 
  {
    ROS_ASSERT(index<sizePoses());
    return pose_vec_.at(index)->pose();
  }
  
  /**
   * @brief 在姿势序列的 pos \c 索引处访问姿势（只读）
   * @param index 内部 PoseSequence 内的索引元素位置
   * @return 对 pos \c 索引处姿势的 const 引用
   */
  const PoseSE2& Pose(int index) const 
  {
    ROS_ASSERT(index<sizePoses());
    return pose_vec_.at(index)->pose();
  }
  
  /**
   * @brief 访问姿势序列中的最后一个 PoseSE2
   */
  PoseSE2& BackPose() {return pose_vec_.back()->pose(); }
  
  /**
   * @brief 访问姿势序列中的最后一个 PoseSE2（只读） 
   */
  const PoseSE2& BackPose() const {return pose_vec_.back()->pose();}
  
  /**
   * @brief 访问时间差异序列中的最后一个 TimeDiff
   */ 
  double& BackTimeDiff() {return timediff_vec_.back()->dt(); }
  
  /**
   * @brief 访问时间差异序列中的最后一个 TimeDiff（只读）
   */  
  const double& BackTimeDiff() const {return timediff_vec_.back()->dt(); }
  
  /**
   * @brief 在 pos \c 索引处访问姿势的顶点以进行优化
   * @param index 内部 PoseSequence 内的索引元素位置
   * @return 指向 pos \c 索引处的位姿顶点的弱原始指针
   */ 
  VertexPose* PoseVertex(int index) 
  {
    ROS_ASSERT(index<sizePoses());
    return pose_vec_.at(index);
  }
  
  /**
   * @brief 在 pos \c 索引处访问时间差的顶点以进行优化
   * @param index 内部 TimeDiffSequence 内的索引元素位置
   * @return 指向 pos \c 索引处 timediff 顶点的弱原始指针
   */  
  VertexTimeDiff* TimeDiffVertex(int index) 
  {
    ROS_ASSERT(index<sizeTimeDiffs()); 
    return timediff_vec_.at(index);
  }
  
  //@}
  
  
  
  /** @name 将新元素附加到姿势和时间差异序列 */
  //@{
  
  /**
   * @brief 在姿势序列的后面附加一个新的姿势顶点 
   * @param pose 姿势 PoseSE2 推回内部 PoseSequence
   * @param fixed 在轨迹优化期间将姿势标记为固定或不固定（对 TebOptimalPlanner 很重要）
   */
  void addPose(const PoseSE2& pose, bool fixed=false);  

  /**
   * @brief 在姿势序列的后面附加一个新的姿势顶点 
   * @param position 表示位置部分的二维向量
   * @param theta 表示方向部分的偏航角
   * @param fixed 在轨迹优化期间将姿势标记为固定或不固定（对 TebOptimalPlanner 很重要）
   */
  void addPose(const Eigen::Ref<const Eigen::Vector2d>& position, double theta, bool fixed=false);
  
  /**
   * @brief 在姿势序列的后面附加一个新的姿势顶点
   * @param x 位置部分的x坐标
   * @param y 位置部分的y坐标
   * @param theta 表示方向部分的偏航角
   * @param fixed 在轨迹优化期间将姿势标记为固定或不固定（对 TebOptimalPlanner 很重要）
   */
  void addPose(double x, double y, double theta, bool fixed=false);
  
  /**
   * @brief 将一个新的时差顶点附加到时差序列的后面 
   * @param dt 时间差值推回内部 TimeDiffSequence
   * @param fixed 在轨迹优化期间将姿势标记为固定或不固定（对 TebOptimalPlanner 很重要）
   */
  void addTimeDiff(double dt, bool fixed=false);
  
  /**
   * @brief 将一个 (pose, timediff) 顶点对附加到当前轨迹的末尾（pose 和 timediff 序列）
   * @param pose 姿势 PoseSE2 推回内部 PoseSequence
   * @param dt 时间差值推回内部 TimeDiffSequence
   * @warning 	由于 timediff 被定义为连接两个连续的姿势，
   *            因此仅当序列中已经存在 n 个姿势和 n-1 个 timediffs (n=1,2,...) 时才允许此调用:
   * 		因此，首先使用 addPose() 添加单个姿势！
   */
  void addPoseAndTimeDiff(const PoseSE2& pose, double dt);
  
  /**
   * @brief 将一个 (pose, timediff) 顶点对附加到当前轨迹的末尾（pose 和 timediff 序列）
   * @param position 表示位置部分的二维向量
   * @param theta 表示方向部分的偏航角
   * @param dt 时间差值推回内部 TimeDiffSequence
   * @warning 见 addPoseAndTimeDiff(const PoseSE2&pose, double dt)
   */
  void addPoseAndTimeDiff(const Eigen::Ref<const Eigen::Vector2d>& position, double theta, double dt);
  
  /**
   * @brief 将一个 (pose, timediff) 顶点对附加到当前轨迹的末尾（pose 和 timediff 序列）
   * @param x 位置部分的x坐标
   * @param y 位置部分的y坐标
   * @param theta 表示方向部分的偏航角
   * @param dt 时间差值推回内部 TimeDiffSequence
   * @warning 见 addPoseAndTimeDiff(const PoseSE2&pose, double dt)
   */
  void addPoseAndTimeDiff(double x, double y, double theta, double dt);
  
  //@}
  

  /** @name 插入新元素并删除姿势和时间差异序列的元素 */
  //@{

  /**
   * @brief 在pos.的索引处的位姿序列插入一个新的姿势顶点
   * @param 内部 PoseSequence 内的索引元素位置
   * @param pose 姿势PoseSE2 元素插入到内部 PoseSequence
   */
  void insertPose(int index, const PoseSE2& pose);
  
  /**
   * @brief 在pos.的索引处的位姿序列插入一个新的姿势顶点
   * @param index 内部 PoseSequence 内的索引元素位置
   * @param position 表示位置部分的二维向量
   * @param theta 表示方向部分的yaw-angle
   */
  void insertPose(int index, const Eigen::Ref<const Eigen::Vector2d>& position, double theta);
  
  /**
   * @brief 在pos.的索引处的位姿序列插入一个新的姿势顶点
   * @param index 内部 PoseSequence 内的索引元素位置
   * @param x 位置部分的x坐标
   * @param y 位置部分的y坐标
   * @param theta 表示方向部分的yaw-angle
   */
  void insertPose(int index, double x, double y, double theta);
  
  /**
   * @brief 在pos.的索引处的时间差异序列插入一个新的时间差异顶点
   * @param 内部 TimeDiffSequence 内的索引元素位置
   * @param dt 时间差异值
   */   
  void insertTimeDiff(int index, double dt);
    
  /**
   * @brief 在pos.的索引处的位姿序列删除一个新的姿势顶点
   * @param index 内部 PoseSequence 内的索引元素位置
   */
  void deletePose(int index);
  
  /**
   * @brief 从位姿序列pos.索引处开始删除多个（number）位姿
   * @param index 内部 PoseSequence 内的第一个元素位置
   * @param number 需要删除的元素个数
   */
  void deletePoses(int index, int number);

  /**
   * @brief 删除时间差异序列中索引为pos.的位姿
   * @param index 内部 TimeDiffSequence 内的索引元素位置
   */
  void deleteTimeDiff(int index);
	
  /** 
   * @brief 删除在时间差异序列中索引为pos.开始的多个（number）时间间隔
   * @param index 第一个元素在内部 TimeDiffSequence 中的位置
   * @param number 需要删除的元素个数
   */
  void deleteTimeDiffs(int index, int number);
  
  //@}
  
  
  /** @name 初始化轨迹 */
  //@{
  
  /**
   * @brief 初始化给定起始姿势和目标姿势之间的轨迹。
   * 
   * 实现的算法使用给定的离散化宽度对起点和目标之间的直线进行二次采样。
   *  
   * 可以使用 diststep 参数在欧几里得空间中定义离散化宽度。
   * 两个连续姿势之间的每个时间差都被初始化为时间步长。
   *  
   * 如果 diststep 选择为零，则生成的轨迹仅包含起始姿势和目标姿势。 
   *  
   * @param start PoseSE2 定义轨迹的起点
   * @param goal PoseSE2 定义轨迹的目标（最终姿势）
   * @param diststep 两个连续姿势之间的欧几里得距离（如果为 0，尽管 min_samples 没有插入中间样本）
   * @param max_vel_x 用于确定时间差的最大平移速度
   * @param min_samples 至少应该初始化的最小样本数
   * @param guess_backwards_motion 如果目标航向指向机器人后面，则允许初始化向后的轨迹
   * @return 如果一切正常，则返回 true，否则返回 false
   */
  bool initTrajectoryToGoal(const PoseSE2& start, const PoseSE2& goal, double diststep=0, double max_vel_x=0.5, int min_samples = 3, bool guess_backwards_motion = false);
  
  
  /**
   * @brief 从通用 2D 参考路径初始化轨迹。
   * 
   * 使用给定的最大速度（包含平移和角速度的 2D 矢量）确定时间信息。 
   *  
   * 实现恒定速度曲线。 \n
   * 如果提供了 max_acceleration 参数，则考虑可能的最大加速度。
   * 
   * 由于方向不包含在参考路径中，因此可以单独提供（例如，从机器人姿势和机器人目标）。
   *  
   * 否则，目标航向将用作起点和目标方向。 \n
   * 沿轨迹的方向将使用参考路径的两个连续位置之间的连接向量来确定。
   *  
   * 
   * 使用容器类的开始和结束迭代器提供引用路径。
   * 您必须提供一个接受取消引用的迭代器并返回对包含 2d 位置的 Eigen::Vector2d 类型的副本或 (const) 引用的一元函数。
   *  
   * 
   * @param path_start 启动通用二维路径的迭代器
   * @param path_end 通用二维路径的结束迭代器
   * @param fun_position 一元函数，返回 Eigen::Vector2d 对象
   * @param max_vel_x 用于确定时间差的最大平移速度
   * @param max_vel_theta 用于确定时间差的最大角速度
   * @param max_acc_x 指定满足最大转换。加速和减速（可选）
   * @param max_acc_theta 指定满足最大角加速度和减速度（可选）
   * @param start_orientation 轨迹第一个位姿的方向（可选，否则使用目标航向）
   * @param goal_orientation 轨迹最后一个位姿的方向（可选，否则使用目标航向）
   * @param min_samples 至少应该初始化的最小样本数
   * @param guess_backwards_motion 如果目标航向指向机器人后面，则允许初始化向后的轨迹
   * @tparam BidirIter 双向迭代器类型
   * @tparam Fun 一元函数，将取消引用的迭代器转换为 Eigen::Vector2d
   * @return 如果一切正常，则返回 true，否则返回 false
   * @remarks 使用 boost::none 跳过可选参数
   */ 
  template<typename BidirIter, typename Fun>
  bool initTrajectoryToGoal(BidirIter path_start, BidirIter path_end, Fun fun_position, double max_vel_x, double max_vel_theta,
		      boost::optional<double> max_acc_x, boost::optional<double> max_acc_theta,
		      boost::optional<double> start_orientation, boost::optional<double> goal_orientation, int min_samples = 3, bool guess_backwards_motion = false);  
  
  template<typename BidirIter, typename Fun>
  bool initTrajectoryToGoal(BidirIter path_start, BidirIter path_end, Fun fun_position, double max_vel_x, double max_vel_theta,
                            boost::optional<double> max_acc_x, boost::optional<double> max_acc_theta,
                            boost::optional<double> start_orientation, boost::optional<double> goal_orientation, int min_samples = 3, bool guess_backwards_motion = false, double diststep =.1);  
  
  /*
  template<typename BidirIter, typename Fun>
  bool initTrajectoryToGoalNI(BidirIter path_start, BidirIter path_end, Fun fun_position, double max_vel_x, double max_vel_theta,
                                                boost::optional<double> max_acc_x, boost::optional<double> max_acc_theta,
                                                boost::optional<double> start_orientation, boost::optional<double> goal_orientation, int min_samples, bool guess_backwards_motion, double diststep, std::shared_ptr<turtlebot_trajectory_testing::GenAndTest>& traj_tester);
  */
  
  /**
   * @brief 从参考姿势序列（位置和方向）初始化轨迹。
   *
   * 此方法使用姿势容器（例如，作为 ros 导航堆栈中的本地计划）初始化定时松紧带。
   *  
   * 两个连续姿势之间的初始时间差可以通过参数 dt 统一设置。
   * 
   * @param plan 计划向量的 geometry_msgs::PoseStamped
   * @param max_vel_x 用于确定时间差的最大平移速度
   * @param estimate_orient 如果 \c 为真，则使用连续姿势之间的直线距离矢量计算方向
   *                        （仅复制开始和目标方向；如果没有可用的方向数据，建议使用）。
   * @param min_samples 至少应该初始化的最小样本数
   * @param guess_backwards_motion 如果目标航向指向机器人后面，则允许初始化后向轨迹（此参数仅在启用 \cestimate_orient 时使用。
   * @return 如果一切正常，则返回 true，否则返回 false
   */
  bool initTrajectoryToGoal(const std::vector<geometry_msgs::PoseStamped>& plan, double max_vel_x, bool estimate_orient=false, int min_samples = 3, bool guess_backwards_motion = false);


  ROS_DEPRECATED bool initTEBtoGoal(const PoseSE2& start, const PoseSE2& goal, double diststep=0, double timestep=1, int min_samples = 3, bool guess_backwards_motion = false)
  {
    ROS_WARN_ONCE("initTEBtoGoal is deprecated and has been replaced by initTrajectoryToGoal. The signature has changed: timestep has been replaced by max_vel_x. \
                   this deprecated method sets max_vel_x = 1. Please update your code.");
    return initTrajectoryToGoal(start, goal, diststep, timestep, min_samples, guess_backwards_motion);
  }

  template<typename BidirIter, typename Fun>
  ROS_DEPRECATED bool initTEBtoGoal(BidirIter path_start, BidirIter path_end, Fun fun_position, double max_vel_x, double max_vel_theta,
          boost::optional<double> max_acc_x, boost::optional<double> max_acc_theta,
          boost::optional<double> start_orientation, boost::optional<double> goal_orientation, int min_samples = 3, bool guess_backwards_motion = false)
  {
    return initTrajectoryToGoal<BidirIter, Fun>(path_start, path_end, fun_position, max_vel_x, max_vel_theta,
                                                max_acc_x, max_acc_theta, start_orientation, goal_orientation, min_samples, guess_backwards_motion);
  }

  ROS_DEPRECATED bool initTEBtoGoal(const std::vector<geometry_msgs::PoseStamped>& plan, double dt, bool estimate_orient=false, int min_samples = 3, bool guess_backwards_motion = false)
  {
    ROS_WARN_ONCE("initTEBtoGoal is deprecated and has been replaced by initTrajectoryToGoal. The signature has changed: dt has been replaced by max_vel_x. \
                   this deprecated method sets max_vel = 1. Please update your code.");
    return initTrajectoryToGoal(plan, dt, estimate_orient, min_samples, guess_backwards_motion);
  }

  
  //@}
  
  /** @name 更新和修改轨迹 */
  //@{
  
  
  /**
   * @brief Hot-Start 从具有更新的开始和目标姿势的现有轨迹开始。
   *
   * 此方法使用新的开始和/或新的目标姿势更新先前优化的轨迹。\n
   * 当前的简单实现切割由于新开始而已经通过的轨迹片段。\n
   * 之后，开始姿势和目标姿势被新姿势取代。由此产生的不连续性将不会被平滑。
   * 优化器必须在 TebOptimalPlanner 中平滑轨迹。\n
   * 
   * @todo 在这里平滑轨迹，测试优化的性能提升。
   * @todo 基于新的参考路径/姿势序列实现 updateAndPruneTEB。
   * 
   * @param new_start 新开始姿势（可选）
   * @param new_goal 新目标姿势（可选）
   * @param min_samples 指定至少应保留在轨迹中的最小样本数
   */  
  void updateAndPruneTEB(boost::optional<const PoseSE2&> new_start, boost::optional<const PoseSE2&> new_goal, int min_samples = 3);
  
  
  /**
   * @brief 根据参考时间分辨率，通过移除或插入 (pose,dt) 对来调整轨迹的大小。
   *
   * 调整轨迹的大小对于以下情况很有帮助：
   * 
   * 	- 障碍需要扩展 teb 以满足给定的离散化宽度（平面分辨率）
   * 	  并避免由于大/小离散化步宽 \f$ \Delta T_i \f$ 而导致的不良行为。 
   *       
   * 	  清除障碍后，teb 应该（重新）收缩到其（时间）最佳版本。
   *    - 如果到目标状态的距离越来越小，则 dt 也在减小。
   *      这导致与许多离散姿势相结合的高度细粒度的离散化。
   *       
   *      因此，计算时间将会/保持很高，
   *      此外还会出现数值不稳定性（例如，由于除以一个小的 \f$ \Delta T_i \f$）。
   *       
   *
   * 实施的策略检查所有 timediffs \f$ \Delta T_i \f$ 和
   * 
   * 	- 如果 \f$ \Delta T_i > \Delta T_{ref} + \Delta T_{hyst} \f$，则插入一个新样本
   *    - 如果 \f$ \Delta T_i < \Delta T_{ref} - \Delta T_{hyst} \f$ 则删除样本
   * 
   * 每次调用仅插入或删除一个新样本（pose-dt-pair）。
   * @param dt_ref 参考时间分辨率
   * @param dt_hysteresis 滞后以避免振荡
   * @param min_samples 调整大小后应保留在轨迹中的最小样本数
   * @param max_samples 调整大小时不应超过的最大样本数
   * @param fast_mode 如果为真，则迭代轨迹一次以插入或删除点； 如果为假，则重复迭代轨迹，直到不再添加或删除任何姿势
   *                   
   */    
  void autoResize(double dt_ref, double dt_hysteresis, int min_samples = 3, int max_samples=1000, bool fast_mode=false);

  /**
   * @brief 在优化期间将姿势序列的 pos \c 索引处的姿势顶点设置为固定或不固定。
   * @param index 姿态顶点的索引
   * @param status 如果\c为真，顶点将被固定，否则不固定
   */
  void setPoseVertexFixed(int index, bool status);
  
  /**
   * @brief 在优化期间将 timediff 序列的 pos 索引处的 timediff 顶点设置为固定或不固定。
   * @param index 到 timediff 顶点的索引
   * @param status 如果\c为真，顶点将被固定，否则不固定
   */
  void setTimeDiffVertexFixed(int index, bool status);
  
  /**
   * @brief 清除轨迹中的所有姿势和时间差异。
   * 姿势和时间差异序列将为空并且 isInit() 将返回 \c false
   */
  void clearTimedElasticBand();
  
  //@}
  
  
  /** @name 实用程序和状态方法 */
  //@{
  
  /**
   * @brief 在轨迹上找到离提供的参考点最近的点。
   * 
   * 此功能可用于查找靠近障碍物的轨迹部分。
   * 
   * @todo 实现一个更高效的版本，首先执行粗略搜索。
   * @todo 实现一个快速近似，假设距离只有一个局部最小值：
   *       允许从轨迹中间开始进行简单比较。
   * 
   * @param ref_point 参考点（二维位置向量）
   * @param[out] distance [optional] 得到的最小距离
   * @param begin_idx 在这个姿势索引处开始搜索
   * @return 姿势序列中最近姿势的索引
   */
  int findClosestTrajectoryPose(const Eigen::Ref<const Eigen::Vector2d>& ref_point, double* distance = NULL, int begin_idx=0) const;

  /**
   * @brief 在轨迹上找到离提供的参考线最近的点。
   * 
   * 此功能可用于查找靠近（线）障碍物的轨迹部分。
   * 
   * @todo 实现一个更高效的版本，首先执行粗略搜索。
   * @todo 实现一个快速近似，假设距离只有一个局部最小值：
   *       允许从轨迹中间开始进行简单比较。
   * 
   * @param ref_line_start 参考线的起点（二维位置向量）
	 * @param ref_line_end 参考线结束（二维位置向量）
   * @param[out] distance [optional] 得到的最小距离
   * @return 姿势序列中最近姿势的索引
   */
  int findClosestTrajectoryPose(const Eigen::Ref<const Eigen::Vector2d>& ref_line_start, const Eigen::Ref<const Eigen::Vector2d>& ref_line_end, double* distance = NULL) const;

  /**
   * @brief 在轨迹上找到与提供的参考多边形最近的点。
   * 
   * 此功能可用于查找靠近（多边形）障碍物的轨迹部分。
   * 
   * @todo 实现一个更高效的版本，首先执行粗略搜索。
   * @todo 实现一个快速近似，假设距离只有一个局部最小值：
   *       允许从轨迹中间开始进行简单比较。
   * 
   * @param vertices 包含 Eigen::Vector2d 点的顶点容器（最后一个点和第一个点相连）
   * @param[out] distance [optional] 得到的最小距离
   * @return 姿势序列中最近姿势的索引
   */
  int findClosestTrajectoryPose(const Point2dContainer& vertices, double* distance = NULL) const;

  /**
   * @brief 在轨迹上找到与提供的障碍物类型最近的点
   * 
   * 此功能可用于查找靠近障碍物的轨迹部分。
   * 该方法是计算点、线和多边形障碍物的适当距离度量。
   * 对于所有未知障碍，使用质心。
   *
   * @param 障碍障碍基类的子类
   * @param[out] distance [optional] 得到的最小距离
   * @return 姿势序列中最近姿势的索引
   */
  int findClosestTrajectoryPose(const Obstacle& obstacle, double* distance = NULL) const;
  
  
  /**
   * @brief 获取内部姿势序列的长度
   */
  int sizePoses() const {return (int)pose_vec_.size();};
  
  /**
   * @brief 获取内部 timediff 序列的长度
   */
  int sizeTimeDiffs() const {return (int)timediff_vec_.size();};
  
  /**
   * @brief 检查轨迹是否初始化（非零位姿和时间差序列）
   */
  bool isInit() const {return !timediff_vec_.empty() && !pose_vec_.empty();}

  /**
   * @brief 计算总转换时间（timediff 序列的所有时间间隔的总和）
   */
  double getSumOfAllTimeDiffs() const;
  
  /**
   * @brief 计算到由 index 表示的位姿的估计转换时间
   * @param index 过渡时间总和的姿势索引
   * @return 估计到姿势索引的过渡时间
   */
  double getSumOfTimeDiffsUpToIdx(int index) const;

  /**
   * @brief 计算轨迹的长度（累积欧式距离）
   */
  double getAccumulatedDistance() const;
  
  /**
   * @brief 检测轨迹是否包含弯路。
   * 
   * 两种不同的情况强制方法返回 \c true :
   * 	1. 轨迹包含增加目标距离的部分。
   * 	   这是通过将方向 theta_i 与目标航向进行比较来检查的。
   * 	   如果标量积低于 \c 阈值参数，则该方法返回 \c true。
   * 	2. 轨迹由轨迹开始时的向后运动组成，
   * 	   例如，第二个姿势在相同目标航向的起始姿势之后。
   * 
   * 绕道并不重要，但如果有多个候选轨迹可用，则可以考虑。
   * @param threshold 允许方向变化的阈值参数（低于 0 -> 大于 90 度）
   * @return \c 如果满足上述两种情况之一，则返回 true，否则返回 false
   */
  bool detectDetoursBackwards(double threshold=0) const;
  
  /**
   * @brief 检查是否所有轨迹点都包含在特定区域中
   * 
   * 特定区域是围绕当前机器人位置 (Pose(0)) 的圆，给定半径 \c 半径。
   * 如果 \c max_dist_behind_robot >= 0，此方法会调查机器人后面的点的不同半径。
   * @param radius 以机器人位置 (Pose(0)) 为中心的区域半径
   * @param max_dist_behind_robot 机器人后方轨迹点的单独半径，如果为 0 或正则激活
   * @param skip_poses If >0: 指定数量的姿势被跳过进行测试，例如 
   * Pose(0), Pose(0+skip_poses+1), Pose(2*skip_poses+2), ... 被测试。
   * @return \c true，如果所有测试的轨迹点都在指定区域内，否则 \c false。
   */
  bool isTrajectoryInsideRegion(double radius, double max_dist_behind_robot=-1, int skip_poses=0);
  
  
  
  //@}
	
protected:
  PoseSequence pose_vec_; //!< Internal container storing the sequence of optimzable pose vertices
  TimeDiffSequence timediff_vec_;  //!< Internal container storing the sequence of optimzable timediff vertices
  
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace teb_local_planner


// include template implementations / definitions
#include <teb_local_planner/timed_elastic_band.hpp>


#endif /* TIMED_ELASTIC_BAND_H_ */
