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

#ifndef TEB_CONFIG_H_
#define TEB_CONFIG_H_

#include <ros/console.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/StdVector>

#include <teb_local_planner/TebLocalPlannerReconfigureConfig.h>


// Definitions
#define USE_ANALYTIC_JACOBI // if available for a specific edge, use analytic jacobi 


namespace teb_local_planner
{

/**
 * @class TebConfig
 * @brief teb_local_planner 及其组件的配置类.
 */  
class TebConfig
{
public:
  
  std::string odom_topic; //!< 里程计消息的主题名称，由机器人驱动程序或模拟器提供
  std::string map_frame; //!< 全局规划坐标系
  
  //! Trajectory related parameters
  struct Trajectory
  { 
    double teb_autosize; //!< 相对于（推荐）时间分辨率的轨迹的使能自动调整大小
    double dt_ref; //!< 所需的轨迹时间分辨率（应该在基本控制率的范围内）
    double dist_step;
    double dt_hysteresis; //!< 根据当前时间分辨率 (dt) 自动调整大小的滞后：通常为 dt_ref 的 10%
    int min_samples; //!< 最小样本数（应始终大于 2）
    int max_samples; //!< 最大样本数；警告：如果太小，离散化/分辨率可能不足以满足给定的机器人模型或避障不再起作用。
    bool global_plan_overwrite_orientation; //!< 覆盖全局规划器提供的局部子目标的方向
    bool allow_init_with_backwards_motion; //!< 如果为 true，则可能会使用向后运动初始化基础轨迹，以防目标位于本地成本图中的起点之后（仅在机器人配备后部传感器时才建议这样做）
    double global_plan_viapoint_sep; //!< 从全局计划中提取的每两个连续通过点之间的最小间隔（如果为负：禁用）
    bool via_points_ordered; //!< 如果为真，则规划器遵循存储容器中的通过点顺序
    double max_global_plan_lookahead_dist; //!< 指定考虑到优化的全局计划子集的最大长度（累积欧几里德距离）[如果 <=0：禁用；长度也受本地成本图大小的限制！]
    bool exact_arc_length; //!< 如果为真，则规划器在速度、加速度和转弯速率计算中使用精确的弧长 [-> 增加的 cpu 时间]，否则使用欧几里德近似。
    double force_reinit_new_goal_dist; //!< Reinitialize the trajectory if a previous goal is updated with a seperation of more than the specified value in meters (skip hot-starting)
    int feasibility_check_no_poses; //!< 指定每个采样间隔应检查预测计划上的位姿的可行性。
    bool egocircle_feasibility;
    bool publish_feedback; //!< 发布包含完整轨迹和活动障碍物列表的规划器反馈（应仅用于评估或调试目的）

  } trajectory; //!< Trajectory related parameters
    
  //! Robot related parameters
  struct Robot
  {
    double max_vel_x; //!< 机器人的最大平移速度
    double max_vel_x_backwards; //!< 机器人向后的最大平移速度
    double max_vel_y; //!< 机器人的最大扫射速度（对于非完整机器人应该为零！）
    double max_vel_theta; //!< 机器人最大角速度
    double acc_lim_x; //!< 机器人最大平移加速度
    double acc_lim_y; //!< 机器人最大扫射加速度
    double acc_lim_theta; //!< 机器人最大角加速度
    double min_turning_radius; //!< 类车机器人的最小转弯半径（差速驱动机器人：零）; 
    double wheelbase; //!< 驱动轴和转向轴之间的距离（仅适用于启用 'cmd_angle_instead_rotvel' 的类车机器人）；对于后轮机器人，该值可能为负！
    bool cmd_angle_instead_rotvel; //!< 用相应的转向角代替指令速度消息中的旋转速度（检查“axles_distance”）
    bool is_footprint_dynamic; //<! 如果为真，则在检查轨迹可行性之前更新足迹
  } robot; //!< Robot related parameters
  
  //! Goal tolerance related parameters
  struct GoalTolerance
  {
    double yaw_goal_tolerance; //!< 允许的最终方向错误
    double xy_goal_tolerance; //!< 到目标位置的允许最终欧几里得距离
    bool free_goal_vel; //!< 出于规划目的，允许机器人的速度为非零（通常为 max_vel）
    bool complete_global_plan; // true 防止机器人在越过最终目标时提前结束路径
  } goal_tolerance; //!< Goal tolerance related parameters

  //! Obstacle related parameters
  struct Obstacles
  {
    double min_obstacle_dist; //!< 与障碍物的最小期望间隔
    double inflation_dist; //!< 具有非零惩罚成本的障碍物周围的缓冲区（应大于 min_obstacle_dist 才能生效）
    double dynamic_obstacle_inflation_dist; //!< 具有非零惩罚成本的动态障碍物预测位置周围的缓冲区（应大于 min_obstacle_dist 才能生效）
    bool include_dynamic_obstacles; //!< 指定是否应通过恒速模型预测动态障碍物的运动（这也会影响同伦类规划）；如果为 false，则所有障碍物都被认为是静态的。
    bool include_costmap_obstacles; //!< 指定是否应直接考虑costmap中的障碍
    bool include_egocircle_obstacles; //!< 指定是否应从自我圈中包含障碍物
    double costmap_obstacles_behind_robot_dist; //!< 限制在机器人后面进行规划时考虑的占用的局部代价地图障碍（以米为单位指定距离）
    int obstacle_poses_affected; //!< 障碍物位置附加到轨迹上最近的位姿以减少计算量，但也要考虑许多邻居
    bool legacy_obstacle_association; //!< 如果为真，则使用旧的关联策略（对于每个障碍物，找到最近的 TEB 位姿），否则使用新的（对于每个 teb 位姿，只找到“相关”障碍物）。
    double obstacle_association_force_inclusion_factor; //!< 非传统障碍物关联技术在优化过程中尝试仅将相关障碍物与离散化轨迹连接，强制包含指定距离内的所有障碍物（作为 min_obstacle_dist 的倍数），例如选择 2.0 以考虑 2.0*min_obstacle_dist 半径内的障碍物。
    double obstacle_association_cutoff_factor; //!< 请参见障碍物关联力_包含_因子，但超出 [值]*min_obstacle_dist 的倍数后，优化期间将忽略所有障碍物。 barrier_association_force_inclusion_factor 首先处理。
    std::string costmap_converter_plugin; //!< 定义 costmap_converter 包的插件名称（costmap 单元转换为点/线/多边形）
    bool costmap_converter_spin_thread; //!< 如果为真，costmap 转换器在不同的线程中调用其回调队列
    int costmap_converter_rate; //!< 定义 costmap_converter 插件处理当前 costmap 频率的速率（该值不应高于 costmap 更新速率）
  } obstacles; //!< Obstacle related parameters

  struct Gaps
  {
    double gap_boundary_exponent;
    double gap_boundary_threshold;
    double gap_boundary_ratio;
  } gaps;
  
  //! Optimization related parameters
  struct Optimization
  {
    int no_inner_iterations; //!< 每次外循环迭代中调用的求解器迭代次数
    int no_outer_iterations; //!< 每次外循环迭代都会自动调整轨迹大小并使用 no_inner_iterations 调用内部优化器
    
    bool optimization_activate; //!< 激活优化
    bool optimization_verbose; //!< 打印详细信息
    
    double penalty_epsilon; //!< 为硬约束近似的惩罚函数添加一个小的安全余量
    
    double weight_max_vel_x; //!< 满足最大允许平移速度的优化权重
    double weight_max_vel_y; //!< 满足最大允许扫射速度的优化权重（仅用于完整机器人）
    double weight_max_vel_theta; //!< 满足最大允许角速度的优化权重
    double weight_acc_lim_x; //!< 满足最大允许平移加速度的优化权重
    double weight_acc_lim_y; //!< 满足最大允许扫射加速度的优化权重（仅用于完整机器人
    double weight_acc_lim_theta; //!< 满足最大允许角加速度的优化权重
    double weight_kinematics_nh; //!< 满足非完整运动学的优化权重
    double weight_kinematics_forward_drive; //!< 强制机器人仅选择正向方向的优化权重（正平移速度，仅 diffdrive 机器人）
    double weight_kinematics_turning_radius; //!< 执行最小转弯半径的优化权重（类似汽车的机器人）
    double weight_optimaltime; //!< 用于收缩相对于过渡时间的轨迹优化权重
    double weight_obstacle; //!< 满足与障碍物的最小距离的优化权重
    double weight_inflation; //!< 膨胀惩罚的优化权重（应该很小）
    double weight_dynamic_obstacle; //!< 满足与动态障碍物的最小间隔的优化权重    
    double weight_dynamic_obstacle_inflation; //!< 动态障碍物膨胀惩罚的优化权重（应该很小）
    double weight_viapoint; //!< 用于最小化到过孔点的距离的优化权重
    double weight_prefer_rotdir; //!< 偏好特定转向方向的优化权重（-> 当前仅在检测到振荡时激活，请参阅“oscillation_recovery”
    double weight_gap;
    double gap_theta_start;
    double gap_theta_end;
    
    double weight_adapt_factor; //!< 一些特殊的权重（当前为“weight_obstacle”）在每个外部 TEB 迭代中被这个因子重复缩放（weight_new = weight_old*factor）；迭代地增加权重而不是先验地设置一个巨大的值会导致潜在优化问题的更好的数值条件。
  } optim; //!< Optimization related parameters
  
  
  struct HomotopyClasses
  {
    bool enable_homotopy_class_planning; //!< 激活同伦类规划（比简单规划需要更多的资源，因为同时优化了多个轨迹）。
    bool enable_multithreading; //!< 激活多线程以并行规划多个轨迹。
    bool simple_exploration; //!< 如果为真，则使用简单的左右方法（通过左侧或右侧的每个障碍物）探索独特的轨迹以生成路径，否则在起点和目标之间的指定区域随机采样可能的路线图。
    bool gap_exploration;
    bool gap_h_signature;
    bool use_gaps;
    
    
    int max_number_classes; //!< 指定允许的替代同伦类的最大数量（限制计算量）
    int feasibility_check_no_tebs;
    double selection_cost_hysteresis; //!< 指定新候选人必须有多少轨迹成本 w.r.t.一个先前选择的轨迹以便被选择（如果 new_cost < old_cost*factor 则选择）。
    double selection_prefer_initial_plan; //!< 为初始计划的等价类中的轨迹指定区间 (0,1) 的成本缩减。
    double selection_obst_cost_scale; //!< 仅用于选择“最佳”候选者的障碍成本项的额外缩放。
    double selection_viapoint_cost_scale; //!< 仅用于选择“最佳”候选者的通过点成本项的额外缩放。
    bool selection_alternative_time_cost; //!< 如果为真，时间成本将被总转换时间所取代。
    double switching_blocking_period; //!< 指定在允许切换到新的等价类之前需要过期的持续时间（以秒为单位）

    int roadmap_graph_no_samples; //! < 如果关闭 simple_exploration，则指定为创建路线图生成的样本数。
    double roadmap_graph_area_width; //!< 在起点和目标之间的矩形区域中对随机关键点/航点进行采样。以米为单位指定该区域的宽度。
    double roadmap_graph_area_length_scale; //!< 矩形区域的长度由起点和终点之间的距离决定。此参数进一步缩放距离，使几何中心保持相等！
    double h_signature_prescaler; //!< 缩放障碍物值的数量以允许大量障碍物。不要选择极低的，否则无法区分障碍物（0.2<H<=1）
    double h_signature_threshold; //!< 如果实部和复部的差均低于指定阈值，则假定两个 h 签名相等。
    
    double obstacle_keypoint_offset; //!< 如果打开了 simple_exploration，则此参数确定障碍物左侧和右侧的距离，在该距离处将创建一个新的关键点（除了 min_obstacle_dist）。
    double obstacle_heading_threshold; //!< 指定障碍物航向和目标航向之间的归一化标量积的值，以便在探索时将它们（障碍物）考虑在内 [0,1]
    double detour_threshold;
    bool viapoints_all_candidates; //!< 如果为真，则不同拓扑的所有轨迹都附加到当前的一组通过点，否则只有与初始/全局计划共享相同的轨迹。
    
    bool visualize_hc_graph; //!< 可视化为探索新的同伦类而创建的图。
    double visualize_with_time_as_z_axis_scale; //!< 如果该值大于 0，则轨迹和障碍物在 3d 中可视化，使用时间作为由该值缩放的 z 轴。对于动态障碍最有用。
  } hcp;
  
  //! Recovery/backup related parameters
  struct Recovery
  {
    bool shrink_horizon_backup; //!< 允许计划者在自动检测到问题的情况下暂时缩小范围 (50%)。
    double shrink_horizon_min_duration; //!< 在检测到不可行轨迹的情况下，指定缩小范围的最小持续时间。
    bool oscillation_recovery; //!< 尝试检测和解决同一等价类中多个解决方案之间的振荡（机器人经常在左/右/前/后之间切换）
    double oscillation_v_eps; //!< 平均归一化线速度的阈值：如果oscillation_v_eps and oscillation_omega_eps两者都没有超过，那么检测到可能的振荡。
    double oscillation_omega_eps; //!< 平均归一化角速度的阈值：如果oscillation_v_eps and oscillation_omega_eps两者都没有超过，那么检测到可能的振荡
    double oscillation_recovery_min_duration; //!< 检测到振荡后激活恢复模式的最短持续时间 [sec]。
    double oscillation_filter_duration; //!< 用于检测振荡的过滤器长度/持续时间 [秒]
  } recovery; //!< Parameters related to recovery and backup strategies

  
  /**
  * @brief Construct the TebConfig using default values.
  * @warning If the \b rosparam server or/and \b dynamic_reconfigure (rqt_reconfigure) node are used,
  *	     the default variables will be overwritten: \n
  *	     E.g. if \e base_local_planner is utilized as plugin for the navigation stack, the initialize() method will register a
  * 	     dynamic_reconfigure server. A subset (not all but most) of the parameters are considered for dynamic modifications.
  * 	     All parameters considered by the dynamic_reconfigure server (and their \b default values) are 
  * 	     set in \e PROJECT_SRC/cfg/TebLocalPlannerReconfigure.cfg. \n
  * 	     In addition the rosparam server can be queried to get parameters e.g. defiend in a launch file.
  * 	     The plugin source (or a possible binary source) can call loadRosParamFromNodeHandle() to update the parameters.
  * 	     In \e summary, default parameters are loaded in the following order (the right one overrides the left ones): \n
  * 		<b>TebConfig Constructor defaults << dynamic_reconfigure defaults << rosparam server defaults</b>
  */
  TebConfig()
  {
    
    odom_topic = "odom";
    map_frame = "odom"; 
    
    // Trajectory
    
    trajectory.teb_autosize = true;
    trajectory.dt_ref = 0.3;
    trajectory.dist_step = 0;
    trajectory.dt_hysteresis = 0.1;
    trajectory.min_samples = 3;
    trajectory.max_samples = 500;
    trajectory.global_plan_overwrite_orientation = true;
    trajectory.allow_init_with_backwards_motion = false;
    trajectory.global_plan_viapoint_sep = -1;
    trajectory.via_points_ordered = false;
    trajectory.max_global_plan_lookahead_dist = 1;
    trajectory.exact_arc_length = false;
    trajectory.force_reinit_new_goal_dist = 1;
    trajectory.feasibility_check_no_poses = 5;
    trajectory.egocircle_feasibility = false;
    trajectory.publish_feedback = false;
    
    // Robot
         
    robot.max_vel_x = 0.4;
    robot.max_vel_x_backwards = 0.2;
    robot.max_vel_y = 0.0;
    robot.max_vel_theta = 0.3;
    robot.acc_lim_x = 0.5;
    robot.acc_lim_y = 0.5;
    robot.acc_lim_theta = 0.5;
    robot.min_turning_radius = 0;
    robot.wheelbase = 1.0;
    robot.cmd_angle_instead_rotvel = false;
    robot.is_footprint_dynamic = false;
    
    // GoalTolerance
    
    goal_tolerance.xy_goal_tolerance = 0.2;
    goal_tolerance.yaw_goal_tolerance = 0.2;
    goal_tolerance.free_goal_vel = false;
    goal_tolerance.complete_global_plan = true;
     
    // Obstacles
    
    obstacles.min_obstacle_dist = 0.5;
    obstacles.inflation_dist = 0.6;
    obstacles.dynamic_obstacle_inflation_dist = 0.6;
    obstacles.include_dynamic_obstacles = true;
    obstacles.include_costmap_obstacles = true;
    obstacles.include_egocircle_obstacles = false;
    obstacles.costmap_obstacles_behind_robot_dist = 1.5;
    obstacles.obstacle_poses_affected = 25;
    obstacles.legacy_obstacle_association = false;
    obstacles.obstacle_association_force_inclusion_factor = 1.5;
    obstacles.obstacle_association_cutoff_factor = 5;
    obstacles.costmap_converter_plugin = "";
    obstacles.costmap_converter_spin_thread = true;
    obstacles.costmap_converter_rate = 5;
    
    
    // Gaps
    
    gaps.gap_boundary_exponent = 1;
    gaps.gap_boundary_threshold=.1;
    gaps.gap_boundary_ratio=.5;
    // Optimization
    
    optim.no_inner_iterations = 5;
    optim.no_outer_iterations = 4;
    optim.optimization_activate = true;
    optim.optimization_verbose = false;
    optim.penalty_epsilon = 0.1;
    optim.weight_max_vel_x = 2; //1
    optim.weight_max_vel_y = 2;
    optim.weight_max_vel_theta = 1;
    optim.weight_acc_lim_x = 1;
    optim.weight_acc_lim_y = 1;
    optim.weight_acc_lim_theta = 1;
    optim.weight_kinematics_nh = 1000;
    optim.weight_kinematics_forward_drive = 1;
    optim.weight_kinematics_turning_radius = 1;
    optim.weight_optimaltime = 1;
    optim.weight_obstacle = 50;
    optim.weight_inflation = 0.1;
    optim.weight_dynamic_obstacle = 50;
    optim.weight_dynamic_obstacle_inflation = 0.1;
    optim.weight_viapoint = 1;
    optim.weight_prefer_rotdir = 50;
    optim.weight_gap = 1;
    optim.gap_theta_start = .26;
    optim.gap_theta_end = 1.05;
    
    optim.weight_adapt_factor = 2.0;
    
    // Homotopy Class Planner
   
    hcp.enable_homotopy_class_planning = true;
    hcp.enable_multithreading = true;
    hcp.simple_exploration = true;
    hcp.gap_exploration = false;
    hcp.gap_h_signature = false;
    hcp.use_gaps = false;
    hcp.max_number_classes = 5; 
    hcp.feasibility_check_no_tebs = 1;
    hcp.selection_cost_hysteresis = 1.0;
    hcp.selection_prefer_initial_plan = 0.95;
    hcp.selection_obst_cost_scale = 100.0;
    hcp.selection_viapoint_cost_scale = 1.0;
    hcp.selection_alternative_time_cost = false;
        
    hcp.obstacle_keypoint_offset = 0.1;
    hcp.obstacle_heading_threshold = 0.45; 
    hcp.roadmap_graph_no_samples = 15;
    hcp.roadmap_graph_area_width = 6; // [m]
    hcp.roadmap_graph_area_length_scale = 1.0;
    hcp.h_signature_prescaler = 1;
    hcp.h_signature_threshold = 0.1;
    hcp.switching_blocking_period = 0.0;
    
    hcp.detour_threshold = -0.1;
    hcp.viapoints_all_candidates = true;
    
    hcp.visualize_hc_graph = false;
    hcp.visualize_with_time_as_z_axis_scale = 0.0;
    
    // Recovery
    
    recovery.shrink_horizon_backup = true;
    recovery.shrink_horizon_min_duration = 10;
    recovery.oscillation_recovery = true;
    recovery.oscillation_v_eps = 0.1;
    recovery.oscillation_omega_eps = 0.1;
    recovery.oscillation_recovery_min_duration = 10;
    recovery.oscillation_filter_duration = 10;


  }
  
  /**
   * @brief Load parmeters from the ros param server.
   * @param nh const reference to the local ros::NodeHandle
   */
  void loadRosParamFromNodeHandle(const ros::NodeHandle& nh);
  
  /**
   * @brief Reconfigure parameters from the dynamic_reconfigure config.
   * Change parameters dynamically (e.g. with <c>rosrun rqt_reconfigure rqt_reconfigure</c>).
   * A reconfigure server needs to be instantiated that calls this method in it's callback.
   * In case of the plugin \e teb_local_planner default values are defined
   * in \e PROJECT_SRC/cfg/TebLocalPlannerReconfigure.cfg.
   * @param cfg Config class autogenerated by dynamic_reconfigure according to the cfg-file mentioned above.
   */
  void reconfigure(TebLocalPlannerReconfigureConfig& cfg);
  
  /**
   * @brief Check parameters and print warnings in case of discrepancies
   * 
   * Call this method whenever parameters are changed using public interfaces to inform the user
   * about some improper uses.
   */
  void checkParameters() const;
  
  /**
   * @brief Check if some deprecated parameters are found and print warnings
   * @param nh const reference to the local ros::NodeHandle
   */
  void checkDeprecated(const ros::NodeHandle& nh) const;
  
  /**
   * @brief Return the internal config mutex
   */
  boost::mutex& configMutex() {return config_mutex_;}
  
private:
  boost::mutex config_mutex_; //!< Mutex for config accesses and changes
  
};


} // namespace teb_local_planner

#endif
