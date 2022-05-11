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


#ifndef OBSTACLES_H
#define OBSTACLES_H

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Geometry>

#include <complex>

#include <boost/shared_ptr.hpp>
#include <boost/pointer_cast.hpp>

#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/QuaternionStamped.h>

#include <tf/tf.h>
#include <teb_local_planner/distance_calculations.h>


namespace teb_local_planner
{

/**
 * @class 障碍物
 * @brief 定义障碍物建模接口的抽象类
 */
class Obstacle
{
public:
  
  /**
    * @brief 抽象障碍类的默认构造函数
    */
  Obstacle() : dynamic_(false), centroid_velocity_(Eigen::Vector2d::Zero())
  {
  }
  
  /**
   * @brief 虚拟析构函数
   */
  virtual ~Obstacle()
  {
  }


  /** @name 质心坐标（抽象，取决于障碍物类型） */
  //@{ 

  /**
    * @brief 获取障碍物的质心坐标
    * @return Eigen::Vector2d 包含质心
    */
  virtual const Eigen::Vector2d& getCentroid() const = 0;

  /**
    * @brief 以复数形式获取障碍物的质心坐标
    * @return std::complex 包含质心坐标
    */
  virtual std::complex<double> getCentroidCplx() const = 0;

  //@}


  /** @name 碰撞检查和距离计算（抽象，取决于障碍物类型）*/
  //@{ 

  /**
    * @brief 检查给定点是否与障碍物碰撞
    * @param position 应检查的二维参考位置
    * @param min_dist 允许到障碍物无碰撞的最小距离
    * @return \c true 如果位置在障碍物区域内或最小距离小于min_dist，则为true
    */
  virtual bool checkCollision(const Eigen::Vector2d& position, double min_dist) const = 0;

  /**
    * @brief 检查两点之间的给定线段是否与障碍物相交（并另外保持安全距离 \c min_dist）
    * @param 参考线末端的二维点
    * @param line_end 参考线末端的二维点
    * @param min_dist 允许到障碍物无碰撞/交叉的最小距离
    * @return \c 如果给定线与障碍物区域相交或最小距离小于 min_dist，则为 true
    */
  virtual bool checkLineIntersection(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double min_dist=0) const = 0;

  /**
    * @brief 获取到障碍物的最小欧式距离（点作为参考）
    * @param position 2d 参考位置
    * @return 距离障碍物最近的距离
    */
  virtual double getMinimumDistance(const Eigen::Vector2d& position) const = 0;

  /**
   * @brief 获取到障碍物的最小欧式距离（线作为参考）
   * @param line_start 参考线开始的二维位置
   * @param line_end 参考线结束的二维位置
   * @return 距离障碍物最近的距离
   */
  virtual double getMinimumDistance(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end) const = 0;
  
  /**
   * @brief 获取到障碍物的最小欧式距离（多边形作为参考）
   * @param polygon 顶点（2D 点）描述一个封闭的多边形
   * @return 距离障碍物最近的距离
   */
  virtual double getMinimumDistance(const Point2dContainer& polygon) const = 0;

  /**
   * @brief 获取指定参考位置的障碍物边界上最近的点
   * @param position 参考二维位置
   * @return 障碍物边界上的最近点
   */
  virtual Eigen::Vector2d getClosestPoint(const Eigen::Vector2d& position) const = 0;

  //@}



  /** @name 非静态、移动障碍物的速度相关方法 */
  //@{ 

  /**
    * @brief 使用等速模型（点作为参考）获取到移动障碍物的估计最小时空距离
    * @param position 2d 参考位置
    * @param t 时间，估计到障碍物的最小距离
    * @return 在时间 t 到障碍物的最近可能距离
    */
  virtual double getMinimumSpatioTemporalDistance(const Eigen::Vector2d& position, double t) const = 0;

  /**
    * @brief 使用等速模型（以线为参考）获取到移动障碍物的估计最小时空距离
    * @param line_start 参考线开始的二维位置
    * @param line_end 参考线结束的二维位置
    * @param t 时间，估计到障碍物的最小距离
    * @return 在时间 t 到障碍物的最近可能距离
    */
  virtual double getMinimumSpatioTemporalDistance(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double t) const = 0;

  /**
    * @brief 使用等速模型（多边形作为参考）获取到移动障碍物的估计最小时空距离
    * @param polygon 顶点（2D 点）描述一个封闭的多边形
    * @param t 时间，估计到障碍物的最小距离
    * @return 在时间 t 到障碍物的最近可能距离
    */
  virtual double getMinimumSpatioTemporalDistance(const Point2dContainer& polygon, double t) const = 0;

  /**
    * @brief 假设恒速模型预测质心的位置
    * @param[in]  t         时间以秒为单位进行预测 (t>=0)
    * @param[out] position 预测的质心的 2d 位置
    */
  virtual void predictCentroidConstantVelocity(double t, Eigen::Ref<Eigen::Vector2d> position) const
  {
    position = getCentroid() + t * getCentroidVelocity();
  }

  /**
    * @brief 检查障碍物是否以（非零）速度移动
    * @return \c 如果障碍物没有被标记为静态，则为 true，否则为 \c false
    */	
  bool isDynamic() const {return dynamic_;}

  /**
    * @brief 将障碍物 wrt 的 2d 速度 (vx, vy) 设置为质心
    * @remarks 使用此功能设置速度将障碍物标记为动态（@see isDynamic）
    * @param vel 2D 向量，包含质心在 x 和 y 方向上的速度
    */
  void setCentroidVelocity(const Eigen::Ref<const Eigen::Vector2d>& vel) {centroid_velocity_ = vel; dynamic_=true;} 

  /**
    * @brief 将障碍物 wrt 的 2d 速度 (vx, vy) 设置为质心
    * @remarks 使用此功能设置速度将障碍物标记为动态（@see isDynamic）
    * @param velocity geometry_msgs::TwistWithCovariance 包含障碍物的速度
    * @param orientation geometry_msgs::QuaternionStamped 包含障碍物的方向
    */
  void setCentroidVelocity(const geometry_msgs::TwistWithCovariance& velocity,
                           const geometry_msgs::Quaternion& orientation)
  {
    // 设置速度，如果障碍物在移动
    Eigen::Vector2d vel;
    vel.coeffRef(0) = velocity.twist.linear.x;
    vel.coeffRef(1) = velocity.twist.linear.y;

    // 如果速度范数小于 0.001，则认为障碍物不是动态的
    // TODO: Get rid of constant
    if (vel.norm() < 0.001)
      return;

    // 当前阶段发布的速度已经在地图框中给出
//    double yaw = tf::getYaw(orientation.quaternion);
//    ROS_INFO("Yaw: %f", yaw);
//    Eigen::Rotation2Dd rot(yaw);
//    vel = rot * vel;
    setCentroidVelocity(vel);
  }

  void setCentroidVelocity(const geometry_msgs::TwistWithCovariance& velocity,
                           const geometry_msgs::QuaternionStamped& orientation)
  {
    setCentroidVelocity(velocity, orientation.quaternion);
  }

  /**
    * @brief 获取障碍物速度 (vx, vy) (wrt to the centroid)
    * @returns 包含质心在 x 和 y 方向上的速度的二维向量
    */
  const Eigen::Vector2d& getCentroidVelocity() const {return centroid_velocity_;}

  //@}



  /** @name 辅助函数 */
  //@{ 
  
  /**
   * @brief 将障碍物转换为多边形消息
   * 
   * 将障碍物转换为相应的多边形消息。
   * 点障碍物有一个顶点，线有两个顶点
   * 和多边形可能是隐式闭合的，因此起始顶点不能重复。
   * @param[out] polygon 多边形消息
   */
  virtual void toPolygonMsg(geometry_msgs::Polygon& polygon) = 0;

  virtual void toTwistWithCovarianceMsg(geometry_msgs::TwistWithCovariance& twistWithCovariance)
  {
    if (dynamic_)
    {
      twistWithCovariance.twist.linear.x = centroid_velocity_(0);
      twistWithCovariance.twist.linear.y = centroid_velocity_(1);
    }
    else
    {
      twistWithCovariance.twist.linear.x = 0;
      twistWithCovariance.twist.linear.y = 0;
    }

    // TODO:协方差
  }

  //@}
	
protected:
	   
  bool dynamic_; //!< 如果障碍物是动态的（分别是移动的障碍物），则存储标志
  Eigen::Vector2d centroid_velocity_; //!< 存储质心的对应速度 (vx, vy)（零，如果 _dynamic 为 \c 真）
  
public:	
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


//! 缩写。用于共享障碍物指针
typedef boost::shared_ptr<Obstacle> ObstaclePtr;
//! 缩写。对于共享障碍 const 指针
typedef boost::shared_ptr<const Obstacle> ObstacleConstPtr;
//! 缩写。用于存放多个障碍物的容器
typedef std::vector<ObstaclePtr> ObstContainer;



/**
 * @class PointObstacle
 * @brief 实现一个 2D 点障碍物
 */
class PointObstacle : public Obstacle
{
public:
  
  /**
    * @brief 点障碍类的默认构造函数
    */
  PointObstacle() : Obstacle(), pos_(Eigen::Vector2d::Zero())
  {}
  
  /**
    * @brief 使用二维位置向量构造 PointObstacle
    * @param position 定义当前障碍物位置的 2d 位置
    */
  PointObstacle(const Eigen::Ref< const Eigen::Vector2d>& position) : Obstacle(), pos_(position)
  {}
  
  /**
    * @brief 使用 x 和 y 坐标构造 PointObstacle
    * @param x x 坐标
    * @param y y 坐标
    */      
  PointObstacle(double x, double y) : Obstacle(), pos_(Eigen::Vector2d(x,y))
  {}


  // 实现基类的 checkCollision()
  virtual bool checkCollision(const Eigen::Vector2d& point, double min_dist) const
  {
      return getMinimumDistance(point) < min_dist;
  }
  
  
  // 实现基类的 checkLineIntersection()
  virtual bool checkLineIntersection(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double min_dist=0) const
  {   
      // 距离线 - 圆
      // 参考 http://www.spieleprogrammierer.de/wiki/2D-Kollisionserkennung#Kollision_Kreis-Strecke
      Eigen::Vector2d a = line_end-line_start; // not normalized!  a=y-x
      Eigen::Vector2d b = pos_-line_start; // b=m-x
      
      // 现在找到离圆最近的点 v=x+a*t，t=a*b/(a*a) 并绑定到 0<=t<=1
      double t = a.dot(b)/a.dot(a);
      if (t<0) t=0; // bound t (since a is not normalized, t can be scaled between 0 and 1 to parametrize the line
      else if (t>1) t=1;
      Eigen::Vector2d nearest_point = line_start + a*t;
      
      // 检查碰撞
      return checkCollision(nearest_point, min_dist);
  }

  
  // 实现基类的getMinimumDistance()
  virtual double getMinimumDistance(const Eigen::Vector2d& position) const
  {
    return (position-pos_).norm();
  }
  
  // 实现基类的getMinimumDistance()
  virtual double getMinimumDistance(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end) const
  {
    return distance_point_to_segment_2d(pos_, line_start, line_end);
  }
  
  // 实现基类的getMinimumDistance()
  virtual double getMinimumDistance(const Point2dContainer& polygon) const
  {
    return distance_point_to_polygon_2d(pos_, polygon);
  }
  
  // 实现基类的getMinimumDistanceVec()
  virtual Eigen::Vector2d getClosestPoint(const Eigen::Vector2d& position) const
  {
    return pos_;
  }
  
  // 实现基类的getMinimumSpatioTemporalDistance()
  virtual double getMinimumSpatioTemporalDistance(const Eigen::Vector2d& position, double t) const
  {
    return (pos_ + t*centroid_velocity_ - position).norm();
  }

  // 实现基类的getMinimumSpatioTemporalDistance()
  virtual double getMinimumSpatioTemporalDistance(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double t) const
  {
    return distance_point_to_segment_2d(pos_ + t*centroid_velocity_, line_start, line_end);
  }

  // 实现基类的getMinimumSpatioTemporalDistance()
  virtual double getMinimumSpatioTemporalDistance(const Point2dContainer& polygon, double t) const
  {
    return distance_point_to_polygon_2d(pos_ + t*centroid_velocity_, polygon);
  }

  // 实现基类的 predictCentroidConstantVelocity()
  virtual void predictCentroidConstantVelocity(double t, Eigen::Ref<Eigen::Vector2d> position) const
  {
    position = pos_ + t*centroid_velocity_;
  }

  // 实现基类的getCentroid()
  virtual const Eigen::Vector2d& getCentroid() const
  {
    return pos_;
  }
  
  // 实现基类的getCentroidCplx()
  virtual std::complex<double> getCentroidCplx() const
  {
    return std::complex<double>(pos_[0],pos_[1]);
  }
  
  // 访问器方法
  const Eigen::Vector2d& position() const {return pos_;} //!< 返回障碍物的当前位置（只读）
  Eigen::Vector2d& position() {return pos_;} //!< 返回障碍物的当前位置
  double& x() {return pos_.coeffRef(0);} //!< 返回障碍物的当前 x 坐标
  const double& x() const {return pos_.coeffRef(0);} //!< 返回障碍物的当前 y 坐标（只读）
  double& y() {return pos_.coeffRef(1);} //!< 返回障碍物的当前 x 坐标
  const double& y() const {return pos_.coeffRef(1);} //!< 返回障碍物的当前 y 坐标（只读）
      
  // 实现基类的 toPolygonMsg()
  virtual void toPolygonMsg(geometry_msgs::Polygon& polygon)
  {
    polygon.points.resize(1);
    polygon.points.front().x = pos_.x();
    polygon.points.front().y = pos_.y();
    polygon.points.front().z = 0;
  }
      
protected:
  
  Eigen::Vector2d pos_; //!< 存储 PointObstacle 的位置
  
  	
public:	
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
};

/**
 * @class CircularObstacle
 * @brief 实现一个 2D 圆形障碍物（点障碍物加半径）
 */
class CircularObstacle : public Obstacle
{
public:

  /**
    * @brief 圆形障碍物类的默认构造函数
    */
  CircularObstacle() : Obstacle(), pos_(Eigen::Vector2d::Zero())
  {}

  /**
    * @brief 使用二维中心位置向量和半径构造 CircularObstacle
    * @param position 2d 定义当前障碍物位置的 2d 位置
    * @param radius 障碍物的半径
    */
  CircularObstacle(const Eigen::Ref< const Eigen::Vector2d>& position, double radius) : Obstacle(), pos_(position), radius_(radius)
  {}

  /**
    * @brief Construct CircularObstacle using x- and y-center-coordinates and radius
    * @param x x-coordinate
    * @param y y-coordinate
    * @param radius radius of the obstacle
    */
  CircularObstacle(double x, double y, double radius) : Obstacle(), pos_(Eigen::Vector2d(x,y)), radius_(radius)
  {}


  // implements checkCollision() of the base class
  virtual bool checkCollision(const Eigen::Vector2d& point, double min_dist) const
  {
      return getMinimumDistance(point) < min_dist;
  }


  // implements checkLineIntersection() of the base class
  virtual bool checkLineIntersection(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double min_dist=0) const
  {
      // Distance Line - Circle
      // refer to http://www.spieleprogrammierer.de/wiki/2D-Kollisionserkennung#Kollision_Kreis-Strecke
      Eigen::Vector2d a = line_end-line_start; // not normalized!  a=y-x
      Eigen::Vector2d b = pos_-line_start; // b=m-x

      // Now find nearest point to circle v=x+a*t with t=a*b/(a*a) and bound to 0<=t<=1
      double t = a.dot(b)/a.dot(a);
      if (t<0) t=0; // bound t (since a is not normalized, t can be scaled between 0 and 1 to parametrize the line
      else if (t>1) t=1;
      Eigen::Vector2d nearest_point = line_start + a*t;

      // check collision
      return checkCollision(nearest_point, min_dist);
  }


  // implements getMinimumDistance() of the base class
  virtual double getMinimumDistance(const Eigen::Vector2d& position) const
  {
    return (position-pos_).norm() - radius_;
  }

  // implements getMinimumDistance() of the base class
  virtual double getMinimumDistance(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end) const
  {
    return distance_point_to_segment_2d(pos_, line_start, line_end) - radius_;
  }

  // implements getMinimumDistance() of the base class
  virtual double getMinimumDistance(const Point2dContainer& polygon) const
  {
    return distance_point_to_polygon_2d(pos_, polygon) - radius_;
  }

  // implements getMinimumDistanceVec() of the base class
  virtual Eigen::Vector2d getClosestPoint(const Eigen::Vector2d& position) const
  {
    return pos_ + radius_*(position-pos_).normalized();
  }

  // implements getMinimumSpatioTemporalDistance() of the base class
  virtual double getMinimumSpatioTemporalDistance(const Eigen::Vector2d& position, double t) const
  {
    return (pos_ + t*centroid_velocity_ - position).norm() - radius_;
  }

  // implements getMinimumSpatioTemporalDistance() of the base class
  virtual double getMinimumSpatioTemporalDistance(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double t) const
  {
    return distance_point_to_segment_2d(pos_ + t*centroid_velocity_, line_start, line_end) - radius_;
  }

  // implements getMinimumSpatioTemporalDistance() of the base class
  virtual double getMinimumSpatioTemporalDistance(const Point2dContainer& polygon, double t) const
  {
    return distance_point_to_polygon_2d(pos_ + t*centroid_velocity_, polygon) - radius_;
  }

  // implements predictCentroidConstantVelocity() of the base class
  virtual void predictCentroidConstantVelocity(double t, Eigen::Ref<Eigen::Vector2d> position) const
  {
    position = pos_ + t*centroid_velocity_;
  }

  // implements getCentroid() of the base class
  virtual const Eigen::Vector2d& getCentroid() const
  {
    return pos_;
  }

  // implements getCentroidCplx() of the base class
  virtual std::complex<double> getCentroidCplx() const
  {
    return std::complex<double>(pos_[0],pos_[1]);
  }

  // Accessor methods
  const Eigen::Vector2d& position() const {return pos_;} //!< Return the current position of the obstacle (read-only)
  Eigen::Vector2d& position() {return pos_;} //!< Return the current position of the obstacle
  double& x() {return pos_.coeffRef(0);} //!< Return the current x-coordinate of the obstacle
  const double& x() const {return pos_.coeffRef(0);} //!< Return the current y-coordinate of the obstacle (read-only)
  double& y() {return pos_.coeffRef(1);} //!< Return the current x-coordinate of the obstacle
  const double& y() const {return pos_.coeffRef(1);} //!< Return the current y-coordinate of the obstacle (read-only)
  double& radius() {return radius_;} //!< Return the current radius of the obstacle
  const double& radius() const {return radius_;} //!< Return the current radius of the obstacle

  // implements toPolygonMsg() of the base class
  virtual void toPolygonMsg(geometry_msgs::Polygon& polygon)
  {
    // TODO(roesmann): the polygon message type cannot describe a "perfect" circle
    //                 We could switch to ObstacleMsg if required somewhere...
    polygon.points.resize(1);
    polygon.points.front().x = pos_.x();
    polygon.points.front().y = pos_.y();
    polygon.points.front().z = 0;
  }

protected:

  Eigen::Vector2d pos_; //!< Store the center position of the CircularObstacle
  double radius_ = 0.0; //!< Radius of the obstacle


public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
* @class LineObstacle
* @brief Implements a 2D line obstacle
*/
  
class LineObstacle : public Obstacle
{
public:
  //! Abbrev. for a container storing vertices (2d points defining the edge points of the polygon)
  typedef std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > VertexContainer;
  
  /**
    * @brief Default constructor of the point obstacle class
    */
  LineObstacle() : Obstacle()
  {
    start_.setZero();
    end_.setZero();
    centroid_.setZero();
  }
  
  /**
   * @brief Construct LineObstacle using 2d position vectors as start and end of the line
   * @param line_start 2d position that defines the start of the line obstacle
   * @param line_end 2d position that defines the end of the line obstacle
   */
  LineObstacle(const Eigen::Ref< const Eigen::Vector2d>& line_start, const Eigen::Ref< const Eigen::Vector2d>& line_end) 
                : Obstacle(), start_(line_start), end_(line_end)
  {
    calcCentroid();
  }
  
  /**
   * @brief Construct LineObstacle using start and end coordinates
   * @param x1 x-coordinate of the start of the line
   * @param y1 y-coordinate of the start of the line
   * @param x2 x-coordinate of the end of the line
   * @param y2 y-coordinate of the end of the line
   */
  LineObstacle(double x1, double y1, double x2, double y2) : Obstacle()     
  {
    start_.x() = x1;
    start_.y() = y1;
    end_.x() = x2;
    end_.y() = y2;
    calcCentroid();
  }

  // implements checkCollision() of the base class
  virtual bool checkCollision(const Eigen::Vector2d& point, double min_dist) const    
  {
    return getMinimumDistance(point) <= min_dist;
  }
  
  // implements checkLineIntersection() of the base class
  virtual bool checkLineIntersection(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double min_dist=0) const 
  {
    return check_line_segments_intersection_2d(line_start, line_end, start_, end_);
  }

  // implements getMinimumDistance() of the base class
  virtual double getMinimumDistance(const Eigen::Vector2d& position) const 
  {
    return distance_point_to_segment_2d(position, start_, end_);
  }
  
  // implements getMinimumDistance() of the base class
  virtual double getMinimumDistance(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end) const
  {
    return distance_segment_to_segment_2d(start_, end_, line_start, line_end);
  }
  
  // implements getMinimumDistance() of the base class
  virtual double getMinimumDistance(const Point2dContainer& polygon) const
  {
    return distance_segment_to_polygon_2d(start_, end_, polygon);
  }

  // implements getMinimumDistanceVec() of the base class
  virtual Eigen::Vector2d getClosestPoint(const Eigen::Vector2d& position) const
  {
    return closest_point_on_line_segment_2d(position, start_, end_);
  }

  // implements getMinimumSpatioTemporalDistance() of the base class
  virtual double getMinimumSpatioTemporalDistance(const Eigen::Vector2d& position, double t) const
  {
    Eigen::Vector2d offset = t*centroid_velocity_;
    return distance_point_to_segment_2d(position, start_ + offset, end_ + offset);
  }

  // implements getMinimumSpatioTemporalDistance() of the base class
  virtual double getMinimumSpatioTemporalDistance(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double t) const
  {
    Eigen::Vector2d offset = t*centroid_velocity_;
    return distance_segment_to_segment_2d(start_ + offset, end_ + offset, line_start, line_end);
  }

  // implements getMinimumSpatioTemporalDistance() of the base class
  virtual double getMinimumSpatioTemporalDistance(const Point2dContainer& polygon, double t) const
  {
    Eigen::Vector2d offset = t*centroid_velocity_;
    return distance_segment_to_polygon_2d(start_ + offset, end_ + offset, polygon);
  }

  // implements getCentroid() of the base class
  virtual const Eigen::Vector2d& getCentroid() const    
  {
    return centroid_;
  }
  
  // implements getCentroidCplx() of the base class
  virtual std::complex<double> getCentroidCplx() const  
  {
    return std::complex<double>(centroid_.x(), centroid_.y());
  }
  
  // Access or modify line
  const Eigen::Vector2d& start() const {return start_;}
  void setStart(const Eigen::Ref<const Eigen::Vector2d>& start) {start_ = start; calcCentroid();}
  const Eigen::Vector2d& end() const {return end_;}
  void setEnd(const Eigen::Ref<const Eigen::Vector2d>& end) {end_ = end; calcCentroid();}
  
  // implements toPolygonMsg() of the base class
  virtual void toPolygonMsg(geometry_msgs::Polygon& polygon)
  {
    polygon.points.resize(2);
    polygon.points.front().x = start_.x();
    polygon.points.front().y = start_.y();
    
    polygon.points.back().x = end_.x();
    polygon.points.back().y = end_.y();
    polygon.points.back().z = polygon.points.front().z = 0;
  }
  
protected:
  void calcCentroid()	{	centroid_ = 0.5*(start_ + end_); }
  
private:
	Eigen::Vector2d start_;
	Eigen::Vector2d end_;
	
  Eigen::Vector2d centroid_;

public:	
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
};
  

/**
 * @class PolygonObstacle
 * @brief Implements a polygon obstacle with an arbitrary number of vertices
 * @details If the polygon has only 2 vertices, than it is considered as a line,
 * 	    otherwise the polygon will always be closed (a connection between the first and the last vertex
 * 	    is included automatically).
 */
class PolygonObstacle : public Obstacle
{
public:
    
  /**
    * @brief Default constructor of the polygon obstacle class
    */
  PolygonObstacle() : Obstacle(), finalized_(false)
  {
    centroid_.setConstant(NAN);
  }
  
  /**
   * @brief Construct polygon obstacle with a list of vertices
   */
  PolygonObstacle(const Point2dContainer& vertices) : Obstacle(), vertices_(vertices)
  {
    finalizePolygon();
  }
  
  
  /* FIXME Not working at the moment due to the aligned allocator version of std::vector
    * And it is C++11 code that is disabled atm to ensure compliance with ROS indigo/jade
  template <typename... Vector2dType>
  PolygonObstacle(const Vector2dType&... vertices) : _vertices({vertices...})
  { 
    calcCentroid();
    _finalized = true;
  }
  */

  
  // implements checkCollision() of the base class
  virtual bool checkCollision(const Eigen::Vector2d& point, double min_dist) const
  {
      // line case
      if (noVertices()==2)
        return getMinimumDistance(point) <= min_dist;
    
      // check if point is in the interior of the polygon
      // point in polygon test - raycasting (http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html)
      // using the following algorithm we may obtain false negatives on edge-cases, but that's ok for our purposes	
      int i, j;
      bool c = false;
      for (i = 0, j = noVertices()-1; i < noVertices(); j = i++) 
      {
        if ( ((vertices_.at(i).y()>point.y()) != (vertices_.at(j).y()>point.y())) &&
              (point.x() < (vertices_.at(j).x()-vertices_.at(i).x()) * (point.y()-vertices_.at(i).y()) / (vertices_.at(j).y()-vertices_.at(i).y()) + vertices_.at(i).x()) )
            c = !c;
      }
      if (c>0) return true;

      // If this statement is reached, the point lies outside the polygon or maybe on its edges
      // Let us check the minium distance as well
      return min_dist == 0 ? false : getMinimumDistance(point) < min_dist;
  }
  

  /**
    * @brief Check if a given line segment between two points intersects with the obstacle (and additionally keeps a safty distance \c min_dist)
    * @param line_start 2D point for the end of the reference line
    * @param line_end 2D point for the end of the reference line
    * @param min_dist Minimum distance allowed to the obstacle to be collision/intersection free
    * @remarks we ignore \c min_dist here
    * @return \c true if given line intersects the region of the obstacle or if the minimum distance is lower than min_dist
    */
  virtual bool checkLineIntersection(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double min_dist=0) const;


  // implements getMinimumDistance() of the base class
  virtual double getMinimumDistance(const Eigen::Vector2d& position) const
  {
    return distance_point_to_polygon_2d(position, vertices_);
  }
  
  // implements getMinimumDistance() of the base class
  virtual double getMinimumDistance(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end) const
  {
    return distance_segment_to_polygon_2d(line_start, line_end, vertices_);
  }

  // implements getMinimumDistance() of the base class
  virtual double getMinimumDistance(const Point2dContainer& polygon) const
  {
    return distance_polygon_to_polygon_2d(polygon, vertices_);
  }
  
  // implements getMinimumDistanceVec() of the base class
  virtual Eigen::Vector2d getClosestPoint(const Eigen::Vector2d& position) const;
  
  // implements getMinimumSpatioTemporalDistance() of the base class
  virtual double getMinimumSpatioTemporalDistance(const Eigen::Vector2d& position, double t) const
  {
    Point2dContainer pred_vertices;
    predictVertices(t, pred_vertices);
    return distance_point_to_polygon_2d(position, pred_vertices);
  }

  // implements getMinimumSpatioTemporalDistance() of the base class
  virtual double getMinimumSpatioTemporalDistance(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double t) const
  {
    Point2dContainer pred_vertices;
    predictVertices(t, pred_vertices);
    return distance_segment_to_polygon_2d(line_start, line_end, pred_vertices);
  }

  // implements getMinimumSpatioTemporalDistance() of the base class
  virtual double getMinimumSpatioTemporalDistance(const Point2dContainer& polygon, double t) const
  {
    Point2dContainer pred_vertices;
    predictVertices(t, pred_vertices);
    return distance_polygon_to_polygon_2d(polygon, pred_vertices);
  }

  virtual void predictVertices(double t, Point2dContainer& pred_vertices) const
  {
    // Predict obstacle (polygon) at time t
    pred_vertices.resize(vertices_.size());
    Eigen::Vector2d offset = t*centroid_velocity_;
    for (std::size_t i = 0; i < vertices_.size(); i++)
    {
      pred_vertices[i] = vertices_[i] + offset;
    }
  }

  // implements getCentroid() of the base class
  virtual const Eigen::Vector2d& getCentroid() const
  {
    assert(finalized_ && "Finalize the polygon after all vertices are added.");
    return centroid_;
  }
  
  // implements getCentroidCplx() of the base class
  virtual std::complex<double> getCentroidCplx() const
  {
    assert(finalized_ && "Finalize the polygon after all vertices are added.");
    return std::complex<double>(centroid_.coeffRef(0), centroid_.coeffRef(1));
  }
  
  // implements toPolygonMsg() of the base class
  virtual void toPolygonMsg(geometry_msgs::Polygon& polygon);

  
  /** @name Define the polygon */
  ///@{
  
  // Access or modify polygon
  const Point2dContainer& vertices() const {return vertices_;} //!< Access vertices container (read-only)
  Point2dContainer& vertices() {return vertices_;} //!< Access vertices container
  
  /**
    * @brief Add a vertex to the polygon (edge-point)
    * @remarks You do not need to close the polygon (do not repeat the first vertex)
    * @warning Do not forget to call finalizePolygon() after adding all vertices
    * @param vertex 2D point defining a new polygon edge
    */
  void pushBackVertex(const Eigen::Ref<const Eigen::Vector2d>& vertex)
  {
    vertices_.push_back(vertex);
    finalized_ = false;
  }
  
  /**
    * @brief Add a vertex to the polygon (edge-point)
    * @remarks You do not need to close the polygon (do not repeat the first vertex)
    * @warning Do not forget to call finalizePolygon() after adding all vertices
    * @param x x-coordinate of the new vertex
    * @param y y-coordinate of the new vertex
    */  
  void pushBackVertex(double x, double y)
  {
    vertices_.push_back(Eigen::Vector2d(x,y));
    finalized_ = false;
  }
  
  /**
    * @brief Call finalizePolygon after the polygon is created with the help of pushBackVertex() methods
    */
  void finalizePolygon()
  {
    fixPolygonClosure();
    calcCentroid();
    finalized_ = true;
  }
  
  /**
    * @brief Clear all vertices (Afterwards the polygon is not valid anymore)
    */
  void clearVertices() {vertices_.clear(); finalized_ = false;}
  
  /**
    * @brief Get the number of vertices defining the polygon (the first vertex is counted once)
    */
  int noVertices() const {return (int)vertices_.size();}
  
  
  ///@}
      
protected:
  
  void fixPolygonClosure(); //!< Check if the current polygon contains the first vertex twice (as start and end) and in that case erase the last redundant one.

  void calcCentroid(); //!< Compute the centroid of the polygon (called inside finalizePolygon())

  
  Point2dContainer vertices_; //!< Store vertices defining the polygon (@see pushBackVertex)
  Eigen::Vector2d centroid_; //!< Store the centroid coordinates of the polygon (@see calcCentroid)
  
  bool finalized_; //!< Flat that keeps track if the polygon was finalized after adding all vertices
  
  	
public:	
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
};


} // namespace teb_local_planner

#endif /* OBSTACLES_H */
