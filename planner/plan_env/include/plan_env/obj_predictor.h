/**
 * This file is part of Fast-Planner.
 *
 * Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
 * Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
 * for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * Fast-Planner is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Fast-Planner is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef OBJ_PREDICTOR_H_
#define OBJ_PREDICTOR_H_

#include <Eigen/Eigen>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <list>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

using std::cout;
using std::endl;
using std::list;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace fast_planner
{
  class PolynomialPrediction;
  typedef shared_ptr<vector<PolynomialPrediction>> ObjPrediction;
  typedef shared_ptr<vector<Eigen::Vector3d>> ObjScale;

  /* ========== prediction polynomial ========== */
  class PolynomialPrediction
  {
  private:
    vector<Eigen::Matrix<double, 6, 1>> polys;
    double t1, t2; // start / end
    rclcpp::Time global_start_time_;

  public:
    PolynomialPrediction(/* args */)
    {
    }
    ~PolynomialPrediction()
    {
    }

    void setPolynomial(vector<Eigen::Matrix<double, 6, 1>> &pls)
    {
      polys = pls;
    }
    void setTime(double t1, double t2)
    {
      this->t1 = t1;
      this->t2 = t2;
    }
    void setGlobalStartTime(rclcpp::Time global_start_time)
    {
      global_start_time_ = global_start_time;
    }

    bool valid()
    {
      return polys.size() == 3;
    }

    /* note that t should be in [t1, t2] */
    Eigen::Vector3d evaluate(double t)
    {
      Eigen::Matrix<double, 6, 1> tv;
      tv << 1.0, pow(t, 1), pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5);

      Eigen::Vector3d pt;
      pt(0) = tv.dot(polys[0]), pt(1) = tv.dot(polys[1]), pt(2) = tv.dot(polys[2]);

      return pt;
    }

    Eigen::Vector3d evaluateConstVel(double t)
    {
      Eigen::Matrix<double, 2, 1> tv;
      tv << 1.0, pow(t - global_start_time_.seconds(), 1);

      // cout << t-global_start_time_.toSec() << endl;

      Eigen::Vector3d pt;
      pt(0) = tv.dot(polys[0].head(2)), pt(1) = tv.dot(polys[1].head(2)), pt(2) = tv.dot(polys[2].head(2));

      return pt;
    }
  };

  /* ========== subscribe and record object history ========== */
  class ObjHistory
  {
  public:
    int skip_num_;
    int queue_size_;
    rclcpp::Time global_start_time_;

    ObjHistory()
    {
    }
    ~ObjHistory()
    {
    }

    void init(int id, int skip_num, int queue_size, rclcpp::Time global_start_time);

    void poseCallback(const geometry_msgs::msg::PoseStamped::ConstPtr &msg);

    void clear()
    {
      history_.clear();
    }

    void getHistory(list<Eigen::Vector4d> &his)
    {
      his = history_;
    }

  private:
    list<Eigen::Vector4d> history_; // x,y,z;t
    int skip_;
    int obj_idx_;
    Eigen::Vector3d scale_;
  };

  /* ========== predict future trajectory using history ========== */
  class ObjPredictor : public rclcpp::Node
  {
  private:
    int obj_num_;
    double lambda_;
    double predict_rate_;

    vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_subs_;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr marker_sub_;
    rclcpp::TimerBase::SharedPtr predict_timer_;
    vector<std::shared_ptr<ObjHistory>> obj_histories_;

    /* share data with planner */
    ObjPrediction predict_trajs_;
    ObjScale obj_scale_;
    vector<bool> scale_init_;

    void markerCallback(const visualization_msgs::msg::Marker::ConstPtr &msg);

    void predictCallback(); // ROS2 的定时器回调不需要 TimerEvent
    void predictPolyFit();
    void predictConstVel();

  public:
    ObjPredictor(const std::string &node_name = "obj_predictor")
        : Node(node_name) {}

    ~ObjPredictor() {}

    void init();

    ObjPrediction getPredictionTraj();
    ObjScale getObjScale();
    int getObjNums() { return obj_num_; }

    Eigen::Vector3d evaluatePoly(int obs_id, double time);
    Eigen::Vector3d evaluateConstVel(int obs_id, double time);

    typedef std::shared_ptr<ObjPredictor> Ptr;
  };

} // namespace fast_planner

#endif