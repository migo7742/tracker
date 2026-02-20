#pragma once

#include <mapping/mapping.h>
// ROS 2 Header
#include <rclcpp/rclcpp.hpp>

#include <Eigen/Core>
#include <queue>
#include <vector>
#include <iostream>
#include <algorithm> // for std::reverse

namespace prediction {

struct Node {
  Eigen::Vector3d p, v, a;
  double t;
  double score;
  double h;
  Node* parent = nullptr;
};
typedef Node* NodePtr;
class NodeComparator {
 public:
  bool operator()(NodePtr& lhs, NodePtr& rhs) {
    return lhs->score + lhs->h > rhs->score + rhs->h;
  }
};

class Predict {
 private:
  static constexpr int MAX_MEMORY = 1 << 22;
  
  rclcpp::Node::SharedPtr nh_;

  double dt;
  double pre_dur;
  double rho_a;
  double car_z, vmax;
  mapping::OccGridMap map;
  std::unique_ptr<Node[]> pool_;
  NodePtr data[MAX_MEMORY];
  int stack_top;

  inline bool isValid(const Eigen::Vector3d& p, const Eigen::Vector3d& v) const {
    return (v.norm() < vmax) && (!map.isOccupied(p));
  }

 public:
  inline Predict(rclcpp::Node::SharedPtr nh) : nh_(nh) {
    auto declare_and_get = [&](const std::string& name, double& var, double default_val) {
        if (!nh_->has_parameter(name)) {
            nh_->declare_parameter(name, default_val);
        }
        nh_->get_parameter(name, var);
    };

    declare_and_get("tracking_dur", pre_dur, 1.0);
    declare_and_get("tracking_dt", dt, 0.1);
    declare_and_get("prediction/rho_a", rho_a, 1.0);
    declare_and_get("prediction/vmax", vmax, 2.0);

    pool_ = std::make_unique<Node[]>(MAX_MEMORY);
    for (int i = 0; i < MAX_MEMORY; ++i) {
      data[i] = &pool_[i];
    }
  }

  inline void setMap(const mapping::OccGridMap& _map) {
    map = _map;
    // map.inflate_last();
  }

  inline bool predict(const Eigen::Vector3d& target_p,
                      const Eigen::Vector3d& target_v,
                      std::vector<Eigen::Vector3d>& target_predcit,
                      const double& max_time = 0.1) {
    auto score = [&](const NodePtr& ptr) -> double {
      return rho_a * ptr->a.norm();
    };
    Eigen::Vector3d end_p = target_p + target_v * pre_dur;
    auto calH = [&](const NodePtr& ptr) -> double {
      return 0.001 * (ptr->p - end_p).norm();
    };

    // ROS 2 Time API
    rclcpp::Time t_start = nh_->get_clock()->now();
    std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> open_set;

    Eigen::Vector3d input(0, 0, 0);

    stack_top = 0;
    NodePtr curPtr = data[stack_top++];
    curPtr->p = target_p;
    curPtr->v = target_v;
    curPtr->a.setZero();
    curPtr->parent = nullptr;
    curPtr->score = 0;
    curPtr->h = 0;
    curPtr->t = 0;
    double dt2_2 = dt * dt / 2;
    while (curPtr->t < pre_dur) {
      for (input.x() = -3; input.x() <= 3; input.x() += 3)
        for (input.y() = -3; input.y() <= 3; input.y() += 3) {
          Eigen::Vector3d p = curPtr->p + curPtr->v * dt + input * dt2_2;
          Eigen::Vector3d v = curPtr->v + input * dt;
          if (!isValid(p, v)) {
            continue;
          }
          if (stack_top == MAX_MEMORY) {
            RCLCPP_ERROR(nh_->get_logger(), "[prediction] out of memory!");
            return false;
          }
          
          // Updated time difference calculation
          double t_cost = (nh_->get_clock()->now() - t_start).seconds();
          if (t_cost > max_time) {
            RCLCPP_WARN(nh_->get_logger(), "[prediction] too slow!");
            return false;
          }
          NodePtr ptr = data[stack_top++];
          ptr->p = p;
          ptr->v = v;
          ptr->a = input;
          ptr->parent = curPtr;
          ptr->t = curPtr->t + dt;
          ptr->score = curPtr->score + score(ptr);
          ptr->h = calH(ptr);
          open_set.push(ptr);
          // std::cout << "open set push: " << state.transpose() << std::endl;
        }
      if (open_set.empty()) {
        RCLCPP_WARN(nh_->get_logger(), "[prediction] no way!");
        return false;
      }
      curPtr = open_set.top();
      open_set.pop();
    }
    target_predcit.clear();
    while (curPtr != nullptr) {
      target_predcit.push_back(curPtr->p);
      curPtr = curPtr->parent;
    }
    std::reverse(target_predcit.begin(), target_predcit.end());
    return true;
  }
};

}  // namespace prediction
