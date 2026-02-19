#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mapping/mapping.h>
#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/occ_map3d.hpp>
#include <quadrotor_msgs/msg/poly_traj.hpp>
#include <quadrotor_msgs/msg/replan_state.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <traj_opt/traj_opt.h>
#include <rclcpp_components/register_node_macro.hpp>

#include <Eigen/Core>
#include <atomic>
#include <env/env.hpp>
#include <prediction/prediction.hpp>
#include <thread>
#include <visualization/visualization.hpp>
#include <wr_msg/wr_msg.hpp>
#include <chrono>

using namespace std::chrono_literals;

namespace planning {

Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");

class PlanningNode : public rclcpp::Node {
 private:
  // ROS 2 Initialization handling
  rclcpp::TimerBase::SharedPtr init_timer_; 
  
  rclcpp::Subscription<quadrotor_msgs::msg::OccMap3d>::SharedPtr gridmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr target_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr triger_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr land_triger_sub_;
  
  rclcpp::TimerBase::SharedPtr plan_timer_;

  rclcpp::Publisher<quadrotor_msgs::msg::PolyTraj>::SharedPtr traj_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat_pub_;
  rclcpp::Publisher<quadrotor_msgs::msg::ReplanState>::SharedPtr replanState_pub_;

  std::shared_ptr<mapping::OccGridMap> gridmapPtr_;
  std::shared_ptr<env::Env> envPtr_;
  std::shared_ptr<visualization::Visualization> visPtr_;
  std::shared_ptr<traj_opt::TrajOpt> trajOptPtr_;
  std::shared_ptr<prediction::Predict> prePtr_;

  // NOTE planning or fake target
  bool fake_ = false;
  Eigen::Vector3d goal_;
  Eigen::Vector3d land_p_;
  Eigen::Quaterniond land_q_;

  // NOTE just for debug
  bool debug_ = false;
  quadrotor_msgs::msg::ReplanState replanStateMsg_;
  rclcpp::Publisher<quadrotor_msgs::msg::OccMap3d>::SharedPtr gridmap_pub_;
  rclcpp::Publisher<quadrotor_msgs::msg::OccMap3d>::SharedPtr inflate_gridmap_pub_;
  quadrotor_msgs::msg::OccMap3d occmap_msg_;

  double tracking_dur_, tracking_dist_, tolerance_d_;

  Trajectory traj_poly_;
  rclcpp::Time replan_stamp_;
  int traj_id_ = 0;
  bool wait_hover_ = true;
  bool force_hover_ = true;

  nav_msgs::msg::Odometry odom_msg_, target_msg_;
  quadrotor_msgs::msg::OccMap3d map_msg_;
  std::atomic_flag odom_lock_ = ATOMIC_FLAG_INIT;
  std::atomic_flag target_lock_ = ATOMIC_FLAG_INIT;
  std::atomic_flag gridmap_lock_ = ATOMIC_FLAG_INIT;
  std::atomic_bool odom_received_ = ATOMIC_VAR_INIT(false);
  std::atomic_bool map_received_ = ATOMIC_VAR_INIT(false);
  std::atomic_bool triger_received_ = ATOMIC_VAR_INIT(false);
  std::atomic_bool target_received_ = ATOMIC_VAR_INIT(false);
  std::atomic_bool land_triger_received_ = ATOMIC_VAR_INIT(false);

  void pub_hover_p(const Eigen::Vector3d& hover_p, const rclcpp::Time& stamp) {
    quadrotor_msgs::msg::PolyTraj traj_msg;
    traj_msg.hover = true;
    traj_msg.hover_p.resize(3);
    for (int i = 0; i < 3; ++i) {
      traj_msg.hover_p[i] = hover_p[i];
    }
    traj_msg.start_time = stamp;
    traj_msg.traj_id = traj_id_++;
    traj_pub_->publish(traj_msg);
  }
  
  void pub_traj(const Trajectory& traj, const double& yaw, const rclcpp::Time& stamp) {
    quadrotor_msgs::msg::PolyTraj traj_msg;
    traj_msg.hover = false;
    traj_msg.order = 5;
    Eigen::VectorXd durs = traj.getDurations();
    int piece_num = traj.getPieceNum();
    traj_msg.duration.resize(piece_num);
    traj_msg.coef_x.resize(6 * piece_num);
    traj_msg.coef_y.resize(6 * piece_num);
    traj_msg.coef_z.resize(6 * piece_num);
    for (int i = 0; i < piece_num; ++i) {
      traj_msg.duration[i] = durs(i);
      CoefficientMat cMat = traj[i].getCoeffMat();
      int i6 = i * 6;
      for (int j = 0; j < 6; j++) {
        traj_msg.coef_x[i6 + j] = cMat(0, j);
        traj_msg.coef_y[i6 + j] = cMat(1, j);
        traj_msg.coef_z[i6 + j] = cMat(2, j);
      }
    }
    traj_msg.start_time = stamp;
    traj_msg.traj_id = traj_id_++;
    // NOTE yaw
    traj_msg.yaw = yaw;
    traj_pub_->publish(traj_msg);
  }

  void triger_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msgPtr) {
    goal_ << msgPtr->pose.position.x, msgPtr->pose.position.y, 0.9;
    triger_received_ = true;
  }

  void land_triger_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msgPtr) {
    land_p_.x() = msgPtr->pose.position.x;
    land_p_.y() = msgPtr->pose.position.y;
    land_p_.z() = msgPtr->pose.position.z;
    land_q_.w() = msgPtr->pose.orientation.w;
    land_q_.x() = msgPtr->pose.orientation.x;
    land_q_.y() = msgPtr->pose.orientation.y;
    land_q_.z() = msgPtr->pose.orientation.z;
    land_triger_received_ = true;
  }

  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msgPtr) {
    while (odom_lock_.test_and_set())
      ;
    odom_msg_ = *msgPtr;
    odom_received_ = true;
    odom_lock_.clear();
  }

  void target_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msgPtr) {
    while (target_lock_.test_and_set())
      ;
    target_msg_ = *msgPtr;
    target_received_ = true;
    target_lock_.clear();
  }

  void gridmap_callback(const quadrotor_msgs::msg::OccMap3d::ConstSharedPtr msgPtr) {
    while (gridmap_lock_.test_and_set())
      ;
    map_msg_ = *msgPtr;
    map_received_ = true;
    gridmap_lock_.clear();
  }

  // NOTE main callback
  void plan_timer_callback() {
    heartbeat_pub_->publish(std_msgs::msg::Empty());
    if (!odom_received_ || !map_received_) {
      return;
    }
    // obtain state of odom
    while (odom_lock_.test_and_set())
      ;
    auto odom_msg = odom_msg_;
    odom_lock_.clear();
    Eigen::Vector3d odom_p(odom_msg.pose.pose.position.x,
                           odom_msg.pose.pose.position.y,
                           odom_msg.pose.pose.position.z);
    Eigen::Vector3d odom_v(odom_msg.twist.twist.linear.x,
                           odom_msg.twist.twist.linear.y,
                           odom_msg.twist.twist.linear.z);
    Eigen::Quaterniond odom_q(odom_msg.pose.pose.orientation.w,
                              odom_msg.pose.pose.orientation.x,
                              odom_msg.pose.pose.orientation.y,
                              odom_msg.pose.pose.orientation.z);
    if (!triger_received_) {
      return;
    }
    if (!target_received_) {
      return;
    }
    // NOTE obtain state of target
    while (target_lock_.test_and_set())
      ;
    replanStateMsg_.target = target_msg_;
    target_lock_.clear();
    Eigen::Vector3d target_p(replanStateMsg_.target.pose.pose.position.x,
                             replanStateMsg_.target.pose.pose.position.y,
                             replanStateMsg_.target.pose.pose.position.z);
    Eigen::Vector3d target_v(replanStateMsg_.target.twist.twist.linear.x,
                             replanStateMsg_.target.twist.twist.linear.y,
                             replanStateMsg_.target.twist.twist.linear.z);
    Eigen::Quaterniond target_q;
    target_q.w() = replanStateMsg_.target.pose.pose.orientation.w;
    target_q.x() = replanStateMsg_.target.pose.pose.orientation.x;
    target_q.y() = replanStateMsg_.target.pose.pose.orientation.y;
    target_q.z() = replanStateMsg_.target.pose.pose.orientation.z;

    // NOTE force-hover: waiting for the speed of drone small enough
    if (force_hover_ && odom_v.norm() > 0.1) {
      return;
    }

    // NOTE just for landing on the car!
    if (land_triger_received_) {
      if (std::fabs((target_p - odom_p).norm() < 0.1 && odom_v.norm() < 0.1 && target_v.norm() < 0.2)) {
        if (!wait_hover_) {
          pub_hover_p(odom_p, this->now());
          wait_hover_ = true;
        }
        RCLCPP_WARN(this->get_logger(), "[planner] HOVERING...");
        return;
      }
      // TODO get the orientation fo target and calculate the pose of landing point
      target_p = target_p + target_q * land_p_;
      wait_hover_ = false;
    } else {
      target_p.z() += 1.0;
      // NOTE determin whether to replan
      Eigen::Vector3d dp = target_p - odom_p;
      // std::cout << "dist : " << dp.norm() << std::endl;
      double desired_yaw = std::atan2(dp.y(), dp.x());
      Eigen::Vector3d project_yaw = odom_q.toRotationMatrix().col(0);  // NOTE ZYX
      double now_yaw = std::atan2(project_yaw.y(), project_yaw.x());
      if (std::fabs((target_p - odom_p).norm() - tracking_dist_) < tolerance_d_ &&
          odom_v.norm() < 0.1 && target_v.norm() < 0.2 &&
          std::fabs(desired_yaw - now_yaw) < 0.5) {
        if (!wait_hover_) {
          pub_hover_p(odom_p, this->now());
          wait_hover_ = true;
        }
        RCLCPP_WARN(this->get_logger(), "[planner] HOVERING...");
        replanStateMsg_.state = -1;
        replanState_pub_->publish(replanStateMsg_);
        return;
      } else {
        wait_hover_ = false;
      }
    }

    // NOTE obtain map
    while (gridmap_lock_.test_and_set())
      ;
    gridmapPtr_->from_msg(map_msg_);
    replanStateMsg_.occmap = map_msg_;
    gridmap_lock_.clear();
    prePtr_->setMap(*gridmapPtr_);

    // visualize the ray from drone to target
    if (envPtr_->checkRayValid(odom_p, target_p)) {
      visPtr_->visualize_arrow(odom_p, target_p, "ray", visualization::yellow);
    } else {
      visPtr_->visualize_arrow(odom_p, target_p, "ray", visualization::red);
    }

    // NOTE prediction
    std::vector<Eigen::Vector3d> target_predcit;
    // ros::Time t_start = ros::Time::now();
    bool generate_new_traj_success = prePtr_->predict(target_p, target_v, target_predcit);
    // ros::Time t_stop = ros::Time::now();
    // std::cout << "predict costs: " << (t_stop - t_start).toSec() * 1e3 << "ms" << std::endl;
    if (generate_new_traj_success) {
      Eigen::Vector3d observable_p = target_predcit.back();
      visPtr_->visualize_path(target_predcit, "car_predict");
      std::vector<Eigen::Vector3d> observable_margin;
      for (double theta = 0; theta <= 2 * M_PI; theta += 0.01) {
        observable_margin.emplace_back(observable_p + tracking_dist_ * Eigen::Vector3d(cos(theta), sin(theta), 0));
      }
      visPtr_->visualize_path(observable_margin, "observable_margin");
    }

    // NOTE replan state
    Eigen::MatrixXd iniState;
    iniState.setZero(3, 3);
    rclcpp::Time replan_stamp = this->now() + rclcpp::Duration::from_seconds(0.03);
    double replan_t = (replan_stamp - replan_stamp_).seconds();
    if (force_hover_ || replan_t > traj_poly_.getTotalDuration()) {
      // should replan from the hover state
      iniState.col(0) = odom_p;
      iniState.col(1) = odom_v;
    } else {
      // should replan from the last trajectory
      iniState.col(0) = traj_poly_.getPos(replan_t);
      iniState.col(1) = traj_poly_.getVel(replan_t);
      iniState.col(2) = traj_poly_.getAcc(replan_t);
    }
    replanStateMsg_.header.stamp = this->now();
    replanStateMsg_.iniState.resize(9);
    Eigen::Map<Eigen::MatrixXd>(replanStateMsg_.iniState.data(), 3, 3) = iniState;

    // NOTE path searching
    Eigen::Vector3d p_start = iniState.col(0);
    std::vector<Eigen::Vector3d> path, way_pts;

    if (generate_new_traj_success) {
      if (land_triger_received_) {
        generate_new_traj_success = envPtr_->short_astar(p_start, target_p, path);
      } else {
        generate_new_traj_success = envPtr_->findVisiblePath(p_start, target_predcit, way_pts, path);
      }
    }

    std::vector<Eigen::Vector3d> visible_ps;
    std::vector<double> thetas;
    Trajectory traj;
    if (generate_new_traj_success) {
      visPtr_->visualize_path(path, "astar");
      if (land_triger_received_) {
        for (const auto& p : target_predcit) {
          path.push_back(p);
        }
      } else {
        // NOTE generate visible regions
        target_predcit.pop_back();
        way_pts.pop_back();
        envPtr_->generate_visible_regions(target_predcit, way_pts,
                                          visible_ps, thetas);
        visPtr_->visualize_pointcloud(visible_ps, "visible_ps");
        visPtr_->visualize_fan_shape_meshes(target_predcit, visible_ps, thetas, "visible_region");

        // TODO change the final state
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> rays;
        for (int i = 0; i < (int)way_pts.size(); ++i) {
          rays.emplace_back(target_predcit[i], way_pts[i]);
        }
        visPtr_->visualize_pointcloud(way_pts, "way_pts");
        way_pts.insert(way_pts.begin(), p_start);
        envPtr_->pts2path(way_pts, path);
      }
      // NOTE corridor generating
      std::vector<Eigen::MatrixXd> hPolys;
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> keyPts;

      envPtr_->generateSFC(path, 2.0, hPolys, keyPts);

      envPtr_->visCorridor(hPolys);
      visPtr_->visualize_pairline(keyPts, "keyPts");

      // NOTE trajectory optimization
      Eigen::MatrixXd finState;
      finState.setZero(3, 3);
      finState.col(0) = path.back();
      finState.col(1) = target_v;
      if (land_triger_received_) {
        finState.col(0) = target_predcit.back();
        generate_new_traj_success = trajOptPtr_->generate_traj(iniState, finState, target_predcit, hPolys, traj);
      } else {
        generate_new_traj_success = trajOptPtr_->generate_traj(iniState, finState,
                                                               target_predcit, visible_ps, thetas,
                                                               hPolys, traj);
      }

      visPtr_->visualize_traj(traj, "traj");
    }

    // NOTE collision check
    bool valid = false;
    if (generate_new_traj_success) {
      valid = validcheck(traj, replan_stamp);
    } else {
      replanStateMsg_.state = -2;
      replanState_pub_->publish(replanStateMsg_);
    }
    if (valid) {
      force_hover_ = false;
      RCLCPP_WARN(this->get_logger(), "[planner] REPLAN SUCCESS");
      replanStateMsg_.state = 0;
      replanState_pub_->publish(replanStateMsg_);
      Eigen::Vector3d dp = target_p + target_v * 0.03 - iniState.col(0);
      double yaw = std::atan2(dp.y(), dp.x());
      if (land_triger_received_) {
        yaw = 2 * std::atan2(target_q.z(), target_q.w());
      }
      pub_traj(traj, yaw, replan_stamp);
      traj_poly_ = traj;
      replan_stamp_ = replan_stamp;
    } else if (force_hover_) {
      RCLCPP_ERROR(this->get_logger(), "[planner] REPLAN FAILED, HOVERING...");
      replanStateMsg_.state = 1;
      replanState_pub_->publish(replanStateMsg_);
      return;
    } else if (validcheck(traj_poly_, replan_stamp_)) {
      force_hover_ = true;
      RCLCPP_FATAL(this->get_logger(), "[planner] EMERGENCY STOP!!!");
      replanStateMsg_.state = 2;
      replanState_pub_->publish(replanStateMsg_);
      pub_hover_p(iniState.col(0), replan_stamp);
      return;
    } else {
      RCLCPP_ERROR(this->get_logger(), "[planner] REPLAN FAILED, EXECUTE LAST TRAJ...");
      replanStateMsg_.state = 3;
      replanState_pub_->publish(replanStateMsg_);
      return;  // current generated traj invalid but last is valid
    }
    visPtr_->visualize_traj(traj, "traj");
  }

  void fake_timer_callback() {
    heartbeat_pub_->publish(std_msgs::msg::Empty());
    if (!odom_received_ || !map_received_) {
      return;
    }
    // obtain state of odom
    while (odom_lock_.test_and_set())
      ;
    auto odom_msg = odom_msg_;
    odom_lock_.clear();
    Eigen::Vector3d odom_p(odom_msg.pose.pose.position.x,
                           odom_msg.pose.pose.position.y,
                           odom_msg.pose.pose.position.z);
    Eigen::Vector3d odom_v(odom_msg.twist.twist.linear.x,
                           odom_msg.twist.twist.linear.y,
                           odom_msg.twist.twist.linear.z);
    if (!triger_received_) {
      return;
    }
    // NOTE force-hover: waiting for the speed of drone small enough
    if (force_hover_ && odom_v.norm() > 0.1) {
      return;
    }

    // NOTE local goal
    Eigen::Vector3d local_goal;
    Eigen::Vector3d delta = goal_ - odom_p;
    if (delta.norm() < 15) {
      local_goal = goal_;
    } else {
      local_goal = delta.normalized() * 15 + odom_p;
    }

    // NOTE obtain map
    while (gridmap_lock_.test_and_set())
      ;
    gridmapPtr_->from_msg(map_msg_);
    replanStateMsg_.occmap = map_msg_;
    gridmap_lock_.clear();

    // NOTE determin whether to replan
    bool no_need_replan = false;
    if (!force_hover_ && !wait_hover_) {
      double last_traj_t_rest = traj_poly_.getTotalDuration() - (this->now() - replan_stamp_).seconds();
      bool new_goal = (local_goal - traj_poly_.getPos(traj_poly_.getTotalDuration())).norm() > tracking_dist_;
      if (!new_goal) {
        if (last_traj_t_rest < 1.0) {
          RCLCPP_WARN(this->get_logger(), "[planner] NEAR GOAL...");
          no_need_replan = true;
        } else if (validcheck(traj_poly_, replan_stamp_, last_traj_t_rest)) {
          RCLCPP_WARN(this->get_logger(), "[planner] NO NEED REPLAN...");
          double t_delta = traj_poly_.getTotalDuration() < 1.0 ? traj_poly_.getTotalDuration() : 1.0;
          double t_yaw = (this->now() - replan_stamp_).seconds() + t_delta;
          Eigen::Vector3d un_known_p = traj_poly_.getPos(t_yaw);
          Eigen::Vector3d dp = un_known_p - odom_p;
          double yaw = std::atan2(dp.y(), dp.x());
          pub_traj(traj_poly_, yaw, replan_stamp_);
          no_need_replan = true;
        }
      }
    }
    // NOTE determin whether to pub hover
    if ((goal_ - odom_p).norm() < tracking_dist_ + tolerance_d_ && odom_v.norm() < 0.1) {
      if (!wait_hover_) {
        pub_hover_p(odom_p, this->now());
        wait_hover_ = true;
      }
      RCLCPP_WARN(this->get_logger(), "[planner] HOVERING...");
      replanStateMsg_.state = -1;
      replanState_pub_->publish(replanStateMsg_);
      return;
    } else {
      wait_hover_ = false;
    }
    if (no_need_replan) {
      return;
    }

    // NOTE replan state
    Eigen::MatrixXd iniState;
    iniState.setZero(3, 3);
    rclcpp::Time replan_stamp = this->now() + rclcpp::Duration::from_seconds(0.03);
    double replan_t = (replan_stamp - replan_stamp_).seconds();
    if (force_hover_ || replan_t > traj_poly_.getTotalDuration()) {
      // should replan from the hover state
      iniState.col(0) = odom_p;
      iniState.col(1) = odom_v;
    } else {
      // should replan from the last trajectory
      iniState.col(0) = traj_poly_.getPos(replan_t);
      iniState.col(1) = traj_poly_.getVel(replan_t);
      iniState.col(2) = traj_poly_.getAcc(replan_t);
    }
    replanStateMsg_.header.stamp = this->now();
    replanStateMsg_.iniState.resize(9);
    Eigen::Map<Eigen::MatrixXd>(replanStateMsg_.iniState.data(), 3, 3) = iniState;

    // NOTE generate an extra corridor
    Eigen::Vector3d p_start = iniState.col(0);
    bool need_extra_corridor = iniState.col(1).norm() > 1.0;
    Eigen::MatrixXd hPoly;
    std::pair<Eigen::Vector3d, Eigen::Vector3d> line;
    if (need_extra_corridor) {
      Eigen::Vector3d v_norm = iniState.col(1).normalized();
      line.first = p_start;
      double step = 0.1;
      for (double dx = step; dx < 1.0; dx += step) {
        p_start += step * v_norm;
        if (gridmapPtr_->isOccupied(p_start)) {
          p_start -= step * v_norm;
          break;
        }
      }
      line.second = p_start;
      envPtr_->generateOneCorridor(line, 2.0, hPoly);
    }
    // NOTE path searching
    std::vector<Eigen::Vector3d> path;
    bool generate_new_traj_success = envPtr_->astar_search(p_start, local_goal, path);
    Trajectory traj;
    if (generate_new_traj_success) {
      visPtr_->visualize_path(path, "astar");
      // NOTE corridor generating
      std::vector<Eigen::MatrixXd> hPolys;
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> keyPts;
      envPtr_->generateSFC(path, 2.0, hPolys, keyPts);
      if (need_extra_corridor) {
        hPolys.insert(hPolys.begin(), hPoly);
        keyPts.insert(keyPts.begin(), line);
      }
      envPtr_->visCorridor(hPolys);
      visPtr_->visualize_pairline(keyPts, "keyPts");

      // NOTE trajectory optimization
      Eigen::MatrixXd finState;
      finState.setZero(3, 3);
      finState.col(0) = path.back();
      // return;
      generate_new_traj_success = trajOptPtr_->generate_traj(iniState, finState, hPolys, traj);
      visPtr_->visualize_traj(traj, "traj");
    }

    // NOTE collision check
    bool valid = false;
    if (generate_new_traj_success) {
      valid = validcheck(traj, replan_stamp);
    } else {
      replanStateMsg_.state = -2;
      replanState_pub_->publish(replanStateMsg_);
    }
    if (valid) {
      force_hover_ = false;
      RCLCPP_WARN(this->get_logger(), "[planner] REPLAN SUCCESS");
      replanStateMsg_.state = 0;
      replanState_pub_->publish(replanStateMsg_);
      // NOTE : if the trajectory is known, watch that direction
      Eigen::Vector3d un_known_p = traj.getPos(traj.getTotalDuration() < 1.0 ? traj.getTotalDuration() : 1.0);
      Eigen::Vector3d dp = un_known_p - odom_p;
      double yaw = std::atan2(dp.y(), dp.x());
      pub_traj(traj, yaw, replan_stamp);
      traj_poly_ = traj;
      replan_stamp_ = replan_stamp;
    } else if (force_hover_) {
      RCLCPP_ERROR(this->get_logger(), "[planner] REPLAN FAILED, HOVERING...");
      replanStateMsg_.state = 1;
      replanState_pub_->publish(replanStateMsg_);
      return;
    } else if (!validcheck(traj_poly_, replan_stamp_)) {
      force_hover_ = true;
      RCLCPP_FATAL(this->get_logger(), "[planner] EMERGENCY STOP!!!");
      replanStateMsg_.state = 2;
      replanState_pub_->publish(replanStateMsg_);
      pub_hover_p(iniState.col(0), replan_stamp);
      return;
    } else {
      RCLCPP_ERROR(this->get_logger(), "[planner] REPLAN FAILED, EXECUTE LAST TRAJ...");
      replanStateMsg_.state = 3;
      replanState_pub_->publish(replanStateMsg_);
      return;  // current generated traj invalid but last is valid
    }
    visPtr_->visualize_traj(traj, "traj");
  }

  void debug_timer_callback() {
    inflate_gridmap_pub_->publish(replanStateMsg_.occmap);
    Eigen::MatrixXd iniState;
    iniState.setZero(3, 3);
    rclcpp::Time replan_stamp = this->now() + rclcpp::Duration::from_seconds(0.03);

    iniState = Eigen::Map<Eigen::MatrixXd>(replanStateMsg_.iniState.data(), 3, 3);
    Eigen::Vector3d target_p(replanStateMsg_.target.pose.pose.position.x,
                             replanStateMsg_.target.pose.pose.position.y,
                             replanStateMsg_.target.pose.pose.position.z);
    Eigen::Vector3d target_v(replanStateMsg_.target.twist.twist.linear.x,
                             replanStateMsg_.target.twist.twist.linear.y,
                             replanStateMsg_.target.twist.twist.linear.z);

    // visualize the target and the drone velocity
    visPtr_->visualize_arrow(iniState.col(0), iniState.col(0) + iniState.col(1), "drone_vel");
    visPtr_->visualize_arrow(target_p, target_p + target_v, "target_vel");

    // visualize the ray from drone to target
    if (envPtr_->checkRayValid(iniState.col(0), target_p)) {
      visPtr_->visualize_arrow(iniState.col(0), target_p, "ray", visualization::yellow);
    } else {
      visPtr_->visualize_arrow(iniState.col(0), target_p, "ray", visualization::red);
    }

    // NOTE prediction
    std::vector<Eigen::Vector3d> target_predcit;
    if (gridmapPtr_->isOccupied(target_p)) {
      std::cout << "target is invalid!" << std::endl;
      assert(false);
    }
    bool generate_new_traj_success = prePtr_->predict(target_p, target_v, target_predcit);

    if (generate_new_traj_success) {
      Eigen::Vector3d observable_p = target_predcit.back();
      visPtr_->visualize_path(target_predcit, "car_predict");
      std::vector<Eigen::Vector3d> observable_margin;
      for (double theta = 0; theta <= 2 * M_PI; theta += 0.01) {
        observable_margin.emplace_back(observable_p + tracking_dist_ * Eigen::Vector3d(cos(theta), sin(theta), 0));
      }
      visPtr_->visualize_path(observable_margin, "observable_margin");
    }

    // NOTE path searching
    Eigen::Vector3d p_start = iniState.col(0);
    std::vector<Eigen::Vector3d> path, way_pts;
    if (generate_new_traj_success) {
      generate_new_traj_success = envPtr_->findVisiblePath(p_start, target_predcit, way_pts, path);
    }

    std::vector<Eigen::Vector3d> visible_ps;
    std::vector<double> thetas;
    Trajectory traj;
    if (generate_new_traj_success) {
      visPtr_->visualize_path(path, "astar");
      // NOTE generate visible regions
      target_predcit.pop_back();
      way_pts.pop_back();
      envPtr_->generate_visible_regions(target_predcit, way_pts,
                                        visible_ps, thetas);
      visPtr_->visualize_pointcloud(visible_ps, "visible_ps");
      visPtr_->visualize_fan_shape_meshes(target_predcit, visible_ps, thetas, "visible_region");
      // NOTE corridor generating
      std::vector<Eigen::MatrixXd> hPolys;
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> keyPts;
      // TODO change the final state
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> rays;
      for (int i = 0; i < (int)way_pts.size(); ++i) {
        rays.emplace_back(target_predcit[i], way_pts[i]);
      }
      visPtr_->visualize_pointcloud(way_pts, "way_pts");
      way_pts.insert(way_pts.begin(), p_start);
      envPtr_->pts2path(way_pts, path);
      visPtr_->visualize_path(path, "corridor_path");
      envPtr_->generateSFC(path, 2.0, hPolys, keyPts);
      envPtr_->visCorridor(hPolys);
      visPtr_->visualize_pairline(keyPts, "keyPts");

      // NOTE trajectory optimization
      Eigen::MatrixXd finState;
      finState.setZero(3, 3);
      finState.col(0) = path.back();
      finState.col(1) = target_v;

      generate_new_traj_success = trajOptPtr_->generate_traj(iniState, finState,
                                                             target_predcit, visible_ps, thetas,
                                                             hPolys, traj);
      visPtr_->visualize_traj(traj, "traj");
    }
    if (!generate_new_traj_success) {
      return;
      // assert(false);
    }
    // check
    bool valid = true;
    std::vector<Eigen::Vector3d> check_pts, invalid_pts;
    double t0 = (this->now() - replan_stamp).seconds();
    t0 = t0 > 0.0 ? t0 : 0.0;
    double check_dur = 1.0;
    double delta_t = check_dur < traj.getTotalDuration() ? check_dur : traj.getTotalDuration();
    for (double t = t0; t < t0 + delta_t; t += 0.1) {
      Eigen::Vector3d p = traj.getPos(t);
      check_pts.push_back(p);
      if (gridmapPtr_->isOccupied(p)) {
        invalid_pts.push_back(p);
      }
    }
    visPtr_->visualize_path(invalid_pts, "invalid_pts");
    visPtr_->visualize_path(check_pts, "check_pts");
    valid = validcheck(traj, replan_stamp);
    if (!valid) {
      std::cout << "invalid!" << std::endl;
    }
  }

  bool validcheck(const Trajectory& traj, const rclcpp::Time& t_start, const double& check_dur = 1.0) {
    double t0 = (this->now() - t_start).seconds();
    t0 = t0 > 0.0 ? t0 : 0.0;
    double delta_t = check_dur < traj.getTotalDuration() ? check_dur : traj.getTotalDuration();
    for (double t = t0; t < t0 + delta_t; t += 0.01) {
      Eigen::Vector3d p = traj.getPos(t);
      if (gridmapPtr_->isOccupied(p)) {
        return false;
      }
    }
    return true;
  }

  void init() {
    // Stop the one-shot initialization timer
    init_timer_->cancel();
    
    // Shared pointer to this node (required by helper classes)
    auto node_ptr = shared_from_this();

    // set parameters of planning
    this->declare_parameter("plan_hz", 10);
    this->declare_parameter("tracking_dur", 0.0);
    this->declare_parameter("tracking_dist", 0.0);
    this->declare_parameter("tolerance_d", 0.0);
    this->declare_parameter("debug", false);
    this->declare_parameter("fake", false);

    int plan_hz;
    this->get_parameter("plan_hz", plan_hz);
    this->get_parameter("tracking_dur", tracking_dur_);
    this->get_parameter("tracking_dist", tracking_dist_);
    this->get_parameter("tolerance_d", tolerance_d_);
    this->get_parameter("debug", debug_);
    this->get_parameter("fake", fake_);

    gridmapPtr_ = std::make_shared<mapping::OccGridMap>();
    envPtr_ = std::make_shared<env::Env>(node_ptr, gridmapPtr_);
    visPtr_ = std::make_shared<visualization::Visualization>(node_ptr);
    trajOptPtr_ = std::make_shared<traj_opt::TrajOpt>(node_ptr);
    prePtr_ = std::make_shared<prediction::Predict>(node_ptr);

    heartbeat_pub_ = this->create_publisher<std_msgs::msg::Empty>("heartbeat", 10);
    traj_pub_ = this->create_publisher<quadrotor_msgs::msg::PolyTraj>("trajectory", 1);
    replanState_pub_ = this->create_publisher<quadrotor_msgs::msg::ReplanState>("replanState", 1);

    if (debug_) {
      plan_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / plan_hz), std::bind(&PlanningNode::debug_timer_callback, this));
      
      std::string debug_path = ament_index_cpp::get_package_share_directory("planning") + "/../../../debug/replan_state.bin";
      wr_msg::readMsg(replanStateMsg_, debug_path);
      inflate_gridmap_pub_ = this->create_publisher<quadrotor_msgs::msg::OccMap3d>("gridmap_inflate", 10);
      gridmapPtr_->from_msg(replanStateMsg_.occmap);
      prePtr_->setMap(*gridmapPtr_);
      std::cout << "plan state: " << replanStateMsg_.state << std::endl;
    } else if (fake_) {
      plan_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / plan_hz), std::bind(&PlanningNode::fake_timer_callback, this));
    } else {
      plan_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / plan_hz), std::bind(&PlanningNode::plan_timer_callback, this));
    }
    
    // QoS for high-rate data: Reliable, Depth 1 (matches queue_size=1)
    auto qos_best_effort = rclcpp::QoS(1).reliable();
    auto qos_reliable_10 = rclcpp::QoS(10).reliable();

    // Use tcpNoDelay equivalent QoS? ROS 2 Reliable + Volatile is usually low latency
    gridmap_sub_ = this->create_subscription<quadrotor_msgs::msg::OccMap3d>(
        "gridmap_inflate", qos_best_effort, std::bind(&PlanningNode::gridmap_callback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", qos_reliable_10, std::bind(&PlanningNode::odom_callback, this, std::placeholders::_1));
    target_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "target", qos_reliable_10, std::bind(&PlanningNode::target_callback, this, std::placeholders::_1));
    triger_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "triger", qos_reliable_10, std::bind(&PlanningNode::triger_callback, this, std::placeholders::_1));
    land_triger_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "land_triger", qos_reliable_10, std::bind(&PlanningNode::land_triger_callback, this, std::placeholders::_1));
    
    RCLCPP_WARN(this->get_logger(), "Planning node initialized!");
  }

 public:
  PlanningNode(const rclcpp::NodeOptions& options) : Node("planning_nodelet", options) {
      // Use a one-shot timer to defer initialization until the shared_ptr to this node is established.
      // This is necessary because helper classes (Env, Predict, etc.) require a shared_ptr to the node.
      init_timer_ = this->create_wall_timer(0ms, std::bind(&PlanningNode::init, this));
  }
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace planning

RCLCPP_COMPONENTS_REGISTER_NODE(planning::PlanningNode)
