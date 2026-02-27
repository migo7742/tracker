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
  rclcpp::Time replan_stamp_{0, 0, RCL_ROS_TIME};
  int traj_id_ = 0;
  bool wait_hover_ = true;
  bool force_hover_ = true;
  Eigen::Vector3d hover_pos_ = Eigen::Vector3d::Zero();
  rclcpp::Time force_hover_start_{0, 0, RCL_ROS_TIME};
  int consecutive_replan_failures_ = 0;
  static constexpr int MAX_CONSECUTIVE_FAILURES = 15;
  
  // Rate-limit target position jumps to reject EKF outliers
  Eigen::Vector3d last_valid_target_ = Eigen::Vector3d::Zero();
  bool has_valid_target_ = false;

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

  static constexpr double MIN_HOVER_Z = 1.3;  // ~1.75m above ground in world frame
  // Z offset added to target for all planning computations.
  // Reduced from 1.0 to 0.3 to avoid excessive climbing.
  static constexpr double TRACKING_Z_OFFSET = 0.3;
  // Maximum XY distance for a single planning step. When the target is farther,
  // we create a virtual intermediate target along the direction to avoid generating
  // trajectories that force excessive speed (distance/tracking_dur > vmax).
  static constexpr double MAX_PLAN_DIST = 6.0;  // meters, ~2m/s average over 3s

  // Compute a safe hover point ahead of current position in the velocity direction
  // to avoid abrupt stops. The drone will decelerate naturally to this point.
  Eigen::Vector3d compute_brake_point(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel) {
    double speed = vel.norm();
    if (speed < 0.3) {
      // Already slow enough, hover at current position
      return pos;
    }
    // Estimate braking distance: v² / (2*a), use amax=1.5
    double brake_dist = speed * speed / (2.0 * 1.5);
    brake_dist = std::min(brake_dist, 2.0);  // cap at 2m
    Eigen::Vector3d brake_p = pos + vel.normalized() * brake_dist;
    brake_p.z() = std::max(brake_p.z(), MIN_HOVER_Z);
    return brake_p;
  }

  void pub_hover_p(const Eigen::Vector3d& hover_p, const rclcpp::Time& stamp, double yaw = NAN) {
    quadrotor_msgs::msg::PolyTraj traj_msg;
    traj_msg.hover = true;
    traj_msg.hover_p.resize(3);
    traj_msg.hover_p[0] = hover_p[0];
    traj_msg.hover_p[1] = hover_p[1];
    traj_msg.hover_p[2] = std::max(hover_p[2], MIN_HOVER_Z);
    traj_msg.start_time = stamp;
    traj_msg.traj_id = traj_id_++;
    traj_msg.yaw = std::isnan(yaw) ? 0.0f : static_cast<float>(yaw);
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
    // Auto-trigger: once we have target data, start planning automatically
    if (!triger_received_) {
      goal_ << msgPtr->pose.pose.position.x, msgPtr->pose.pose.position.y, 0.9;
      triger_received_ = true;
      RCLCPP_INFO(this->get_logger(), "[planner] Auto-triggered by target data (goal=%.2f, %.2f, %.2f)",
                  goal_.x(), goal_.y(), goal_.z());
    }
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
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "Waiting: odom=%d map=%d triger=%d target=%d",
        odom_received_.load(), map_received_.load(),
        triger_received_.load(), target_received_.load());
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
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "[planner] Waiting for trigger (publish PoseStamped to /triger)");
      return;
    }
    if (!target_received_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "[planner] Waiting for target EKF data");
      return;
    }
    // NOTE obtain state of target
    while (target_lock_.test_and_set())
      ;
    replanStateMsg_.target = target_msg_;
    target_lock_.clear();

    double target_age = (this->now() - rclcpp::Time(replanStateMsg_.target.header.stamp)).seconds();
      if (target_age > 10.0) {
      if (!wait_hover_) {
        Eigen::Vector3d brake_p = compute_brake_point(odom_p, odom_v);
        pub_hover_p(brake_p, this->now());
        wait_hover_ = true;
        force_hover_ = true;
        force_hover_start_ = this->now();
      }
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "[planner] target data stale (%.1fs), hovering...", target_age);
      return;
    }

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

    // Check EKF observation staleness: covariance[0] contains seconds since last YOLO update.
    // If no real observation for > STALE_OBS_THRESHOLD, the EKF is purely predicting
    // (velocity is decaying but position may still be drifting). Hover and wait.
    static constexpr double STALE_OBS_THRESHOLD = 3.0;  // seconds
    double obs_age = replanStateMsg_.target.pose.covariance[0];
    if (obs_age > STALE_OBS_THRESHOLD) {
      // Compute yaw towards last known target position so camera faces
      // the target — this gives YOLO a chance to re-detect after turning.
      Eigen::Vector3d dp_stale = target_p - odom_p;
      double stale_yaw = std::atan2(dp_stale.y(), dp_stale.x());
      if (!wait_hover_) {
        Eigen::Vector3d brake_p = compute_brake_point(odom_p, odom_v);
        pub_hover_p(brake_p, this->now(), stale_yaw);
        wait_hover_ = true;
        force_hover_ = true;
        force_hover_start_ = this->now();
      } else {
        // Keep updating yaw while hovering to track target direction
        pub_hover_p(hover_pos_, this->now(), stale_yaw);
      }
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "[planner] EKF observation stale (%.1fs no YOLO update), hovering & facing target...", obs_age);
      return;
    }

    // Sanity check: reject clearly invalid target estimates (EKF drift)
    // Only reject extreme outliers — original project had NO distance check.
    // Use a generous threshold to avoid blocking normal tracking.
    {
      double target_z_in_world = target_p.z() + 0.45;
      bool z_invalid = target_z_in_world < -0.5 || target_z_in_world > 5.0;
      double xy_dist_raw = (target_p - odom_p).head(2).norm();
      bool dist_invalid = xy_dist_raw > 30.0;  // absolute 30m limit, not relative to tracking_dist
      if (z_invalid || dist_invalid) {
        if (!wait_hover_) {
          Eigen::Vector3d brake_p = compute_brake_point(odom_p, odom_v);
          pub_hover_p(brake_p, this->now());
          wait_hover_ = true;
          force_hover_ = true;
          force_hover_start_ = this->now();
        }
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "[planner] target estimate unreasonable (z_world=%.2f, xy_dist=%.2f), hovering",
          target_z_in_world, xy_dist_raw);
        return;
      }
    }

    // Rate-limit target position jumps: if the target moved more than MAX_JUMP
    // since last valid position, clamp it. This prevents EKF glitches (e.g., 
    // target y jumping from 1.5 to -5.9 in one step) from sending the drone
    // on wild trajectories.
    {
      static constexpr double MAX_JUMP = 1.0;  // meters, max target jump per replan cycle (~10Hz)
      if (has_valid_target_) {
        double jump = (target_p - last_valid_target_).head(2).norm();
        if (jump > MAX_JUMP) {
          Eigen::Vector3d dir = (target_p - last_valid_target_);
          dir.z() = 0;
          dir.normalize();
          Eigen::Vector3d clamped = last_valid_target_ + dir * MAX_JUMP;
          clamped.z() = target_p.z();  // keep z as-is (handled separately)
          RCLCPP_WARN(this->get_logger(),
            "[planner] target jumped %.2fm (%.2f,%.2f)->(%.2f,%.2f), clamping to (%.2f,%.2f)",
            jump, last_valid_target_.x(), last_valid_target_.y(),
            target_p.x(), target_p.y(), clamped.x(), clamped.y());
          target_p = clamped;
        }
      }
      last_valid_target_ = target_p;
      has_valid_target_ = true;
    }

    // NOTE force-hover: wait for drone to settle before replanning.
    // IMPORTANT: odom_v from /Odometry is ZERO (small_point_lio doesn't fill twist),
    // so we CANNOT use velocity to determine settling. Use a fixed timeout instead.
    if (force_hover_) {
      double hover_elapsed = (this->now() - force_hover_start_).seconds();
      if (hover_elapsed < 1.0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
          "[planner] force_hover: waiting %.1fs/1.0s to settle", hover_elapsed);
        return;
      }
      force_hover_ = false;
    }

    // Safety: clamp target EKF position so it can't be closer than a minimum
    {
      Eigen::Vector2d dp_xy = (target_p - odom_p).head(2);
      double xy_dist = dp_xy.norm();
      double min_target_dist = tracking_dist_ * 0.4;
      if (xy_dist < min_target_dist && xy_dist > 0.01) {
        Eigen::Vector2d dir = dp_xy / xy_dist;
        target_p.head(2) = odom_p.head(2) + dir * min_target_dist;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "[planner] target est too close (xy=%.2f < %.2f), clamping outward",
          xy_dist, min_target_dist);
      }
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
      // Clamp target z to reasonable ground-target range BEFORE adding offset.
      // This is the last line of defense against EKF z drift.
      // Ground is at z ≈ -0.45 in odom frame; person feet are near there.
      target_p.z() = std::clamp(target_p.z(), -0.8, 0.5);
      // Compute desired tracking height offset for traj_opt (drone should fly above target)
      target_p.z() += TRACKING_Z_OFFSET;
      // Clamp target_p.z (planning height) to MIN_HOVER_Z
      target_p.z() = std::max(target_p.z(), MIN_HOVER_Z);
      // Also clamp target_v.z to prevent downward drift from polluting planning
      if (target_p.z() <= MIN_HOVER_Z + 0.1 && target_v.z() < 0) {
        target_v.z() = 0.0;  // Don't predict target going underground
      }

      // Limit max planning distance: when target is too far, create a virtual
      // intermediate target along the direction. This prevents the optimizer
      // from generating trajectories that exceed vmax (distance/tracking_dur).
      double dist_to_target_xy = (target_p - odom_p).head(2).norm();
      if (dist_to_target_xy > MAX_PLAN_DIST + tracking_dist_) {
        // Virtual target: MAX_PLAN_DIST along the direction, keep z the same
        Eigen::Vector3d dir = (target_p - odom_p);
        dir.z() = 0;
        dir.normalize();
        Eigen::Vector3d virtual_target = odom_p + dir * (MAX_PLAN_DIST + tracking_dist_);
        virtual_target.z() = target_p.z();
        // Scale velocity proportionally (still heading towards real target)
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "[planner] Target too far (%.1fm), clamping plan dist to %.1fm",
          dist_to_target_xy, MAX_PLAN_DIST + tracking_dist_);
        target_p = virtual_target;
      }
      // NOTE determin whether to replan
      Eigen::Vector3d dp = target_p - odom_p;
      // std::cout << "dist : " << dp.norm() << std::endl;
      double desired_yaw = std::atan2(dp.y(), dp.x());
      Eigen::Vector3d project_yaw = odom_q.toRotationMatrix().col(0);  // NOTE ZYX
      double now_yaw = std::atan2(project_yaw.y(), project_yaw.x());
      double dist_err = std::fabs(dp.head(2).norm() - tracking_dist_);
      double yaw_err = std::fabs(desired_yaw - now_yaw);
      if (yaw_err > M_PI) yaw_err = 2 * M_PI - yaw_err;

      // NOTE determin whether to replan: hover if close enough and stable
      // Hover condition: match original project logic (simple, no hysteresis)
      // Original: |dist - tracking_dist| < tolerance_d && v < 0.1 && target_v < 0.2 && |yaw_err| < 0.5
      bool should_hover = (dist_err < tolerance_d_) &&
                           (odom_v.norm() < 0.1) &&
                           (target_v.norm() < 0.2) &&
                           (yaw_err < 0.5);

      if (should_hover) {
        if (!wait_hover_) {
          hover_pos_ = odom_p;  // Lock hover position on entry
          hover_pos_.z() = std::max(hover_pos_.z(), MIN_HOVER_Z);  // Clamp z up
          pub_hover_p(hover_pos_, this->now(), desired_yaw);
          wait_hover_ = true;
          RCLCPP_WARN(this->get_logger(),
            "[planner] HOVERING (dist_err=%.2f, yaw_err=%.2f, v=%.2f)", dist_err, yaw_err, odom_v.norm());
        } else {
          // Only update yaw while hovering, keep position locked
          pub_hover_p(hover_pos_, this->now(), desired_yaw);
        }
        replanStateMsg_.state = -1;
        replanState_pub_->publish(replanStateMsg_);
        return;
      } else {
        if (wait_hover_) {
          RCLCPP_WARN(this->get_logger(),
            "[planner] Leaving hover (dist_err=%.2f, yaw_err=%.2f, v_drone=%.2f, v_target=%.2f)",
            dist_err, yaw_err, odom_v.norm(), target_v.norm());
        }
        wait_hover_ = false;
      }

      // target_p.z now has TRACKING_Z_OFFSET applied.
      // All downstream (prediction, path, corridor, traj_opt) use this consistently.
    }

    // NOTE obtain map
    while (gridmap_lock_.test_and_set())
      ;
    gridmapPtr_->from_msg(map_msg_);
    replanStateMsg_.occmap = map_msg_;
    gridmap_lock_.clear();

    // Clear a small region around the target in the local map copy.
    // The target (a person) is itself detected as an obstacle by the depth
    // camera. Without this, prediction search fails immediately because
    // the starting cell is occupied ("no way!"). The mapping node already
    // does this via use_mask, but the mask may be too small when combined
    // with inflation (inflate_size=2 → 0.3m extra), and the target might
    // be near real obstacles like pillars whose inflated zone overlaps.
    // We clear ±0.6m in xy and ±1.0m in z around the target.
    {
      Eigen::Vector3d mask_ld = target_p;
      Eigen::Vector3d mask_ru = target_p;
      mask_ld.x() -= 0.6; mask_ld.y() -= 0.6; mask_ld.z() -= 1.0;
      mask_ru.x() += 0.6; mask_ru.y() += 0.6; mask_ru.z() += 1.0;
      gridmapPtr_->setFree(mask_ld, mask_ru);
    }

    prePtr_->setMap(*gridmapPtr_);

    // visualize the ray from drone to target
    if (envPtr_->checkRayValid(odom_p, target_p)) {
      visPtr_->visualize_arrow(odom_p, target_p, "ray", visualization::yellow);
    } else {
      visPtr_->visualize_arrow(odom_p, target_p, "ray", visualization::red);
    }

    // NOTE prediction
    // target_p.z already has TRACKING_Z_OFFSET applied — use it directly
    // for prediction, path search, corridor, and traj_opt (all at same z).
    std::vector<Eigen::Vector3d> target_predcit;
    bool generate_new_traj_success = prePtr_->predict(target_p, target_v, target_predcit);
    if (!generate_new_traj_success) {
      // Fallback: constant-velocity linear prediction when search-based prediction fails
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "[planner] prediction search failed, using linear fallback");
      target_predcit.clear();
      double dt = 0.2;  // tracking_dt
      double dur = 3.0; // tracking_dur
      // Zero out target_v.z for linear fallback: the EKF's z velocity estimate
      // is unreliable for a ground target, and non-zero v.z causes the predicted
      // trajectory to climb/dive, leading to excessive altitude changes.
      Eigen::Vector3d fallback_v = target_v;
      fallback_v.z() = 0.0;
      for (double t = 0; t <= dur + 1e-6; t += dt) {
        Eigen::Vector3d pt = target_p + fallback_v * t;
        // Clamp predicted z: target (with TRACKING_Z_OFFSET applied) should not go below MIN_HOVER_Z
        pt.z() = std::max(pt.z(), MIN_HOVER_Z);
        // Stop predicting into obstacles: truncate here and fill remaining with last valid point
        if (t > 0 && gridmapPtr_->isOccupied(pt)) {
          Eigen::Vector3d last_valid = target_predcit.back();
          while (target_predcit.size() < static_cast<size_t>((dur / dt) + 2)) {
            target_predcit.push_back(last_valid);
          }
          break;
        }
        target_predcit.push_back(pt);
      }
      generate_new_traj_success = true;
    }
    // Post-process: truncate any prediction points that land inside obstacles.
    // This protects findVisiblePath from searching toward unreachable targets,
    // which causes search timeouts.
    if (generate_new_traj_success && target_predcit.size() > 2) {
      for (size_t i = 1; i < target_predcit.size(); ++i) {
        if (gridmapPtr_->isOccupied(target_predcit[i])) {
          Eigen::Vector3d last_valid = target_predcit[i - 1];
          // Fill remaining with last valid point
          for (size_t j = i; j < target_predcit.size(); ++j) {
            target_predcit[j] = last_valid;
          }
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "[planner] prediction truncated at step %zu/%zu (occupied)", i, target_predcit.size());
          break;
        }
      }
    }
    {
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
      iniState.col(0) = odom_p;
      iniState.col(1) = odom_v;
    } else {
      Eigen::Vector3d traj_p = traj_poly_.getPos(replan_t);
      double tracking_err = (traj_p - odom_p).norm();
      if (tracking_err > 0.8) {
        iniState.col(0) = odom_p;
        iniState.col(1) = odom_v;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "[planner] tracking error %.2fm, using odom for iniState", tracking_err);
      } else {
        iniState.col(0) = traj_p;
        iniState.col(1) = traj_poly_.getVel(replan_t);
        iniState.col(2) = traj_poly_.getAcc(replan_t);
      }
    }
    // Clamp iniState z to minimum flight altitude to break the low-z feedback loop:
    // if drone is below MIN_HOVER_Z, start replanning from MIN_HOVER_Z so the new
    // trajectory pulls the drone back up instead of staying at the low altitude.
    if (iniState.col(0).z() < MIN_HOVER_Z) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "[planner] iniState.z=%.2f < MIN_HOVER_Z=%.1f, clamping up", iniState.col(0).z(), MIN_HOVER_Z);
      iniState.col(0).z() = MIN_HOVER_Z;
    }
    replanStateMsg_.header.stamp = this->now();
    replanStateMsg_.ini_state.resize(9);
    Eigen::Map<Eigen::MatrixXd>(replanStateMsg_.ini_state.data(), 3, 3) = iniState;

    // NOTE path searching
    Eigen::Vector3d p_start = iniState.col(0);
    std::vector<Eigen::Vector3d> path, way_pts;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "[planner] drone=(%.2f,%.2f,%.2f) target=(%.2f,%.2f,%.2f) dist3d=%.2f distXY=%.2f trackDist=%.2f",
      odom_p.x(), odom_p.y(), odom_p.z(),
      target_p.x(), target_p.y(), target_p.z(),
      (odom_p - target_p).norm(),
      (odom_p - target_p).head(2).norm(),
      tracking_dist_);
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

    // Sanity check: reject paths that go in the OPPOSITE direction from the target.
    // This can happen when findVisiblePath wraps around dense obstacles.
    // When detected, treat it as "too close" and hover toward target rather than
    // accumulating REPLAN FAILED counts toward EMERGENCY STOP.
    if (generate_new_traj_success && path.size() >= 2 && !land_triger_received_) {
      Eigen::Vector2d drone_to_target = (target_p - p_start).head(2);
      Eigen::Vector2d drone_to_pathend = (path.back() - p_start).head(2);
      double dot = drone_to_target.dot(drone_to_pathend);
      if (dot < 0 && drone_to_target.norm() > 1.0) {
        // Path endpoint is behind the drone relative to target direction
        RCLCPP_WARN(this->get_logger(),
          "[planner] path direction reversed (dot=%.2f), hovering instead. "
          "path_end=(%.2f,%.2f), target=(%.2f,%.2f)",
          dot, path.back().x(), path.back().y(), target_p.x(), target_p.y());
        // Instead of rejecting and counting toward EMERGENCY STOP,
        // hover at current position facing target. This is much safer.
        double yaw_to_target = std::atan2(drone_to_target.y(), drone_to_target.x());
        Eigen::Vector3d hover_p = odom_p;
        hover_p.z() = std::max(odom_p.z(), MIN_HOVER_Z);
        pub_hover_p(hover_p, this->now(), yaw_to_target);
        replanStateMsg_.state = -1;
        replanState_pub_->publish(replanStateMsg_);
        consecutive_replan_failures_ = 0;  // Reset failure count — this is a controlled hover, not a failure
        return;
      }
    }

    if (generate_new_traj_success) {
      // Clamp all path z values to MIN_HOVER_Z to prevent low-altitude corridors
      for (auto& pt : path) {
        if (pt.z() < MIN_HOVER_Z) pt.z() = MIN_HOVER_Z;
      }
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "[planner] path: start=(%.2f,%.2f,%.2f) end=(%.2f,%.2f,%.2f) pts=%zu",
        path.front().x(), path.front().y(), path.front().z(),
        path.back().x(), path.back().y(), path.back().z(),
        path.size());
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

        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> rays;
        for (int i = 0; i < (int)way_pts.size(); ++i) {
          rays.emplace_back(target_predcit[i], way_pts[i]);
        }
        visPtr_->visualize_pointcloud(way_pts, "way_pts");
        way_pts.insert(way_pts.begin(), p_start);
        if (!envPtr_->pts2path(way_pts, path)) {
          generate_new_traj_success = false;
        }
      }
    }
    if (generate_new_traj_success) {
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
      // Clamp finState z to ensure traj_opt doesn't get an underground target
      if (finState(2, 0) < MIN_HOVER_Z) {
        finState(2, 0) = MIN_HOVER_Z;
      }
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "[planner] finState=(%.2f,%.2f,%.2f) corridors=%zu",
        finState.col(0).x(), finState.col(0).y(), finState.col(0).z(),
        hPolys.size());
      // target_predcit z is already at correct height (original + TRACKING_Z_OFFSET)
      // consistent with path and corridor — no modification needed
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
      consecutive_replan_failures_ = 0;
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
    } else if (!validcheck(traj_poly_, replan_stamp_)) {
      force_hover_ = true;
      RCLCPP_FATAL(this->get_logger(), "[planner] EMERGENCY STOP!!!");
      replanStateMsg_.state = 2;
      replanState_pub_->publish(replanStateMsg_);
      force_hover_start_ = this->now();
      // Use brake point instead of current position to avoid abrupt stop
      Eigen::Vector3d brake_p = compute_brake_point(odom_p, odom_v);
      pub_hover_p(brake_p, replan_stamp);
      return;
    } else {
      consecutive_replan_failures_++;
      if (consecutive_replan_failures_ >= MAX_CONSECUTIVE_FAILURES) {
        force_hover_ = true;
        force_hover_start_ = this->now();
        consecutive_replan_failures_ = 0;
        // Use brake point instead of current position to avoid abrupt stop
        Eigen::Vector3d brake_p = compute_brake_point(odom_p, odom_v);
        pub_hover_p(brake_p, this->now());
        RCLCPP_ERROR(this->get_logger(),
          "[planner] %d consecutive failures, force hovering at brake point (v=%.2f)", MAX_CONSECUTIVE_FAILURES, odom_v.norm());
        return;
      }
      RCLCPP_ERROR(this->get_logger(), "[planner] REPLAN FAILED, EXECUTE LAST TRAJ... (%d/%d)",
        consecutive_replan_failures_, MAX_CONSECUTIVE_FAILURES);
      replanStateMsg_.state = 3;
      replanState_pub_->publish(replanStateMsg_);
      return;
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
    if (force_hover_) {
      double hover_elapsed = (this->now() - force_hover_start_).seconds();
      if (odom_v.norm() > 0.3 && hover_elapsed < 3.0) {
        return;
      }
      if (hover_elapsed >= 3.0) {
        force_hover_ = false;
      }
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
      iniState.col(0) = odom_p;
      iniState.col(1) = odom_v;
    } else {
      iniState.col(0) = traj_poly_.getPos(replan_t);
      iniState.col(1) = traj_poly_.getVel(replan_t);
      iniState.col(2) = traj_poly_.getAcc(replan_t);
    }
    replanStateMsg_.header.stamp = this->now();
    replanStateMsg_.ini_state.resize(9);
    Eigen::Map<Eigen::MatrixXd>(replanStateMsg_.ini_state.data(), 3, 3) = iniState;

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
      force_hover_start_ = this->now();
      RCLCPP_FATAL(this->get_logger(), "[planner] EMERGENCY STOP!!!");
      replanStateMsg_.state = 2;
      replanState_pub_->publish(replanStateMsg_);
      Eigen::Vector3d brake_p = compute_brake_point(odom_p, odom_v);
      pub_hover_p(brake_p, replan_stamp);
      return;
    } else {
      RCLCPP_ERROR(this->get_logger(), "[planner] REPLAN FAILED, EXECUTE LAST TRAJ...");
      replanStateMsg_.state = 3;
      replanState_pub_->publish(replanStateMsg_);
      return;
    }
    visPtr_->visualize_traj(traj, "traj");
  }

  void debug_timer_callback() {
    inflate_gridmap_pub_->publish(replanStateMsg_.occmap);
    Eigen::MatrixXd iniState;
    iniState.setZero(3, 3);
    rclcpp::Time replan_stamp = this->now() + rclcpp::Duration::from_seconds(0.03);

    iniState = Eigen::Map<Eigen::MatrixXd>(replanStateMsg_.ini_state.data(), 3, 3);
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
      init_timer_ = this->create_wall_timer(0ms, std::bind(&PlanningNode::init, this));
  }

  ~PlanningNode() override {
    if (plan_timer_) plan_timer_->cancel();
    if (init_timer_) init_timer_->cancel();
  }
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace planning

RCLCPP_COMPONENTS_REGISTER_NODE(planning::PlanningNode)
