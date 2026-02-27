#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <limits>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_attitude_setpoint.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "quadrotor_msgs/msg/position_command.hpp"
#include "std_msgs/msg/string.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std::chrono_literals;

class OffboardControl : public rclcpp::Node {
public:
    OffboardControl() : Node("offboard_geometric_controller") {
        // --- 参数设置 ---
        this->declare_parameter("kp_x", 6.0);
        this->declare_parameter("kp_y", 6.0);
        this->declare_parameter("kp_z", 10.0);
        this->declare_parameter("kv_x", 3.0); 
        this->declare_parameter("kv_y", 3.0);
        this->declare_parameter("kv_z", 5.0);
        this->declare_parameter("hover_throttle", 0.5); // 关键：悬停油门
        this->declare_parameter("grav_acc", 9.81);

        auto qos_best_effort = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
        auto qos_reliable = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_default);

        status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "fmu/out/vehicle_status", qos_best_effort,
            std::bind(&OffboardControl::vehicle_status_callback, this, std::placeholders::_1));

        vehicle_local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", qos_best_effort,
            std::bind(&OffboardControl::vehicle_local_position_callback, this, std::placeholders::_1));

        vehicle_visual_odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/in/vehicle_visual_odometry", qos_best_effort,
            std::bind(&OffboardControl::vehicle_visual_odom_callback, this, std::placeholders::_1));

        // Subscribe to /Odometry (same source as planning) to get odom z for z-offset correction
        ros_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10,
            std::bind(&OffboardControl::ros_odom_callback, this, std::placeholders::_1));

        planning_pos_cmd_sub_ = this->create_subscription<quadrotor_msgs::msg::PositionCommand>(
            "/drone_0_planning/pos_cmd", qos_best_effort,
            std::bind(&OffboardControl::planning_pos_cmd_callback, this, std::placeholders::_1));

        mode_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/mode_key", qos_reliable,
            std::bind(&OffboardControl::mode_cmd_callback, this, std::placeholders::_1));

        publisher_offboard_mode_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "fmu/in/offboard_control_mode", qos_reliable);
        
        publisher_trajectory_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "fmu/in/trajectory_setpoint", qos_reliable);

        publisher_attitude_ = this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
            "fmu/in/vehicle_attitude_setpoint", qos_reliable);
        
        publisher_vehicle_command_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", qos_reliable);

        // 50Hz 控制循环
        timer_ = this->create_wall_timer(
            20ms, std::bind(&OffboardControl::cmdloop_callback, this));
        
        control_mode_ = "m";
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_visual_odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ros_odom_sub_;
    rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr planning_pos_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_cmd_sub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr publisher_offboard_mode_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr publisher_trajectory_;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr publisher_attitude_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr publisher_vehicle_command_;

    std::string control_mode_;
    px4_msgs::msg::VehicleLocalPosition vehicle_local_position_;
    px4_msgs::msg::VehicleOdometry vehicle_visual_odom_;
    px4_msgs::msg::VehicleStatus vehicle_status_;
    quadrotor_msgs::msg::PositionCommand latest_planning_msg_;

    uint8_t nav_state_;
    uint8_t arming_state_;
    
    bool vehicle_local_position_received_ = false;
    bool vehicle_visual_odom_received_ = false;
    bool ros_odom_received_ = false;
    bool planning_pos_command_received_ = false;
    bool takeoff_hover_des_set_ = false;
    bool offboard_hover_des_set_ = false;
    rclcpp::Time last_planning_cmd_time_;
    static constexpr double CMD_TIMEOUT_SEC = 0.5;

    px4_msgs::msg::TrajectorySetpoint hover_setpoint_;

    // Z from /Odometry in NED (for z-offset correction in geometric controller)
    double odom_z_ned_ = 0.0;  // -odom_z_enu

    double quaternion_to_yaw(double w, double x, double y, double z) {
        double siny_cosp = 2 * (w * z + x * y);
        double cosy_cosp = 1 - 2 * (y * y + z * z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        nav_state_ = msg->nav_state;
        arming_state_ = msg->arming_state;
        vehicle_status_ = *msg;
    }
    void vehicle_visual_odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        vehicle_visual_odom_ = *msg;
        vehicle_visual_odom_received_ = true;
    }
    void ros_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Only extract z for z-offset correction (ENU z -> NED z = -z)
        odom_z_ned_ = -msg->pose.pose.position.z;
        ros_odom_received_ = true;
    }
    void vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        vehicle_local_position_ = *msg;
        vehicle_local_position_received_ = true;
    }
    void planning_pos_cmd_callback(const quadrotor_msgs::msg::PositionCommand::SharedPtr msg) {
        latest_planning_msg_ = *msg;
        planning_pos_command_received_ = true;
        last_planning_cmd_time_ = this->now();
    }
    void mode_cmd_callback(const std_msgs::msg::String::SharedPtr msg) {
        control_mode_ = msg->data;
        if (control_mode_ == "t" || control_mode_ == "p") {
            takeoff_hover_des_set_ = false;
            offboard_hover_des_set_ = false;
        }
    }

    // =========================================================================
    // [核心修正] 几何控制器 (带防坠机保护)
    // =========================================================================
    void calculate_geometric_control() {
        if (!vehicle_local_position_received_ && !vehicle_visual_odom_received_) return;

        double kp_x, kp_y, kp_z, kv_x, kv_y, kv_z, hover_throttle, g;
        // 建议稍微增大 Z 轴增益，让高度修正更积极
        this->get_parameter("kp_x", kp_x); this->get_parameter("kp_y", kp_y); this->get_parameter("kp_z", kp_z);
        this->get_parameter("kv_x", kv_x); this->get_parameter("kv_y", kv_y); this->get_parameter("kv_z", kv_z);
        this->get_parameter("hover_throttle", hover_throttle);
        this->get_parameter("grav_acc", g);

        if (latest_planning_msg_.kx[0] > 1e-3) {
            kp_x = latest_planning_msg_.kx[0]; kp_y = latest_planning_msg_.kx[1]; kp_z = latest_planning_msg_.kx[2];
            kv_x = latest_planning_msg_.kv[0]; kv_y = latest_planning_msg_.kv[1]; kv_z = latest_planning_msg_.kv[2];
        }
        Eigen::Vector3d Kp(kp_x, kp_y, kp_z);
        Eigen::Vector3d Kv(kv_x, kv_y, kv_z);

        // 1. 获取状态 (NED)
        Eigen::Vector3d pos_curr, vel_curr;
        if (vehicle_local_position_received_) {
            pos_curr << vehicle_local_position_.x, vehicle_local_position_.y, vehicle_local_position_.z;
            vel_curr << vehicle_local_position_.vx, vehicle_local_position_.vy, vehicle_local_position_.vz;
        } else {
            pos_curr << vehicle_visual_odom_.position[0], vehicle_visual_odom_.position[1], vehicle_visual_odom_.position[2];
            vel_curr << vehicle_visual_odom_.velocity[0], vehicle_visual_odom_.velocity[1], vehicle_visual_odom_.velocity[2];
        }
        // Z-offset correction: planning uses /Odometry z, but pos_curr uses
        // vehicle_local_position z which has a different origin (~0.77m offset).
        // Replace only the z component with odom z to match planning's coordinate.
        // XY and velocity are kept from vehicle_local_position (no origin offset issue for xy/vel).
        if (ros_odom_received_) {
            pos_curr(2) = odom_z_ned_;
        }

        // 2. 转换指令 (ENU -> NED)
        Eigen::Vector3d pos_des_enu(latest_planning_msg_.position.x, latest_planning_msg_.position.y, latest_planning_msg_.position.z);
        Eigen::Vector3d vel_des_enu(latest_planning_msg_.velocity.x, latest_planning_msg_.velocity.y, latest_planning_msg_.velocity.z);
        Eigen::Vector3d acc_des_enu(latest_planning_msg_.acceleration.x, latest_planning_msg_.acceleration.y, latest_planning_msg_.acceleration.z);
        
        Eigen::Vector3d pos_des_ned(pos_des_enu.y(), pos_des_enu.x(), -pos_des_enu.z());
        Eigen::Vector3d vel_des_ned(vel_des_enu.y(), vel_des_enu.x(), -vel_des_enu.z());
        Eigen::Vector3d acc_des_ned(acc_des_enu.y(), acc_des_enu.x(), -acc_des_enu.z());
        
        double yaw_des_enu = latest_planning_msg_.yaw;
        double yaw_des_ned = -yaw_des_enu + M_PI_2;

        // 3. 计算误差
        Eigen::Vector3d error_p = pos_des_ned - pos_curr;
        Eigen::Vector3d error_v = vel_des_ned - vel_curr;
        Eigen::Vector3d acc_fb = Kp.cwiseProduct(error_p) + Kv.cwiseProduct(error_v);

        // 4. 计算期望总加速度 (Target Acceleration)
        // target_acc = acc_fb + acc_des_ned - [0,0,g]
        // NED系中，重力是+9.8(向下)，我们要产生向上的力，所以是减去重力向量
        Eigen::Vector3d gravity_vec(0.0, 0.0, g);
        Eigen::Vector3d target_acc = acc_fb + acc_des_ned - gravity_vec;

        // =================================================================
        // [关键保护] 倾角限制 (Tilt Limit)
        // 防止由于避障加速度过大导致机体倾斜过度而掉高
        // =================================================================
        
        // 设定最大倾角
        // 45° causes significant altitude loss during horizontal pursuit
        // (cos(45°)=0.71, only 71% vertical thrust). This leads to z dropping
        // from 1.3 to 0.95, which causes the camera to lose sight of the target.
        // 30° keeps cos(30°)=0.87 vertical thrust, trading horizontal speed for
        // stable altitude and reliable YOLO detections.
        double max_tilt_angle = 30.0 * (M_PI / 180.0); 

        // 分解水平和垂直加速度
        // 在 NED 中，向上的加速度是负值。我们取绝对值看大小。
        // target_acc(2) 通常应该是负数（指向上）。
        double z_acc_up = -target_acc(2); 
        
        // 如果 Z 轴加速度要求太小（甚至变成向下），强制给一个最小向上的力，防止翻车
        if (z_acc_up < 0.1) z_acc_up = 0.1;

        double xy_acc_norm = std::sqrt(target_acc(0)*target_acc(0) + target_acc(1)*target_acc(1));
        double max_xy_acc = z_acc_up * std::tan(max_tilt_angle);

        // 如果水平加速度要求过大，超过了最大倾角限制，进行缩放
        if (xy_acc_norm > max_xy_acc) {
            double scale = max_xy_acc / xy_acc_norm;
            target_acc(0) *= scale;
            target_acc(1) *= scale;
            // 注意：我们只缩减水平分量，保留垂直分量，这样就保住了高度！
        }
        // =================================================================

        // 5. 姿态计算
        // 机体 -Z 轴对齐 target_acc (因为推力是 Body -Z)
        // 所以 机体 +Z 轴对齐 -target_acc
        Eigen::Vector3d z_b_des = -target_acc.normalized();

        Eigen::Vector3d x_c(std::cos(yaw_des_ned), std::sin(yaw_des_ned), 0.0);
        Eigen::Vector3d y_b_des = z_b_des.cross(x_c).normalized();
        Eigen::Vector3d x_b_des = y_b_des.cross(z_b_des);

        Eigen::Matrix3d R_des;
        R_des.col(0) = x_b_des;
        R_des.col(1) = y_b_des;
        R_des.col(2) = z_b_des;

        Eigen::Quaterniond q_des(R_des);
        q_des.normalize();

        // 6. 油门计算 (增加非线性补偿)
        // 原始公式：double thrust_val = (target_acc.norm() / g) * hover_throttle;
        // 改进：当倾斜时，target_acc.norm() 会变得很大，可能会超过 1.0
        double acc_norm = target_acc.norm();
        double thrust_val = (acc_norm / g) * hover_throttle;

        // 极端保护：如果计算出的推力 > 1.0，说明动力不足以维持当前加速度
        // 此时物理上必然掉高。限制在 1.0 虽然能满油门，但还是建议检查 hover_throttle 是否偏小。
        thrust_val = std::max(0.0, std::min(1.0, thrust_val));

        // 7. 发布
        px4_msgs::msg::VehicleAttitudeSetpoint msg;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        
        msg.q_d[0] = q_des.w();
        msg.q_d[1] = q_des.x();
        msg.q_d[2] = q_des.y();
        msg.q_d[3] = q_des.z();

        msg.thrust_body[0] = 0.0;
        msg.thrust_body[1] = 0.0;
        msg.thrust_body[2] = -thrust_val; 

        publisher_attitude_->publish(msg);
    }

    void publish_offboard_control_heartbeat_signal(bool use_attitude_ctrl) {
        px4_msgs::msg::OffboardControlMode msg;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        if (use_attitude_ctrl) {
            msg.position = false; msg.velocity = false; msg.acceleration = false;
            msg.attitude = true;  msg.body_rate = false;
        } else {
            msg.position = true;  msg.velocity = false; msg.acceleration = false;
            msg.attitude = false; msg.body_rate = false;
        }
        publisher_offboard_mode_->publish(msg);
    }

    void publish_vehicle_command(uint16_t command, double p1 = 0.0, double p2 = 0.0) {
        px4_msgs::msg::VehicleCommand msg;
        msg.command = command;
        msg.param1 = p1; msg.param2 = p2;
        msg.target_system = 1; msg.target_component = 1; msg.source_system = 1; msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        publisher_vehicle_command_->publish(msg);
    }

    void publish_position_setpoint(double x, double y, double z, double yaw) {
        px4_msgs::msg::TrajectorySetpoint msg;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.position[0] = x; msg.position[1] = y; msg.position[2] = z; msg.yaw = yaw;
        publisher_trajectory_->publish(msg);
    }

    void cmdloop_callback() {
        if (control_mode_ == "m") {
            publish_offboard_control_heartbeat_signal(false);
            return;
        }
        if (control_mode_ == "t") {
            publish_offboard_control_heartbeat_signal(false);
            if (nav_state_ != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
                publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
            }
            if (arming_state_ != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
                publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
            }
            if (!takeoff_hover_des_set_ && vehicle_local_position_received_) {
                hover_setpoint_.position[0] = vehicle_local_position_.x;
                hover_setpoint_.position[1] = vehicle_local_position_.y;
                // Use odom z for takeoff hover to stay consistent with planning frame
                hover_setpoint_.position[2] = ros_odom_received_ ? std::min(odom_z_ned_, -1.5) : -1.5; 
                hover_setpoint_.yaw = vehicle_local_position_.heading;
                takeoff_hover_des_set_ = true;
                RCLCPP_WARN(this->get_logger(), "Takeoff hover set: z=%.2f", hover_setpoint_.position[2]);
            }
            if(takeoff_hover_des_set_) publish_position_setpoint(hover_setpoint_.position[0], hover_setpoint_.position[1], hover_setpoint_.position[2], hover_setpoint_.yaw);
            return;
        }
        if (control_mode_ == "p" || (control_mode_ == "o" && !planning_pos_command_received_)) {
            publish_offboard_control_heartbeat_signal(false);
            if (!offboard_hover_des_set_ && vehicle_local_position_received_) {
                hover_setpoint_.position[0] = vehicle_local_position_.x;
                hover_setpoint_.position[1] = vehicle_local_position_.y;
                // Use odom z (same source as planning) to avoid z-offset between PX4 EKF2 and LIO
                double hover_z = ros_odom_received_ ? odom_z_ned_ : vehicle_local_position_.z;
                hover_setpoint_.position[2] = std::min(hover_z, -1.0);  // NED: at least 1.0m up
                hover_setpoint_.yaw = vehicle_local_position_.heading;
                offboard_hover_des_set_ = true;
                RCLCPP_WARN(this->get_logger(), "Hover set: z=%.2f (odom_z_ned=%.2f, vlp_z=%.2f)",
                    hover_setpoint_.position[2], odom_z_ned_, vehicle_local_position_.z);
            }
            if(offboard_hover_des_set_) publish_position_setpoint(hover_setpoint_.position[0], hover_setpoint_.position[1], hover_setpoint_.position[2], hover_setpoint_.yaw);
            return;
        }
        if (control_mode_ == "o" && planning_pos_command_received_) {
            double cmd_age = (this->now() - last_planning_cmd_time_).seconds();
            if (cmd_age > CMD_TIMEOUT_SEC) {
                // Planning commands stale — fall back to position hold
                publish_offboard_control_heartbeat_signal(false);
            if (!offboard_hover_des_set_ && vehicle_local_position_received_) {
                hover_setpoint_.position[0] = vehicle_local_position_.x;
                hover_setpoint_.position[1] = vehicle_local_position_.y;
                // Use odom z (same source as planning) to avoid z-offset between PX4 EKF2 and LIO
                double hover_z = ros_odom_received_ ? odom_z_ned_ : vehicle_local_position_.z;
                hover_setpoint_.position[2] = std::min(hover_z, -1.0);  // NED: at least 1.0m up
                hover_setpoint_.yaw = vehicle_local_position_.heading;
                offboard_hover_des_set_ = true;
                RCLCPP_WARN(this->get_logger(), "Planning cmd timeout (%.2fs), holding at z=%.2f (odom_z_ned=%.2f, vlp_z=%.2f)",
                    cmd_age, hover_setpoint_.position[2], odom_z_ned_, vehicle_local_position_.z);
            }
                if (offboard_hover_des_set_) {
                    publish_position_setpoint(hover_setpoint_.position[0], hover_setpoint_.position[1],
                                              hover_setpoint_.position[2], hover_setpoint_.yaw);
                }
                return;
            }
            publish_offboard_control_heartbeat_signal(true);
            if (nav_state_ != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
                publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
            }
            takeoff_hover_des_set_ = false; offboard_hover_des_set_ = false;
            calculate_geometric_control();
            return;
        }
        if (control_mode_ == "l") publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
        if (control_mode_ == "d") publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}
