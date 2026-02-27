#include <chrono>
#include <cmath>
#include <memory>
#include <array>
#include <limits>
#include <algorithm> // for std::fill

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"

using std::placeholders::_1;

class VisualOdomBridge : public rclcpp::Node
{
public:
    VisualOdomBridge() : Node("visual_odom_bridge")
    {
        // QoS 配置 (完全对应 Python: BestEffort, TransientLocal, KeepLast, Depth=1)
        // 这种写法比 rmw_qos_profile_t 更现代、更符合 C++ 风格
        rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
        qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos_profile.durability(rclcpp::DurabilityPolicy::TransientLocal);

        // 创建订阅者 (ROS Odometry 通常频率较高，depth 设为 10 以防丢包)
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry",
            10,
            std::bind(&VisualOdomBridge::listener_callback, this, _1));

        // 创建发布者
        publisher_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
            "/fmu/in/vehicle_visual_odometry",
            qos_profile);

        RCLCPP_INFO(this->get_logger(), "Visual Odom Bridge (SITL Mode) Started [C++ Optimized]");
    }

private:
    void listener_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        px4_msgs::msg::VehicleOdometry vehicle_odom{};

        // --- 1. 时间戳 ---
        auto now = this->get_clock()->now();
        uint64_t timestamp_us = now.nanoseconds() / 1000;
        vehicle_odom.timestamp = timestamp_us;
        vehicle_odom.timestamp_sample = timestamp_us;

        // --- 2. 位置转换 (ROS ENU -> PX4 NED) ---
        // ROS X(东) -> PX4 Y(东)
        // ROS Y(北) -> PX4 X(北)
        // ROS Z(上) -> PX4 -Z(下)
        // static_cast 消除 double->float 警告
        vehicle_odom.position[0] = static_cast<float>(msg->pose.pose.position.y);  // North
        vehicle_odom.position[1] = static_cast<float>(msg->pose.pose.position.x);  // East
        vehicle_odom.position[2] = static_cast<float>(-msg->pose.pose.position.z); // Down

        // --- 3. 姿态转换 ---
        double roll, pitch, yaw;
        const auto& q_ros = msg->pose.pose.orientation;
        
        // 提取欧拉角 (ROS 坐标系)
        euler_from_quaternion(q_ros.x, q_ros.y, q_ros.z, q_ros.w, roll, pitch, yaw);

        // 偏航角转换逻辑:
        // 1. 取反 (逆时针 -> 顺时针)
        // 2. 加 90度 (ROS 0度指向东，PX4 0度指向北)
        double yaw_px4 = -yaw + (M_PI / 2.0);

        // 使用 atan2 进行鲁棒的归一化 (-PI ~ PI)
        yaw_px4 = std::atan2(std::sin(yaw_px4), std::cos(yaw_px4));

        // Roll/Pitch 转换: Roll不变, Pitch取反
        // 重新生成 PX4 所需的四元数 (NED)
        auto q_px4 = quaternion_from_euler(roll, -pitch, yaw_px4);
        
        vehicle_odom.q[0] = static_cast<float>(q_px4[0]); // W
        vehicle_odom.q[1] = static_cast<float>(q_px4[1]); // X
        vehicle_odom.q[2] = static_cast<float>(q_px4[2]); // Y
        vehicle_odom.q[3] = static_cast<float>(q_px4[3]); // Z

        // --- 4. 速度转换 (FLU -> FRD) ---
        // 假设 Twist 是机体坐标系 (Child Frame = base_link)
        vehicle_odom.velocity[0] = static_cast<float>(msg->twist.twist.linear.x);
        vehicle_odom.velocity[1] = static_cast<float>(-msg->twist.twist.linear.y);
        vehicle_odom.velocity[2] = static_cast<float>(-msg->twist.twist.linear.z);

        vehicle_odom.angular_velocity[0] = static_cast<float>(msg->twist.twist.angular.x);
        vehicle_odom.angular_velocity[1] = static_cast<float>(-msg->twist.twist.angular.y);
        vehicle_odom.angular_velocity[2] = static_cast<float>(-msg->twist.twist.angular.z);

        vehicle_odom.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD;

        // --- 5. 坐标系与方差 ---
        vehicle_odom.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;

        // 填充 NaN (表示方差未知，PX4 将使用参数中的默认值 EKF2_EV_POS_X 等)
        const float nan_val = std::numeric_limits<float>::quiet_NaN();
        std::fill(std::begin(vehicle_odom.position_variance), std::end(vehicle_odom.position_variance), nan_val);
        std::fill(std::begin(vehicle_odom.orientation_variance), std::end(vehicle_odom.orientation_variance), nan_val);
        std::fill(std::begin(vehicle_odom.velocity_variance), std::end(vehicle_odom.velocity_variance), nan_val);

        publisher_->publish(vehicle_odom);
    }

    // 辅助函数：四元数转欧拉角 (ZYX 顺序)
    void euler_from_quaternion(double x, double y, double z, double w, double &roll, double &pitch, double &yaw)
    {
        // Roll (x-axis rotation)
        double t0 = 2.0 * (w * x + y * z);
        double t1 = 1.0 - 2.0 * (x * x + y * y);
        roll = std::atan2(t0, t1);

        // Pitch (y-axis rotation)
        double t2 = 2.0 * (w * y - z * x);
        // 限制范围，防止 asin 越界
        t2 = std::max(-1.0, std::min(1.0, t2));
        pitch = std::asin(t2);

        // Yaw (z-axis rotation)
        double t3 = 2.0 * (w * z + x * y);
        double t4 = 1.0 - 2.0 * (y * y + z * z);
        yaw = std::atan2(t3, t4);
    }

    // 辅助函数：欧拉角转四元数
    std::array<double, 4> quaternion_from_euler(double roll, double pitch, double yaw)
    {
        double cy = std::cos(yaw * 0.5);
        double sy = std::sin(yaw * 0.5);
        double cp = std::cos(pitch * 0.5);
        double sp = std::sin(pitch * 0.5);
        double cr = std::cos(roll * 0.5);
        double sr = std::sin(roll * 0.5);

        std::array<double, 4> q;
        q[0] = cr * cp * cy + sr * sp * sy; // w
        q[1] = sr * cp * cy - cr * sp * sy; // x
        q[2] = cr * sp * cy + sr * cp * sy; // y
        q[3] = cr * cp * sy - sr * sp * cy; // z
        return q;
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualOdomBridge>());
    rclcpp::shutdown();
    return 0;
}
