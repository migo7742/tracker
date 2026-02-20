#include <chrono>
#include <cmath>
#include <memory>
#include <array>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"

using std::placeholders::_1;

// PX4 VehicleOdometry (NED/FRD) → nav_msgs Odometry (ENU/FLU) 转换桥接
//
// 坐标系对照：
//   PX4 NED:  X=North, Y=East,  Z=Down
//   ROS ENU:  X=East,  Y=North, Z=Up
//
//   PX4 FRD body: X=Forward, Y=Right, Z=Down
//   ROS FLU body: X=Forward, Y=Left,  Z=Up
//
// 转换关系：
//   position:  x_enu = y_ned,  y_enu = x_ned,  z_enu = -z_ned
//   quaternion: 通过欧拉角中间转换 (roll 不变, pitch 取反, yaw 补偿 90°)
//   velocity (NED→ENU): vx_enu = vy_ned, vy_enu = vx_ned, vz_enu = -vz_ned
//   velocity (FRD→FLU body): vx_flu = vx_frd, vy_flu = -vy_frd, vz_flu = -vz_frd

class Px4OdomBridge : public rclcpp::Node
{
public:
    Px4OdomBridge() : Node("px4_odom_bridge")
    {
        this->declare_parameter("px4_odom_topic", "/fmu/out/vehicle_odometry");
        this->declare_parameter("ros_odom_topic", "/odom");
        this->declare_parameter("frame_id", "odom");
        this->declare_parameter("child_frame_id", "base_link");

        std::string px4_topic = this->get_parameter("px4_odom_topic").as_string();
        std::string ros_topic = this->get_parameter("ros_odom_topic").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();
        child_frame_id_ = this->get_parameter("child_frame_id").as_string();

        // PX4 micro-XRCE-DDS 使用 BestEffort QoS
        rclcpp::QoS px4_qos(rclcpp::KeepLast(1));
        px4_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        px4_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);

        subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            px4_topic, px4_qos,
            std::bind(&Px4OdomBridge::px4_callback, this, _1));

        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(ros_topic, 50);

        RCLCPP_INFO(this->get_logger(),
                    "PX4 Odom Bridge started: [%s] (PX4 NED) -> [%s] (ROS ENU)",
                    px4_topic.c_str(), ros_topic.c_str());
    }

private:
    void px4_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        nav_msgs::msg::Odometry odom{};

        // --- 1. Header ---
        odom.header.stamp = this->now();
        odom.header.frame_id = frame_id_;
        odom.child_frame_id = child_frame_id_;

        // --- 2. 位置: NED → ENU ---
        odom.pose.pose.position.x = msg->position[1];   // East  = NED_Y
        odom.pose.pose.position.y = msg->position[0];   // North = NED_X
        odom.pose.pose.position.z = -msg->position[2];  // Up    = -NED_Z

        // --- 3. 姿态: NED/FRD quaternion → ENU/FLU quaternion ---
        // PX4 q = [w, x, y, z], 表示 FRD body → NED world
        double qw = msg->q[0];
        double qx = msg->q[1];
        double qy = msg->q[2];
        double qz = msg->q[3];

        if (std::isnan(qw)) {
            odom.pose.pose.orientation.w = 1.0;
            odom.pose.pose.orientation.x = 0.0;
            odom.pose.pose.orientation.y = 0.0;
            odom.pose.pose.orientation.z = 0.0;
        } else {
            // 从 NED/FRD 四元数提取欧拉角
            double roll_ned, pitch_ned, yaw_ned;
            euler_from_quaternion(qx, qy, qz, qw, roll_ned, pitch_ned, yaw_ned);

            // NED→ENU 欧拉角转换：
            //   roll_enu  =  roll_ned   (前进轴不变)
            //   pitch_enu = -pitch_ned  (左右翻转)
            //   yaw_enu   = π/2 - yaw_ned  (NED 0°=North CW → ENU 0°=East CCW)
            double roll_enu = roll_ned;
            double pitch_enu = -pitch_ned;
            double yaw_enu = M_PI / 2.0 - yaw_ned;
            yaw_enu = std::atan2(std::sin(yaw_enu), std::cos(yaw_enu));

            auto q_enu = quaternion_from_euler(roll_enu, pitch_enu, yaw_enu);
            odom.pose.pose.orientation.w = q_enu[0];
            odom.pose.pose.orientation.x = q_enu[1];
            odom.pose.pose.orientation.y = q_enu[2];
            odom.pose.pose.orientation.z = q_enu[3];
        }

        // --- 4. 速度 ---
        if (msg->velocity_frame == px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED) {
            // NED world → ENU world
            odom.twist.twist.linear.x = msg->velocity[1];   // East
            odom.twist.twist.linear.y = msg->velocity[0];   // North
            odom.twist.twist.linear.z = -msg->velocity[2];  // Up
        } else if (msg->velocity_frame == px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD) {
            // FRD body → FLU body
            odom.twist.twist.linear.x = msg->velocity[0];   // Forward
            odom.twist.twist.linear.y = -msg->velocity[1];  // Left
            odom.twist.twist.linear.z = -msg->velocity[2];  // Up
        } else {
            // FRD (默认当作 NED 处理)
            odom.twist.twist.linear.x = msg->velocity[1];
            odom.twist.twist.linear.y = msg->velocity[0];
            odom.twist.twist.linear.z = -msg->velocity[2];
        }

        // --- 5. 角速度: FRD body → FLU body ---
        odom.twist.twist.angular.x = msg->angular_velocity[0];   // Forward 轴不变
        odom.twist.twist.angular.y = -msg->angular_velocity[1];  // Right→Left 取反
        odom.twist.twist.angular.z = -msg->angular_velocity[2];  // Down→Up 取反

        publisher_->publish(odom);
    }

    void euler_from_quaternion(double x, double y, double z, double w,
                               double &roll, double &pitch, double &yaw)
    {
        double t0 = 2.0 * (w * x + y * z);
        double t1 = 1.0 - 2.0 * (x * x + y * y);
        roll = std::atan2(t0, t1);

        double t2 = 2.0 * (w * y - z * x);
        t2 = std::max(-1.0, std::min(1.0, t2));
        pitch = std::asin(t2);

        double t3 = 2.0 * (w * z + x * y);
        double t4 = 1.0 - 2.0 * (y * y + z * z);
        yaw = std::atan2(t3, t4);
    }

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

    std::string frame_id_;
    std::string child_frame_id_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Px4OdomBridge>());
    rclcpp::shutdown();
    return 0;
}
