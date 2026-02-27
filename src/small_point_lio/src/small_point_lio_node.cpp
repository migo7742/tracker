/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "small_point_lio_node.hpp"
#include "io/pcd_io.h"
#include "lidar_adapter/custom_mid360_driver.h"
#include "lidar_adapter/livox_custom_msg.h"
#include "lidar_adapter/livox_pointcloud2.h"
#include "lidar_adapter/unitree_lidar.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace small_point_lio {

    SmallPointLioNode::SmallPointLioNode(const rclcpp::NodeOptions &options)
        : Node("small_point_lio", options) {
        std::string lidar_topic = declare_parameter<std::string>("lidar_topic");
        std::string imu_topic = declare_parameter<std::string>("imu_topic");
        std::string lidar_type = declare_parameter<std::string>("lidar_type");
        std::string lidar_frame = declare_parameter<std::string>("lidar_frame");
        bool save_pcd = declare_parameter<bool>("save_pcd");
        double z_offset = declare_parameter<double>("z_offset", 0.45);
        small_point_lio = std::make_unique<small_point_lio::SmallPointLio>(*this);
        odometry_publisher = create_publisher<nav_msgs::msg::Odometry>("/Odometry", 1000);
        pointcloud_publisher = create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 1000);
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
        if (save_pcd) {
            pointcloud_mapping = std::make_unique<util::PointcloudMapping>(0.02);
        }
        map_save_trigger = create_service<std_srvs::srv::Trigger>(
                "map_save",
                [this, save_pcd, lidar_frame](const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res) {
                    if (!save_pcd) {
                        res->success = false;
                        res->message = "pcd save is disabled";
                        RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"), "pcd save is disabled");
                        return;
                    }
                    res->success = true;
                    RCLCPP_INFO(rclcpp::get_logger("small_point_lio"), "waiting for pcd saving ...");
                    auto pointcloud_to_save = std::make_shared<std::vector<Eigen::Vector3f>>();
                    *pointcloud_to_save = pointcloud_mapping->get_points();
                    std::thread([pointcloud_to_save, lidar_frame]() {
                        io::pcd::write_pcd(ROOT_DIR + "/pcd/scan.pcd", *pointcloud_to_save);
                        RCLCPP_INFO(rclcpp::get_logger("small_point_lio"), "save pcd success");
                    }).detach();
                });
        small_point_lio->set_odometry_callback([this, lidar_frame,z_offset](const common::Odometry &odometry) {
            last_odometry = odometry;

            builtin_interfaces::msg::Time time_msg;
            time_msg.sec = std::floor(odometry.timestamp);
            time_msg.nanosec = static_cast<uint32_t>((odometry.timestamp - time_msg.sec) * 1e9);

            // 1. 获取 Base -> Lidar 的静态变换 (从 TF 树里查出来的，通常在 URDF 里定义)
            geometry_msgs::msg::TransformStamped base_link_to_lidar_msg;
            try {
                base_link_to_lidar_msg = tf_buffer->lookupTransform(lidar_frame, "base_link", time_msg);
            } catch (tf2::TransformException &ex) {
                RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"), "Failed to lookup transform: %s", ex.what());
                return;
            }

            // 将 msg 转换为 tf2 格式方便计算
            tf2::Transform tf_base_to_lidar;
            tf2::fromMsg(base_link_to_lidar_msg.transform, tf_base_to_lidar);

            // 2. 获取 LIO 算出来的 "原点 -> 当前雷达" 变换
            tf2::Transform tf_odom_origin_to_lidar;
            tf_odom_origin_to_lidar.setOrigin(tf2::Vector3(odometry.position.x(), odometry.position.y(), odometry.position.z()));
            tf_odom_origin_to_lidar.setRotation(tf2::Quaternion(odometry.orientation.x(), odometry.orientation.y(), odometry.orientation.z(), odometry.orientation.w()));

            // ================= 修改核心开始 =================
            
            // 3. 高度补偿 (Z轴偏移)
            // 目的：把 LIO 的原点从“雷达高度”下移到“地面高度”。
            // 这一步做完，tf_odom_to_lidar 就变成了 "地面 -> 当前雷达"
            tf2::Vector3 origin = tf_odom_origin_to_lidar.getOrigin();
            origin.setZ(origin.z() + z_offset); 
            tf_odom_origin_to_lidar.setOrigin(origin);

            // 4. 坐标系链式计算：求 "地面(odom) -> 车体(base_link)"
            // 公式：T_odom_base = T_odom_lidar * T_lidar_base
            // 因为我们手里有的是 base->lidar，所以要取逆 (inverse)
            tf2::Transform tf_odom_to_base = tf_odom_origin_to_lidar * tf_base_to_lidar.inverse();

            // ================= 修改核心结束 =================

            // 5. 准备发布 TF 消息
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped.header.stamp = time_msg;
            transform_stamped.header.frame_id = "odom";
            transform_stamped.child_frame_id = "base_link";
            transform_stamped.transform = tf2::toMsg(tf_odom_to_base);
            
            // 发布 TF
            tf_broadcaster->sendTransform(transform_stamped);

            // 6. 准备发布 Odometry 消息
            nav_msgs::msg::Odometry odometry_msg;
            odometry_msg.header.stamp = time_msg;
            odometry_msg.header.frame_id = "odom";
            odometry_msg.child_frame_id = "base_link";
            
            // 【重点】直接使用上面算好的 tf_odom_to_base 的值
            // 不要再手动 + 0.45 了，因为第3步已经在源头加过了！
            odometry_msg.pose.pose.position.x = transform_stamped.transform.translation.x;
            odometry_msg.pose.pose.position.y = transform_stamped.transform.translation.y;
            odometry_msg.pose.pose.position.z = transform_stamped.transform.translation.z; 
            odometry_msg.pose.pose.orientation = transform_stamped.transform.rotation;

            // 7. 填充 twist（ESKF velocity 是 odom 世界系，planning_node 也直接按 odom 系使用）
            odometry_msg.twist.twist.linear.x = odometry.velocity.x();
            odometry_msg.twist.twist.linear.y = odometry.velocity.y();
            odometry_msg.twist.twist.linear.z = odometry.velocity.z();
            // angular_velocity 是 body 系，符合 ROS twist 约定
            odometry_msg.twist.twist.angular.x = odometry.angular_velocity.x();
            odometry_msg.twist.twist.angular.y = odometry.angular_velocity.y();
            odometry_msg.twist.twist.angular.z = odometry.angular_velocity.z();

            odometry_publisher->publish(odometry_msg);
        });
        small_point_lio->set_pointcloud_callback([this, save_pcd, lidar_frame](const std::vector<Eigen::Vector3f> &pointcloud) {
            if (pointcloud_publisher->get_subscription_count() > 0) {
                builtin_interfaces::msg::Time time_msg;
                time_msg.sec = std::floor(last_odometry.timestamp);
                time_msg.nanosec = static_cast<uint32_t>((last_odometry.timestamp - time_msg.sec) * 1e9);

                geometry_msgs::msg::TransformStamped lidar_frame_to_base_link_transform;
                try {
                    lidar_frame_to_base_link_transform = tf_buffer->lookupTransform("base_link", lidar_frame, time_msg);
                } catch (tf2::TransformException &ex) {
                    RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"), "Failed to lookup transform from %s to base_link: %s", lidar_frame.c_str(), ex.what());
                    return;
                }
                Eigen::Vector3f lidar_frame_to_base_link_T;
                lidar_frame_to_base_link_T << static_cast<float>(lidar_frame_to_base_link_transform.transform.translation.x),
                        static_cast<float>(lidar_frame_to_base_link_transform.transform.translation.y),
                        static_cast<float>(lidar_frame_to_base_link_transform.transform.translation.z);
                Eigen::Matrix3f lidar_frame_to_base_link_R =
                        Eigen::Quaternionf(
                                static_cast<float>(lidar_frame_to_base_link_transform.transform.rotation.w),
                                static_cast<float>(lidar_frame_to_base_link_transform.transform.rotation.x),
                                static_cast<float>(lidar_frame_to_base_link_transform.transform.rotation.y),
                                static_cast<float>(lidar_frame_to_base_link_transform.transform.rotation.z))
                                .toRotationMatrix();
                sensor_msgs::msg::PointCloud2 msg;
                msg.header.stamp = time_msg;
                msg.header.frame_id = "odom";
                msg.width = pointcloud.size();
                msg.height = 1;
                msg.fields.reserve(4);
                sensor_msgs::msg::PointField field;
                field.name = "x";
                field.offset = 0;
                field.datatype = sensor_msgs::msg::PointField::FLOAT32;
                field.count = 1;
                msg.fields.push_back(field);
                field.name = "y";
                field.offset = 4;
                field.datatype = sensor_msgs::msg::PointField::FLOAT32;
                field.count = 1;
                msg.fields.push_back(field);
                field.name = "z";
                field.offset = 8;
                field.datatype = sensor_msgs::msg::PointField::FLOAT32;
                field.count = 1;
                msg.fields.push_back(field);
                field.name = "intensity";
                field.offset = 12;
                field.datatype = sensor_msgs::msg::PointField::FLOAT32;
                field.count = 1;
                msg.fields.push_back(field);
                msg.is_bigendian = false;
                msg.point_step = 16;
                msg.row_step = msg.width * msg.point_step;
                msg.data.resize(msg.row_step * msg.height);
                Eigen::Vector3f transformed_point;
                auto pointer = reinterpret_cast<float *>(msg.data.data());
                for (const auto &point: pointcloud) {
                    transformed_point = lidar_frame_to_base_link_R * point + lidar_frame_to_base_link_T;
                    *pointer = transformed_point.x();
                    ++pointer;
                    *pointer = transformed_point.y();
                    ++pointer;
                    *pointer = transformed_point.z();
                    ++pointer;
                    *pointer = 0;
                    ++pointer;
                }
                msg.is_dense = false;
                pointcloud_publisher->publish(msg);
            }
            if (save_pcd) {
                for (const auto &point: pointcloud) {
                    pointcloud_mapping->add_point(point);
                }
            }
        });
        if (lidar_type == "livox_custom_msg") {
#ifdef HAVE_LIVOX_DRIVER
            lidar_adapter = std::make_unique<LivoxCustomMsgAdapter>();
#else
            RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"), "livox_custom_msg requested but not available!");
            rclcpp::shutdown();
            return;
#endif
        } else if (lidar_type == "livox_pointcloud2") {
            lidar_adapter = std::make_unique<LivoxPointCloud2Adapter>();
        } else if (lidar_type == "custom_mid360_driver") {
            lidar_adapter = std::make_unique<CustomMid360DriverAdapter>();
        } else if (lidar_type == "unilidar") {
            lidar_adapter = std::make_unique<UnilidarAdapter>();
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"), "unknwon lidar type");
            rclcpp::shutdown();
            return;
        }
        lidar_adapter->setup_subscription(this, lidar_topic, [this](const std::vector<common::Point> &pointcloud) {
            small_point_lio->on_point_cloud_callback(pointcloud);
            small_point_lio->handle_once();
        });
        imu_subsciber = create_subscription<sensor_msgs::msg::Imu>(
                imu_topic,
                rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::Imu &msg) {
                    common::ImuMsg imu_msg;
                    imu_msg.angular_velocity = Eigen::Vector3d(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
                    imu_msg.linear_acceleration = Eigen::Vector3d(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
                    imu_msg.timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
                    small_point_lio->on_imu_callback(imu_msg);
                    small_point_lio->handle_once();
                });
    }

}// namespace small_point_lio

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(small_point_lio::SmallPointLioNode)
