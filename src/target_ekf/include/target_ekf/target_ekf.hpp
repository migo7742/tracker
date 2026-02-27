#pragma once

#include <cmath> // 引入数学库，用于 sin, cos, atan2, M_PI
#include <Eigen/Geometry>
#include <Eigen/Dense>

// 添加 inline 防止 "multiple definition" 链接错误
inline Eigen::Quaterniond euler2quaternion(const Eigen::Vector3d& euler) {
  // 注意：这里的 euler 顺序取决于你的定义，通常 Eigen 处理的是弧度
  double cr = std::cos(euler(0) / 2);
  double sr = std::sin(euler(0) / 2);
  double cp = std::cos(euler(1) / 2);
  double sp = std::sin(euler(1) / 2);
  double cy = std::cos(euler(2) / 2);
  double sy = std::sin(euler(2) / 2);
  Eigen::Quaterniond q;
  q.w() = cr * cp * cy + sr * sp * sy;
  q.x() = sr * cp * cy - cr * sp * sy;
  q.y() = cr * sp * cy + sr * cp * sy;
  q.z() = cr * cp * sy - sr * sp * cy;
  return q;
}

// 添加 inline 防止 "multiple definition" 链接错误
inline Eigen::Vector3d quaternion2euler(const Eigen::Quaterniond& q) {
  Eigen::Matrix3d m = q.toRotationMatrix();
  Eigen::Vector3d rpy;
  // 这里使用的是 Z-Y-X 顺序 (RPY) 的解算公式
  rpy.x() = std::atan2(m(2, 1), m(2, 2)); // Roll
  rpy.y() = std::asin(-m(2, 0));          // Pitch
  rpy.z() = std::atan2(m(1, 0), m(0, 0)); // Yaw
  return rpy;
}

struct Ekf {
  double dt;
  Eigen::MatrixXd A, B, C;
  Eigen::MatrixXd Qt, Rt;
  Eigen::MatrixXd Sigma, K;
  Eigen::VectorXd x;

  // states: x, y, z, vx, vy, vz, roll, pitch, yaw
  // Total states: 9

  Ekf(double _dt) : dt(_dt) {
    A.setIdentity(9, 9);
    Sigma.setZero(9, 9);
    B.setZero(9, 6);
    C.setZero(6, 9);
    
    // Constant Velocity Model for position
    A(0, 3) = dt;
    A(1, 4) = dt;
    A(2, 5) = dt;
    
    double t2 = dt * dt / 2;
    // Noise mapping
    B(0, 0) = t2;
    B(1, 1) = t2;
    B(2, 2) = t2;
    B(3, 0) = dt;
    B(4, 1) = dt;
    B(5, 2) = dt;
    B(6, 3) = dt;
    B(7, 4) = dt;
    B(8, 5) = dt;
    
    // Measurement matrix: checking x,y,z and roll,pitch,yaw?
    // Based on Qt logic below, measurements seem to be 6D: [x, y, z, r, p, y]
    C(0, 0) = 1;
    C(1, 1) = 1;
    C(2, 2) = 1;
    C(3, 6) = 1;
    C(4, 7) = 1;
    C(5, 8) = 1;
    
    K = C; // Just initializing size
    
    Qt.setIdentity(6, 6);
    Rt.setIdentity(6, 6);
    
    // Process Noise Covariance
    Qt(0, 0) = 4;    // x
    Qt(1, 1) = 4;    // y
    Qt(2, 2) = 1;    // z
    Qt(3, 3) = 1;    // roll
    Qt(4, 4) = 1;    // pitch
    Qt(5, 5) = 0.1;  // yaw
    
    // Measurement Noise Covariance
    Rt(0, 0) = 0.1;
    Rt(1, 1) = 0.1;
    Rt(2, 2) = 0.1;
    Rt(3, 3) = 0.01;
    Rt(4, 4) = 0.01;
    Rt(5, 5) = 0.01;
    
    x.setZero(9);
  }

  inline void predict() {
    x = A * x;
    Sigma = A * Sigma * A.transpose() + B * Qt * B.transpose();
  }

  inline void reset(const Eigen::Vector3d& z, const Eigen::Vector3d& z_rpy) {
    x.setZero();
    x.head(3) = z;
    x.tail(3) = z_rpy;
    Sigma.setZero();
  }

  inline bool update(const Eigen::Vector3d& z, const Eigen::Vector3d& z_rqp) {
    // 1. Calculate Kalman Gain
    K = Sigma * C.transpose() * (C * Sigma * C.transpose() + Rt).inverse();
    
    // 2. Prepare measurement vector
    Eigen::VectorXd zz(6);
    zz.head(3) = z;
    zz.tail(3) = z_rqp;
    
    // 3. Outlier rejection / Validation check
    Eigen::VectorXd x_tmp = x + K * (zz - C * x);
    static double vmax = 4;
    // Checking velocity magnitude (indices 3,4,5 in state vector)
    if (x_tmp.middleRows(3, 3).norm() > vmax) {
      return false;
    }

    // 4. Angle Wrapping Logic
    // Ensure the state angle is close to the measurement angle to avoid +/- 2PI jumps
    // This modifies the current state 'x' before the update step, which is a common trick
    Eigen::Vector3d d_rpy = x.tail(3) - z_rqp;
    
    // Helper lambda or manual unwrapping
    auto unwrap = [](double& angle, double diff) {
        if (diff > M_PI) angle -= 2 * M_PI;
        else if (diff < -M_PI) angle += 2 * M_PI;
    };

    unwrap(x.tail(3).x(), d_rpy.x());
    unwrap(x.tail(3).y(), d_rpy.y());
    unwrap(x.tail(3).z(), d_rpy.z());

    // 5. Final Update
    x = x + K * (zz - C * x);
    Sigma = Sigma - K * C * Sigma;
    
    return true;
  }

  inline const Eigen::Vector3d pos() const {
    return x.head(3);
  }

  inline const Eigen::Vector3d vel() const {
    // Velocity is indices 3, 4, 5
    return x.middleRows(3, 3);
  }

  inline const Eigen::Vector3d rpy() const {
    return x.tail(3);
  }
};
