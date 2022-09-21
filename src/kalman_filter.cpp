#include "../include/kalman_filter.h"

bool KalmanFilter::check_filter_matrix_vector_size(
    const Eigen::VectorXf& pre_x_hat, const Eigen::VectorXf& pre_u,
    const Eigen::VectorXf& cur_y) {
  /// error handling for size inconsistency
  if (A_.cols() != pre_x_hat.size()) {
    std::cerr << "Error: A matrix and x vector sizes are inconsistent!"
              << std::endl;
    return false;
  }
  if (C_.cols() != pre_x_hat.size()) {
    std::cerr << "Error: C matrix and x vector sizes are inconsistent!"
              << std::endl;
    return false;
  }
  if (B_.cols() != pre_u.size()) {
    std::cerr << "Error: B matrix and u vector sizes are inconsistent!"
              << std::endl;
    return false;
  }

  if (pre_x_hat.size() != (A_ * pre_x_hat).rows()) {
    std::cerr << "Error: x_hat vector and Ax sizes are inconsistent!"
              << std::endl;
    return false;
  }
  if (pre_x_hat.size() != (B_ * pre_u).rows()) {
    std::cerr << "Error: x_hat vector and Bu sizes are inconsistent!"
              << std::endl;
    return false;
  }
  if (cur_y.size() != (C_ * pre_x_hat).rows()) {
    std::cerr << "Error: y vector and Cx vector sizes are inconsistent!"
              << std::endl;
    return false;
  }

  return true;
}

Eigen::VectorXf KalmanFilter::filter(const Eigen::VectorXf& pre_x_hat,
                                     const Eigen::VectorXf& pre_u,
                                     const Eigen::VectorXf& cur_y) {
  if (!check_filter_matrix_vector_size(pre_x_hat, pre_u, cur_y))
    return pre_x_hat;

  /// prediction step
  Eigen::VectorXf x_hat_ps = A_ * pre_x_hat + B_ * pre_u;
  Eigen::MatrixXf P_ps = A_ * P_ * A_.transpose() + Bv_ * Q_ * Bv_.transpose();

  /// filtering step
  Eigen::MatrixXf G =
      P_ps * C_.transpose() * (C_ * P_ps * C_.transpose() + R_).inverse();
  Eigen::VectorXf new_x_hat = x_hat_ps + G * (cur_y - C_ * x_hat_ps);
  Eigen::MatrixXf I = Eigen::MatrixXf::Identity(A_.cols(), A_.cols());
  P_ = (I - G * C_) * P_ps;

  return new_x_hat;
}
