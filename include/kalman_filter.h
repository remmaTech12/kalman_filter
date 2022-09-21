#pragma once
#include <bits/stdc++.h>
#include <Eigen/Dense>

/**
 * @brief Kalman Filter for Linear Systems
 */
class KalmanFilter {
 private:
  Eigen::MatrixXf P_;

  Eigen::MatrixXf A_;
  Eigen::MatrixXf B_;
  Eigen::MatrixXf Bv_;
  Eigen::MatrixXf C_;
  Eigen::MatrixXf D_;

  Eigen::MatrixXf Q_;
  Eigen::MatrixXf R_;

  /**
   * @brief Warn if system matrices sizes are inconsistent.
   * @return bool false if any system matrices sizes are inconsistent, otherwise
   * true
   */
  bool warn_system_matrix_inconsistency() {
    const int n = A_.rows();
    const int r = B_.cols();
    const int p = C_.rows();

    if (A_.size() != P_.size()) {
      std::cout << "A and P size is inconsistent!" << std::endl;
      return false;
    }
    if (B_.size() != Bv_.size()) {
      std::cout << "B and Bv size is inconsistent!" << std::endl;
      return false;
    }

    if (n != A_.cols()) {
      std::cout << "A is not a square matrix!" << std::endl;
      return false;
    }
    if (n != B_.rows()) {
      std::cout << "A and B row size is inconsistent!" << std::endl;
      return false;
    }
    if (n != C_.cols()) {
      std::cout << "A and C collumn size is inconsistent!" << std::endl;
      return false;
    }
    if (p != D_.rows()) {
      std::cout << "C and D row size is inconsistent!" << std::endl;
      return false;
    }
    if (r != D_.cols()) {
      std::cout << "B and D collumn size is inconsistent!" << std::endl;
      return false;
    }

    if (r != Q_.rows() || r != Q_.cols()) {
      std::cout << "Q is not a rxr sized matrix!" << std::endl;
      return false;
    }
    if (p != R_.rows() || p != R_.cols()) {
      std::cout << "R is not a pxp sized matrix!" << std::endl;
      return false;
    }

    return true;
  }

  /**
   * @brief Check if the sizes of matrices and vectors for filtering are
   * consistent.
   * @param[in] pre_x_hat previous x_hat vector
   * @param[in] pre_u previous input vector
   * @param[in] cur_y current measurement vector
   * @return bool whether the sizes are consistent
   */
  bool check_filter_matrix_vector_size(const Eigen::VectorXf& pre_x_hat,
                                       const Eigen::VectorXf& pre_u,
                                       const Eigen::VectorXf& cur_y);

 public:
  KalmanFilter(const Eigen::MatrixXf& P0, const Eigen::MatrixXf& A,
               const Eigen::MatrixXf& B, const Eigen::MatrixXf& Bv,
               const Eigen::MatrixXf& C, const Eigen::MatrixXf& D,
               const Eigen::MatrixXf& Q, const Eigen::MatrixXf& R) {
    P_ = P0;

    A_ = A;
    B_ = B;
    Bv_ = Bv;
    C_ = C;
    D_ = D;

    Q_ = Q;  // covariance matrix for v
    R_ = R;  // covariance matrix for w

    warn_system_matrix_inconsistency();
  }

  virtual ~KalmanFilter() = default;

  /**
   * @brief filtering process of Kalman filter
   * @param[in] pre_x_hat previous x_hat vector
   * @param[in] pre_u previous input vector
   * @param[in] cur_y current measurement vector
   * @return VectorXf current x_hat vector: estimated state
   */
  Eigen::VectorXf filter(const Eigen::VectorXf& pre_x_hat,
                         const Eigen::VectorXf& pre_u,
                         const Eigen::VectorXf& cur_y);
};
