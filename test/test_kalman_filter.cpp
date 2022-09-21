#include <gtest/gtest.h>
#include <Eigen/Dense>

#define private public
#include "../include/kalman_filter.h"

KalmanFilter default_kf(void) {
  Eigen::MatrixXf A(2, 2);
  A << 0.0, 1.0, -1.0, -1.0;
  Eigen::MatrixXf B(2, 1);
  B << 0, 1.0;
  Eigen::MatrixXf C(1, 2);
  C << 1.0, 0.0;
  Eigen::MatrixXf D(1, 1);
  D << 0.0;

  Eigen::Matrix2f P0 = Eigen::Matrix2f::Identity();

  Eigen::MatrixXf Q(1, 1);
  Q << 0.05;
  Eigen::MatrixXf R(1, 1);
  R << 0.05;
  KalmanFilter kf(P0, A, B, B, C, D, Q, R);

  return kf;
}

TEST(KalmanFilterTest, system_matrix_consistent) {
  KalmanFilter kf = default_kf();

  EXPECT_EQ(true, kf.warn_system_matrix_inconsistency());
}

TEST(KalmanFilterTest, system_matrix_inconsistent) {
  Eigen::MatrixXf A(1, 2);  // test would fail here.
  A << 0.0, 1.0;
  Eigen::MatrixXf B(2, 1);
  B << 0, 1.0;
  Eigen::MatrixXf C(1, 2);
  C << 1.0, 0.0;
  Eigen::MatrixXf D(1, 1);
  D << 0.0;

  Eigen::Matrix2f P0 = Eigen::Matrix2f::Identity();

  Eigen::MatrixXf Q(1, 1);
  Q << 0.05;
  Eigen::MatrixXf R(1, 1);
  R << 0.05;
  KalmanFilter kf(P0, A, B, B, C, D, Q, R);

  EXPECT_EQ(false, kf.warn_system_matrix_inconsistency());
}

TEST(KalmanFilterTest, matrix_vector_consistent) {
  KalmanFilter kf = default_kf();
  Eigen::Vector2f x;
  x << 0, 0;
  Eigen::Matrix<float, 1, 1> y;
  y << 0;
  Eigen::Matrix<float, 1, 1> u;
  u << 0;

  EXPECT_EQ(true, kf.check_filter_matrix_vector_size(x, u, y));
}

TEST(KalmanFilterTest, matrix_vector_inconsistent) {
  KalmanFilter kf = default_kf();
  Eigen::Vector3f x;
  x << 0, 0, 0;
  Eigen::Matrix<float, 1, 1> y;
  y << 0;
  Eigen::Matrix<float, 1, 1> u;
  u << 0;

  EXPECT_EQ(false, kf.check_filter_matrix_vector_size(x, u, y));
}

TEST(KalmanFilterTest, filter_update) {
  KalmanFilter kf = default_kf();
  Eigen::Vector2f x;
  x << 0, 0;
  Eigen::Matrix<float, 1, 1> y;
  y << 1.0;
  Eigen::Matrix<float, 1, 1> u;
  u << 0;

  Eigen::VectorXf fx = kf.filter(x, u, y);
  EXPECT_EQ(false, x == fx);
}

TEST(KalmanFilterTest, filter_update_zero) {
  KalmanFilter kf = default_kf();
  Eigen::Vector2f x;
  x << 0, 0;
  Eigen::Matrix<float, 1, 1> y;
  y << 0;
  Eigen::Matrix<float, 1, 1> u;
  u << 0;

  Eigen::VectorXf fx = kf.filter(x, u, y);
  EXPECT_EQ(true, x == fx);
}

TEST(KalmanFilterTest, filter_no_update) {
  KalmanFilter kf = default_kf();
  // inconsistency of matrix and vector sizes
  Eigen::Vector3f x;
  x << 0, 0, 0;
  Eigen::Matrix<float, 1, 1> y;
  y << 1.0;
  Eigen::Matrix<float, 1, 1> u;
  u << 0;

  Eigen::VectorXf fx = kf.filter(x, u, y);
  EXPECT_EQ(true, x == fx);
}
