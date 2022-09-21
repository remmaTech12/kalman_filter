#pragma once
#include <bits/stdc++.h>
#include <Eigen/Dense>

#include "../../include/kalman_filter.h"

/**
 * @brief Example of state estimation for a spring-mass-damper system utilizing
 * Kalman filter
 */
class SpringMassDamper {
 private:
  std::shared_ptr<KalmanFilter> kf_;

  std::vector<Eigen::VectorXf> x_v_;
  std::vector<Eigen::VectorXf> x_hat_v_;
  std::vector<Eigen::VectorXf> y_v_;
  std::vector<Eigen::VectorXf> u_v_;

  std::vector<Eigen::VectorXf> v_v_;
  std::vector<Eigen::VectorXf> w_v_;
  float delta_t_ = 0.01;
  float sim_duration_ = 10.0;
  int sim_times_ = sim_duration_ / delta_t_;

  /**
   * @brief Initialize system matrices and generate Kalman filter object.
   * @return void
   */
  void initialize();

  /**
   * @brief Simulate dynamics with system matrices, states, inputs and random
   * noises.
   * @param[in] Ad system matrix
   * @param[in] Bd system matrix
   * @param[in] Cd system matrix
   * @param[in] Dd system matrix
   * @return void
   */
  void simulate_dynamics(const Eigen::MatrixXf& Ad, const Eigen::MatrixXf& Bd,
                         const Eigen::MatrixXf& Cd, const Eigen::MatrixXf& Dd);

  /**
   * @brief Estimate states with Kalman filter.
   * @return void
   */
  void estimate_states();

  /**
   * @brief Save states, input and output data for graph plot
   * @return void
   */
  void save_data_file();

  /**
   * @brief Get a random number from -1 to +1
   * @return float random number from -1 to +1
   */
  float get_rand_m1p1();

  /**
   * @brief Generate normally distributed random numbers
   * @param[in] rand_size size of random number array
   * @return std::vector<float> return generated random number array
   */
  std::vector<float> generate_rand_nd(int rand_size);

 public:
  SpringMassDamper() = default;
  virtual ~SpringMassDamper() = default;

  /**
   * @brief API for public: Simulate dynamics, estimate states with Kalman
   * filter and output data file.
   * @return void
   */
  void simulate() {
    initialize();
    estimate_states();
    save_data_file();
  }
};
