#include "../include/spring_mass_damper.h"

#include <random>

void SpringMassDamper::initialize() {
  /// physical parameter
  float m = 1.0;
  float k = 1.0;
  float c = 1.0;

  /// system matrices
  Eigen::MatrixXf A(2, 2);
  A << 0.0, 1.0, -k / m, -c / m;
  Eigen::MatrixXf Ad(2, 2);
  Ad = Eigen::MatrixXf::Identity(2, 2) + A * delta_t_;

  Eigen::MatrixXf B(2, 1);
  B << 0, 1 / m;
  Eigen::MatrixXf Bd(2, 1);
  Bd = B * delta_t_;

  Eigen::MatrixXf Cd(1, 2);
  Cd << 1.0, 0.0;

  Eigen::MatrixXf Dd(1, 1);
  Dd << 0.0;

  /// covariance matrices
  float gamma = 0.0;
  Eigen::Matrix2f P0 = gamma * Eigen::Matrix2f::Identity();

  float sigma_v = 1e-2;
  Eigen::MatrixXf Q(1, 1);  // sigma_v^2
  Q << sigma_v * sigma_v;

  float sigma_w = 5.0 * 1e-3;
  // float sigma_w = 5.0 * 1e-6;
  Eigen::MatrixXf R(1, 1);  // sigma_w^2
  R << sigma_w * sigma_w;

  kf_ = std::make_shared<KalmanFilter>(P0, Ad, Bd, Bd, Cd, Dd, Q, R);
  simulate_dynamics(Ad, Bd, Cd, Dd);
}

void SpringMassDamper::simulate_dynamics(const Eigen::MatrixXf& Ad,
                                         const Eigen::MatrixXf& Bd,
                                         const Eigen::MatrixXf& Cd,
                                         const Eigen::MatrixXf& Dd) {
  /// parameter for noise and input
  float sigma_v = 1e-2;
  float sigma_w = 5.0 * 1e-3;
  float u_rand_max = 0.5;

  /// generate random numbers for v and w
  int rand_size = sim_times_ + 1;
  std::vector<float> rand_nd_v = generate_rand_nd(rand_size);
  std::vector<float> rand_nd_w = generate_rand_nd(rand_size);

	/// initialize setting for random number u
  // std::srand(std::time(0));
  std::srand(2);

  // define 1-dimentional vector as follows
  Eigen::Matrix<float, 1, 1> v(sigma_v * rand_nd_v[0]);
  Eigen::Matrix<float, 1, 1> w(sigma_w * rand_nd_w[0]);

  Eigen::Vector2f x(0, 0);
  Eigen::Matrix<float, 1, 1> u(u_rand_max * get_rand_m1p1());
  Eigen::Matrix<float, 1, 1> y = Cd * x + Dd * u + w;
  x_v_.push_back(x);
  u_v_.push_back(u);
  y_v_.push_back(y);

  for (int i = 0; i < sim_times_; i++) {
    /// generate noise
    v(0, 0) = sigma_v * rand_nd_v[i + 1];
    w(0, 0) = sigma_w * rand_nd_w[i + 1];

    /// generate input
    u(0, 0) = u_rand_max * get_rand_m1p1();

    /// calculate states
    x = Ad * x + Bd * u + Bd * v;
    y = Cd * x + Dd * u + w;

    x_v_.push_back(x);
    y_v_.push_back(y);
    u_v_.push_back(u);
  }
}

void SpringMassDamper::estimate_states() {
  Eigen::Vector2f x_hat(0, 0);
  x_hat_v_.push_back(x_hat);

  /// calculate filtered value
  for (int i = 1; i < sim_times_; i++) {
    x_hat = kf_->filter(x_hat, u_v_[i - 1], y_v_[i]);
    x_hat_v_.push_back(x_hat);

    /// for debug
    // std::cout << "x_hat = (" << x_hat(0) << ", " << x_hat(1) << ")" <<
    // std::endl; std::cout << "u =      " << u_v_[i](0) << std::endl; std::cout
    // << "y =      " << y_v_[i](0) << std::endl;
  }
}

void SpringMassDamper::save_data_file() {
  std::ofstream file("./data.dat");
  if (!file) {
    std::cout << "Could not open the file!!" << std::endl;
    return;
  }

  for (int i = 0; i < sim_times_; i++) {
    file << i * delta_t_ << ' ' << y_v_[i](0) << ' ' << x_v_[i](0) << ' '
         << x_v_[i](1) << ' ' << x_hat_v_[i](0) << ' ' << x_hat_v_[i](1) << ' '
         << u_v_[i](0) << std::endl;
  }
  file.close();
}

float SpringMassDamper::get_rand_m1p1() {
  return (static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX)) * 2.0 - 1.0;
}

std::vector<float> SpringMassDamper::generate_rand_nd(int rand_size) {
  std::vector<float> rand_arr(rand_size, 0);

  std::random_device seed_gen;
  std::default_random_engine engine(seed_gen());

  float mean_val = 0.0;
  float cov_val = 1.0;
  std::normal_distribution<> dist(mean_val, cov_val);

  for (int i = 0; i < rand_size; i++) {
    rand_arr[i] = dist(engine);
  }

  return rand_arr;
}
