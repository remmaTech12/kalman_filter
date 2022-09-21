#include <gtest/gtest.h>
#include <Eigen/Dense>

#define private public
#include "../example/include/spring_mass_damper.h"


TEST(SpringMassDamperTest, general_performance) {
  SpringMassDamper smd;
  smd.simulate();

  float total_error = 0.0;
  for (int i=0; i<smd.sim_times_; i++) {
    total_error += std::fabs(smd.x_hat_v_[i](0) - smd.x_v_[i](0));
  }
  std::cout << "total abs error between true and estimated values: " << total_error << std::endl;

  EXPECT_EQ(true, total_error < 10.0);
}

TEST(SpringMassDamperTest, rand_m1p1) {
  SpringMassDamper smd;
  float rand_num = smd.get_rand_m1p1();
  std::cout << "random number: " << rand_num << std::endl;

  EXPECT_EQ(true, rand_num >= -1.0 && rand_num <= 1.0);
}

/*
// usually not run because this deletes data.dat file.
TEST(SpringMassDamperTest, save_data_file) {
  if (std::remove("./data.dat") != 0) {
    std::cout << "Could not remove the file!!" << std::endl;
  }

  SpringMassDamper smd;
  smd.simulate();

  bool ret = false;
  std::ifstream file("./data.dat");
  if (file) ret = true;

  EXPECT_EQ(true, ret);
}
*/

TEST(SpringMassDamperTest, rand_nd) {
  int rand_size = 1000;
  SpringMassDamper smd;
  std::vector<float> rand_nd = smd.generate_rand_nd(rand_size);

  int sd_count = 0;
  for (int i=0; i<rand_size; i++) {
    if (rand_nd[i] >= -1.0 && rand_nd[i] <= 1.0) sd_count++;
  }
  float sd_ratio = (float) sd_count / rand_size;
  std::cout << "ratio of the random numbers inside standard deviation: "
            << sd_ratio << std::endl;

  EXPECT_EQ(true, sd_ratio >= 0.6 && sd_ratio <= 0.8);
}
