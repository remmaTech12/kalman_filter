#include <iostream>
#include <Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::MatrixXf;
int main()
{
  MatrixXf m = MatrixXf::Identity(2,2);
  /*
  m(0,0) = 3;
  m(1,0) = 2.5;
  m(0,1) = -1;
  m(1,1) = m(1,0) + m(0,1);
  */
  std::cout << m << std::endl;
}