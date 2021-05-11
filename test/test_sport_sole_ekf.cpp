#include "sport_sole_ekf/ExtendedKalmanFilter.hpp"
#include "sport_sole_ekf/SystemModel.hpp"
#include "sport_sole_ekf/AccelMeasurementModel.hpp"
#include <iostream>
using namespace sport_sole;

int main()
{
  AccelMeasurementModel<double> amm;
  State<double> x;

  x.template segment<4>(0) << 0.921061, 0.3894183, 0, 0;
  std::cout << "q =\n" << x.q() << std::endl;
  std::cout << "h(x) =\n" << amm.h(x) << std::endl;
  
  x.template segment<4>(0) << 0.921061, 0, 0.3894183, 0;
  std::cout << "q =\n" << x.q() << std::endl;
  std::cout << "h(x) =\n" << amm.h(x) << std::endl;
  
  x.template segment<4>(0) << 0.921061, 0, 0, 0.3894183;
  std::cout << "q =\n" << x.q() << std::endl;
  std::cout << "h(x) =\n" << amm.h(x) << std::endl;

  return 0;
}