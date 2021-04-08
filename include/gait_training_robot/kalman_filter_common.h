#ifndef KALMAN_FILTER_COMMON_H
#define KALMAN_FILTER_COMMON_H

#include <array>
#include <ctime>
#include <iomanip>
#include <kalman/ExtendedKalmanFilter.hpp>

/**
 * @brief Function template for setting the covariance matrix of different models
 * 
 * @param StateType The vector-type of the system state (usually some type derived from Kalman::Vector)
 * @param model Various models derived from base Kalman::StandardBase<StateType> such as: 
 *        SystemModel, PositionModel, and OrientationModel.
 * @param var_vec The elements on the diagonal of the covariance matrix.
 */
template<class StateType>
void setModelCovariance(Kalman::StandardBase<StateType>& model, 
  const std::array<typename StateType::Scalar, StateType::RowsAtCompileTime>& var_vec)
{
  auto cov = decltype(model.getCovariance()){};
  cov.setZero();
  for (size_t i = 0; i < StateType::RowsAtCompileTime; i++)
    cov(i, i) = var_vec[i];
  model.setCovariance(cov);
}

template<class StateType>
void setModelCovariance(Kalman::StandardBase<StateType>& model, typename StateType::Scalar sigma)
{
  auto cov = decltype(model.getCovariance()){};
  cov.setZero();
  for (size_t i = 0; i < StateType::RowsAtCompileTime; i++)
    cov(i, i) = sigma;
  model.setCovariance(cov);
}


template<class VectorType>
struct Averager {
  VectorType sum;
  size_t num;
  Averager() {sum.setZero(); num = 0;}
  void clear() {sum.setZero(); num = 0;}
  void put(const VectorType & v) { sum += v; num++;}
  VectorType getAverage() {assert(num); return sum / num;}
};


std::string generateDateString()
{
  std::time_t t = std::time(nullptr);
  auto timeinfo = std::localtime (&t);

  std::ostringstream oss;

  oss << std::put_time(timeinfo, "%F-%H-%M-%S");
  return oss.str();
}

template<class VectorType, typename T>
inline void assignVector3(VectorType & v1, const Eigen::Matrix<T, 3, 1>& v2)
{
  v1.x = v2(0);
  v1.y = v2(1);
  v1.z = v2(2);
}

template<class QuaternionType, typename T>
inline void assignQuaternion(QuaternionType & q1, Eigen::Quaternion<T> q2)
{
  q1.x = q2.x();
  q1.y = q2.y();
  q1.z = q2.z();
  q1.w = q2.w();
}

// Convert quaternion from that in geometry_msgs to that in Eigen
template<typename T>
inline Eigen::Quaternion<T> toEigenQuaternion(const geometry_msgs::Quaternion& q)
{
  return Eigen::Quaternion<T>(q.w, q.x, q.y, q.z);
}

template<typename T>
inline Eigen::Quaternion<T> getImuQuaternion(geometry_msgs::Quaternion q)
{
  return toEigenQuaternion<T>(q);
}

#endif