#ifndef COMKF_HPP_
#define COMKF_HPP_

#include "SystemModel.hpp"
#include "PVMeasurementModel.hpp"

namespace comkf
{
  template <typename T>
  struct KalmanFilter: public Kalman::ExtendedKalmanFilter<State<T>>
  {
    using S = State<T>;
    using C = Control<T>;
    using ZPV = PVMeasurement<T>;

    using BaseType = Kalman::ExtendedKalmanFilter<S>;

    using BaseType::x;
    using BaseType::P;

    
    KalmanFilter()
    {
      // Set initial state
      x.setZero();
    }
    
    /**
     * @brief Predict the state with customized step length
     * 
     * @param u The control input vector.
     * @param dt The step length. Will be ignored if negative.
     */
    const S& predict(const C & u, T dt)
    {
      assert(dt > T(0.0));
      sys.setSamplingPeriod(dt);
      sys.updateJacobians( x, u );

      // predict state
      x = sys.f(x, u);

      // predict covariance
      P  = ( sys.F * P * sys.F.transpose() ) + sys.getCovariance() * dt;

      // return state prediction
      return this->getState();
    }
    
    const S& update(const ZPV& zpv)
    {
      return BaseType::update(pvmm, zpv);
    }

    SystemModel<T> sys;
    PVMeasurementModel<T> pvmm;
  };
} // namespace comkf

#endif