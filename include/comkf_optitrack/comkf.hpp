#ifndef COMKF_HPP_
#define COMKF_HPP_

#include "SystemModel.hpp"
// #include "PositionMeasurementModel.hpp"
#include "PositionMeasurementModel.hpp"
#include "CopMeasurementModel.hpp"
namespace comkf
{
  template <typename T>
  struct KalmanFilter: public Kalman::ExtendedKalmanFilter<State<T>>
  {
    using S = State<T>;
    using C = Control<T>;
    using ZP = PositionMeasurement<T>;
    using ZCOP = CopMeasurement<T>;

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
    const S& predict(T dt)
    {
      assert(dt > T(0.0));
      sys.setSamplingPeriod(dt);
      const C u_dummy;
      sys.updateJacobians( x, u_dummy );

      // predict state
      x = sys.f(x, u_dummy);

      // predict covariance
      P  = ( sys.F * P * sys.F.transpose() ) + sys.getCovariance() * dt;

      // return state prediction
      return this->getState();
    }
    
    const S& update(const ZP& z)
    {
      return BaseType::update(pmm, z);
    }
    
    const S& update(const ZCOP& z)
    {
      return BaseType::update(copmm, z);
    }

    const S& forceUpdate(const ZCOP& z)
    {
      x.copx() = x.px();
      x.copy() = x.py();

      return BaseType::update(copmm, z);
    }

    SystemModel<T> sys;
    PositionMeasurementModel<T> pmm;
    CopMeasurementModel<T>    copmm;
  };
} // namespace comkf

#endif