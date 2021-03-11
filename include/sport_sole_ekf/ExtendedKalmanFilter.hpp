#ifndef SPORT_SOLE_EXTENDEDKALMANFILTER_HPP_
#define SPORT_SOLE_EXTENDEDKALMANFILTER_HPP_

#include <iostream>
#include <kalman/ExtendedKalmanFilter.hpp>
#include "SystemModel.hpp"
#include "AccelMeasurementModel.hpp"
#include "GyroMeasurementModel.hpp"
#include "PositionMeasurementModel.hpp"
#include "VelocityMeasurementModel.hpp"
#include "QuatMeasurementModel.hpp"
#include "YawMeasurementModel.hpp"
#include "VAMeasurementModel.hpp"

namespace sport_sole
{
  template<typename T>
  struct ExtendedKalmanFilter : public Kalman::ExtendedKalmanFilter<State<T>>
  {
    SystemModel<T> sys;
    AccelMeasurementModel<T> amm;
    GyroMeasurementModel<T> gmm;
    PositionMeasurementModel<T> pmm;
    VelocityMeasurementModel<T> vmm;
    QuatMeasurementModel<T> qmm;
    YawMeasurementModel<T> ymm;
    VAMeasurementModel<T> vamm;

    typedef State<T> S;
    typedef Control<T> C;
    typedef AccelMeasurement<T> ZA;
    typedef GyroMeasurement<T> ZG;
    typedef PositionMeasurement<T> ZP;
    typedef VelocityMeasurement<T> ZV;
    typedef QuatMeasurement<T> ZQ;
    typedef YawMeasurement<T> ZY;
    typedef VAMeasurement<T> ZVA;

    using Kalman::ExtendedKalmanFilter<S>::x;
    using Kalman::ExtendedKalmanFilter<S>::P;

    ExtendedKalmanFilter()
    {
      x.setZero();
      x.q0() = 1.0;
    }

    const S& predict( T dt )
    {
      return predict( sys, dt );
    }
    
    const S& predict( SystemModel<T>& s, T dt )
    {
      // predict state (without control)
      C u;
      u.setZero();
      return predict( s, u, dt );
    }
    
    const S& predict( SystemModel<T>& s, const C& u, T dt )
    {
      s.setTimestep(dt);
      s.updateJacobians( x, u );
      
      // predict state
      x = s.f(x, u);
      
      // predict covariance
      P  = s.F * P * s.F.transpose() + s.getCovariance() * s.dt;
      
      // return state prediction
      return this->getState();
    }

    using Kalman::ExtendedKalmanFilter<S>::update;
    const S& update(const ZA& z)
    {
      return update(amm, z);
    }

    const S& update(const ZG& z)
    {
      return update(gmm, z);
    }

    const S& update(const ZP& z)
    {
      return update(pmm, z);
    }

    const S& update(const ZV& z)
    {
      return update(vmm, z);
    }

    const S& update(const ZQ& z)
    {
      return update(qmm, z);
    }
    
    const S& update(const ZY& z)
    {
      return update(ymm, z);
    }

    const S& update(const ZVA& z)
    {
      return update(vamm, z);
    }

    void repairQuaternion()
    {
      // repair quaternion
      x.template segment<4>(S::Q0).normalize();
      if (x.q0() < 0) 
        x.template segment<4>(S::Q0) *= -1;
    }

    bool initialized = false;

    void reducePVA()
    {
      for (auto xyz : {0, 1, 2})
      {
        P(S::PX + xyz, S::VX + xyz) /= 1e5;
        P(S::VX + xyz, S::PX + xyz) /= 1e5;
        P(S::PX + xyz, S::AX + xyz) /= 1e5;
        P(S::AX + xyz, S::PX + xyz) /= 1e5;
      }
    }

    void printP()
    {
      return;
      std::cout << "P =\n" << P << "\n\n";
    }

  protected:
    void updateSpecialState() override
    {
      repairQuaternion();
    }
    
  };

  
}

#endif
