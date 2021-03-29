#ifndef SPORT_SOLE_EXTENDEDKALMANFILTER_HPP_
#define SPORT_SOLE_EXTENDEDKALMANFILTER_HPP_

#define SSEKF_ENABLE_ACC_BIAS() 0

#include <fstream>
#include <iostream>
#include <functional>
#include <kalman/ExtendedKalmanFilter.hpp>
#include "SystemModel.hpp"
#include "AccelMeasurementModel.hpp"
#include "GyroMeasurementModel.hpp"
#include "PositionMeasurementModel.hpp"
#include "VelocityMeasurementModel.hpp"
#include "QuatMeasurementModel.hpp"
#include "YawMeasurementModel.hpp"
#include "VAMeasurementModel.hpp"

#include <unsupported/Eigen/MatrixFunctions>

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

    using EKFBase = Kalman::ExtendedKalmanFilter<S>;
    using EKFBase::x;
    using EKFBase::P;

    ExtendedKalmanFilter(T ab_max=0.3): ab_max_(ab_max)
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

    template<class MeasurementModel>
    const S& update( MeasurementModel& m, const typename MeasurementModel::M& z, const std::function<T(T)>& psi = {})
    {
      using Measurement = typename MeasurementModel::M;
      m.updateJacobians( x );

      Measurement residual = z - m.h( x );
      
      // COMPUTE KALMAN GAIN
      // compute innovation covariance
      Kalman::Covariance<Measurement> SS = m.H * P * m.H.transpose() + m.getCovariance();

      if (psi) SS.diagonal() += (residual.unaryExpr(psi)).array().square().matrix();
      
      // compute kalman gain
      Kalman::KalmanGain<S, Measurement> K = P * m.H.transpose() * SS.inverse();
      
      // UPDATE STATE ESTIMATE AND COVARIANCE
      // Update state using computed kalman gain and innovation
      x += K * residual;

      updateSpecialState();

      // Update covariance
      P -= K * m.H * P;
      
      // return updated state estimate
      return this->getState();
    }

    const S& update(const ZA& z)
    {
      return update(amm, z);
    }

    const S& update(const ZG& z)
    {
      return update(gmm, z);
    }

    const S& update(const ZP& z, const std::function<T(T)>& psi = {})
    {
      return update(pmm, z, psi);
    }

    // Robust estimation
    const S& updateHuber(const ZP& z, const std::function<T(T)>& psi, std::ofstream * const ofs = nullptr)
    {
      using M = ZP;
      auto& m = pmm;
      m.updateJacobians( x );

      M residual = z - m.h( x );

      Kalman::SquareMatrix<T, M::RowsAtCompileTime> R_sqrt = m.getCovariance().sqrt();
      Kalman::SquareMatrix<T, S::RowsAtCompileTime> P_sqrt = P.sqrt();
      Kalman::SquareMatrix<T, M::RowsAtCompileTime> R_sqrt_inv = R_sqrt.inverse();
      Kalman::SquareMatrix<T, S::RowsAtCompileTime> P_sqrt_inv = P_sqrt.inverse();

      S x_posterior = x;
      Kalman::KalmanGain<S, M> K;
      Eigen::DiagonalMatrix<T, M::RowsAtCompileTime> Psi_y_inv;
      Eigen::DiagonalMatrix<T, S::RowsAtCompileTime> Psi_x_inv;

      S x_posterior_prev;
      for (int i = 0; i < 6; ++i)
      {
        // State prediction residual
        S x_tilde = x_posterior - x;
        Psi_y_inv = (R_sqrt_inv * ((m.H * x_tilde) - residual)).unaryExpr(psi).asDiagonal().inverse();
        Psi_x_inv = (P_sqrt_inv * x_tilde).unaryExpr(psi).asDiagonal().inverse();
        Kalman::Covariance<M> SS = m.H * P_sqrt * Psi_x_inv * P_sqrt * m.H.transpose() + R_sqrt * Psi_y_inv * R_sqrt;
        K = P_sqrt * Psi_x_inv * P_sqrt * m.H.transpose() * SS.inverse();

        if (ofs) 
        {
          (*ofs) << "x_tilde = [\n" << x_tilde.transpose() << "]\n";
          (*ofs) << "K = [\n" << K.transpose() << "]\n";
        }
        
        x_posterior = x + K * residual;

        // Break if it converged
        if (i == 0) x_posterior_prev = x_posterior;
        else
        {
          if ((x_posterior - x_posterior_prev).maxCoeff() < 1e-3) break;
        }

      }
      x = x_posterior;
      updateSpecialState();
      P = (Kalman::SquareMatrix<T, S::RowsAtCompileTime>::Identity() - K * m.H) * P_sqrt * Psi_x_inv * P_sqrt;
      // return updated state estimate
      return this->getState();
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
      
#if SSEKF_ENABLE_ACC_BIAS()
      T ab_norm = x.ab().squaredNorm();
      if (ab_norm > ab_max_)
      {
        x.template segment<3>(S::ABX) = (ab_max_ / ab_norm) * x.ab();
      }
#endif
    }

    T ab_max_;
    
  };

  
}

#endif
