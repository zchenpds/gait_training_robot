#ifndef COMKF_HPP_
#define COMKF_HPP_

#include <fstream>
#include <unsupported/Eigen/MatrixFunctions>
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
      // updateSpecialState();
      P = (Kalman::SquareMatrix<T, S::RowsAtCompileTime>::Identity() - K * m.H) * P_sqrt * Psi_x_inv * P_sqrt;
      // return updated state estimate
      return this->getState();
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

    void updateMeasurementScheme(int scheme)
    {
      if (scheme == 1)
      {
        pmm.H.template block<2, 2>(ZP::PX, S::BX).setIdentity();
        copmm.H.template block<2, 2>(ZCOP::PX, S::BX).setZero();
      }
      else if (scheme == 2)
      {
        pmm.H.template block<2, 2>(ZP::PX, S::BX).setZero();
        copmm.H.template block<2, 2>(ZCOP::PX, S::BX).setIdentity();
      }
      else if (scheme == 3)
      {
        pmm.H.template block<2, 2>(ZP::PX, S::BX).setZero();
        copmm.H.template block<2, 2>(ZCOP::PX, S::BX).setZero();
      }
    }

    SystemModel<T> sys;
    PositionMeasurementModel<T> pmm;
    CopMeasurementModel<T>    copmm;
  };
} // namespace comkf

#endif