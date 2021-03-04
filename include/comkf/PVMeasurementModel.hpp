#ifndef COMKF_PVMEASUREMENTMODEL_HPP_
#define COMKF_PVMEASUREMENTMODEL_HPP_

#include <kalman/LinearizedMeasurementModel.hpp>
#include "SystemModel.hpp"

namespace comkf
{
/**
 * @brief Measurement vector measuring the body CoM and CoMv
 *
 * @param T Numeric scalar type
 */
template<typename T>
class PVMeasurement : public Kalman::Vector<T, 4>
{
public:
    KALMAN_VECTOR(PVMeasurement, T, 4)
    
    // X-position
    static constexpr size_t PX = 0;
    // Y-position
    static constexpr size_t PY = 1;

    // X-velocity
    static constexpr size_t VX = 2;
    // Y-velocity
    static constexpr size_t VY = 3;

    Kalman::Vector<T, 2> p() const { return this->template segment<2>(PX); }    
    T px()       const { return (*this)[ PX ]; }
    T py()       const { return (*this)[ PY ]; }
    
    T& px()      { return (*this)[ PX ]; }
    T& py()      { return (*this)[ PY ]; }

    Kalman::Vector<T, 2> v() const { return this->template segment<2>(VX); }    
    T vx()       const { return (*this)[ VX ]; }
    T vy()       const { return (*this)[ VY ]; }
    
    T& vx()      { return (*this)[ VX ]; }
    T& vy()      { return (*this)[ VY ]; }
};

/**
 * @brief Measurement model
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class PVMeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, PVMeasurement<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef  State<T> S;
    
    //! PVMeasurement type shortcut definition
    typedef  PVMeasurement<T> M;
    
    /**
     * @brief Constructor
     *
     */
    PVMeasurementModel()
    {        
        // Setup noise jacobian. As this one is static, we can define it once
        // and do not need to update it dynamically
        this->V.setIdentity();
        //this->V *= pow(0.05, 2.0);

        // H = dh/dx (Jacobian of measurement function w.r.t. the state)
        //this->H.setZero();
        this->H.template block<4, 4>(M::PX, S::PX).setIdentity();
        this->H.template block<2, 2>(M::PX, S::BX) = -Kalman::Matrix<T, 2, 2>::Identity();
    }
    
    /**
     * @brief Definition of (possibly non-linear) measurement function
     *
     * This function maps the system state to the measurement that is expected
     * to be received from the sensor assuming the system is currently in the
     * estimated state.
     *
     * @param [in] x The system state in current time-step
     * @returns The (predicted) sensor measurement for the system state
     */
    M h(const S& x) const
    {
        M measurement;
        measurement.template segment<2>(M::PX) = x.p() - x.b();
        measurement.template segment<2>(M::VX) = x.v();
        
        return measurement;
    }

protected:
    
    /**
     * @brief Update jacobian matrices for the measurement function using current state
     *
     * This will re-compute the (state-dependent) elements of the jacobian matrices
     * to linearize the non-linear measurement function \f$h(x)\f$ around the
     * current state \f$x\f$.
     *
     * @note This is only needed when implementing a LinearizedSystemModel,
     *       for usage with an ExtendedKalmanFilter or SquareRootExtendedKalmanFilter.
     *       When using a fully non-linear filter such as the UnscentedKalmanFilter
     *       or its square-root form then this is not needed.
     *
     * @param x The current system state around which to linearize
     * @param u The current system control input
     */
    void updateJacobians( const S& x )
    {}
};

} // namespace comkf

#endif