#ifndef KPEKF_VELOCITYMEASUREMENTMODEL_HPP_
#define KPEKF_VELOCITYMEASUREMENTMODEL_HPP_

#include <kalman/LinearizedMeasurementModel.hpp>
#include <kpekf/SystemModel.hpp>

namespace kpekf
{

template<typename T>
class ExtendedKalmanFilter;

/**
 * @brief Measurement vector measuring the sport sole IMU position with Kinect
 *
 * @param T Numeric scalar type
 */
template<typename T>
class VelocityMeasurement : public Kalman::Vector<T, 3>
{
    friend class ExtendedKalmanFilter<T>;

public:
    KALMAN_VECTOR(VelocityMeasurement, T, 3)
    
    static constexpr size_t X = 0;
    static constexpr size_t Y = 1;
    static constexpr size_t Z = 2;
    
    T  x() const { return (*this)[ X ]; }
    T  y() const { return (*this)[ Y ]; }
    T  z() const { return (*this)[ Z ]; }
    
    T& x()       { return (*this)[ X ]; }
    T& y()       { return (*this)[ Y ]; }
    T& z()       { return (*this)[ Z ]; }
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
class VelocityMeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, VelocityMeasurement<T>, CovarianceBase>
{
    friend class ExtendedKalmanFilter<T>;

public:
    //! State type shortcut definition
    typedef  kpekf::State<T> S;
    
    //! Measurement type shortcut definition
    typedef  kpekf::VelocityMeasurement<T> M;
    
    /**
     * @brief Constructor
     */
    VelocityMeasurementModel()
    {   
        // Linear measurement model
        this->H.setZero();

        this->H.template block<3, 3>(M::X, S::VX).setIdentity();

        // Setup noise jacobian. As this one is static, we can define it once
        // and do not need to update it dynamically
        this->V.setIdentity();
    }
    
    /**
     * @brief Definition of (possibly non-linear) measurement function
     * 
     * @param [in] x The system state in current time-step
     * @returns The (predicted) sensor measurement for the system state
     */
    M h(const S& x) const
    {
        M measurement = x.template segment<3>(S::VX);
        return measurement;
    }

protected:
    
    /**
     * @brief Update jacobian matrices for the system state transition function using current stateed.
     *
     * @param x The current system state around which to linearize
     */
    void updateJacobians( const S& x ) {}
};

} // namespace kpekf

#endif