#ifndef SPORT_SOLE_YAWMEASUREMENTMODEL_HPP_
#define SPORT_SOLE_YAWMEASUREMENTMODEL_HPP_

#include <kalman/LinearizedMeasurementModel.hpp>
#include <sport_sole_ekf/SystemModel.hpp>

namespace sport_sole
{

/**
 * @brief Measurement vector measuring the sport sole acceleration with sport sole Kinect
 *
 * @param T Numeric scalar type
 */
template<typename T>
class YawMeasurement : public Kalman::Vector<T, 3>
{
public:
    KALMAN_VECTOR(YawMeasurement, T, 3)
    
    static constexpr size_t X = 0;
    static constexpr size_t Y = 1;
    static constexpr size_t Z = 2;
    
    T x() const { return (*this)[ X ]; }
    T y() const { return (*this)[ Y ]; }
    T z() const { return (*this)[ Z ]; }
    
    T& x()      { return (*this)[ X ]; }
    T& y()      { return (*this)[ Y ]; }
    T& z()      { return (*this)[ Z ]; }
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
class YawMeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, YawMeasurement<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef  sport_sole::State<T> S;
    
    //! Measurement type shortcut definition
    typedef  sport_sole::YawMeasurement<T> M;
    
    /**
     * @brief Constructor
     */
    YawMeasurementModel()
    {   
        // Nonlinear measurement model
        this->H.setZero();

        // Nonlinear measurement model
        this->H.setZero();

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
        const T& q0 = x.q0();
        const T& q1 = x.q1();
        const T& q2 = x.q2();
        const T& q3 = x.q3();
        T q0sq = q0 * q0;
        T q1sq = q1 * q1;
        T q2sq = q2 * q2;
        T q3sq = q3 * q3;

        M measurement;
        measurement << 
            2*q0*q3 + 2*q1*q2,
            q0sq - q1sq + q2sq - q3sq,
            2*q2*q3 - 2*q0*q1;
        return measurement;
    }

protected:
    
    /**
     * @brief Update jacobian matrices for the system state transition function using current state.
     *
     * @param x The current system state around which to linearize
     */
    void updateJacobians( const S& x )
    {
        const T& q0 = x.q0();
        const T& q1 = x.q1();
        const T& q2 = x.q2();
        const T& q3 = x.q3();

        this->H.template block<3, 4>(M::X, S::Q0) <<
             2*q3,  2*q2,  2*q1,  2*q0,
             2*q0, -2*q1,  2*q2, -2*q3,
            -2*q2, -2*q0,  2*q3,  2*q2;
    }

};

} // namespace sport_sole

#endif