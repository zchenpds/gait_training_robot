#ifndef SPORT_SOLE_QUATMEASUREMENTMODEL_HPP_
#define SPORT_SOLE_QUATMEASUREMENTMODEL_HPP_

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
class QuatMeasurement : public Kalman::Vector<T, 4>
{
public:
    KALMAN_VECTOR(QuatMeasurement, T, 4)
    
    static constexpr size_t Q0 = 0;
    static constexpr size_t Q1 = 1;
    static constexpr size_t Q2 = 2;
    static constexpr size_t Q3 = 3;
    
    T q0() const { return (*this)[ Q0 ]; }
    T q1() const { return (*this)[ Q1 ]; }
    T q2() const { return (*this)[ Q2 ]; }
    T q3() const { return (*this)[ Q3 ]; }
    
    T& q0()      { return (*this)[ Q0 ]; }
    T& q1()      { return (*this)[ Q1 ]; }
    T& q2()      { return (*this)[ Q2 ]; }
    T& q3()      { return (*this)[ Q3 ]; }
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
class QuatMeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, QuatMeasurement<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef  sport_sole::State<T> S;
    
    //! Measurement type shortcut definition
    typedef  sport_sole::QuatMeasurement<T> M;
    
    /**
     * @brief Constructor
     */
    QuatMeasurementModel()
    {   
        // Nonlinear measurement model
        this->H.setZero();

        this->H.template block<4, 4>(M::Q0, S::Q0).setIdentity();

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
        M measurement;
        measurement << x.q0(), x.q1(), x.q2(), x.q3();
        return measurement;
    }

protected:
    
    /**
     * @brief Update jacobian matrices for the system state transition function using current state.
     *
     * @param x The current system state around which to linearize
     */
    void updateJacobians( const S& x ) {}

};

} // namespace sport_sole

#endif