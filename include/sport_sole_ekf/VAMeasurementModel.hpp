#ifndef SPORT_SOLE_VAMEASUREMENTMODEL_HPP_
#define SPORT_SOLE_VAMEASUREMENTMODEL_HPP_

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
class VAMeasurement : public Kalman::Vector<T, 6>
{
public:
    KALMAN_VECTOR(VAMeasurement, T, 6)
    
    static constexpr size_t VX = 0;
    static constexpr size_t VY = 1;
    static constexpr size_t VZ = 2;
    
    static constexpr size_t AX = 3;
    static constexpr size_t AY = 4;
    static constexpr size_t AZ = 5;
    
    T vx() const { return (*this)[ VX ]; }
    T vy() const { return (*this)[ VY ]; }
    T vz() const { return (*this)[ VZ ]; }
    
    T ax() const { return (*this)[ AX ]; }
    T ay() const { return (*this)[ AY ]; }
    T az() const { return (*this)[ AZ ]; }
    
    T& vx()      { return (*this)[ VX ]; }
    T& vy()      { return (*this)[ VY ]; }
    T& vz()      { return (*this)[ VZ ]; }
    
    T& ax()      { return (*this)[ AX ]; }
    T& ay()      { return (*this)[ AY ]; }
    T& az()      { return (*this)[ AZ ]; }
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
class VAMeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, VAMeasurement<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef  sport_sole::State<T> S;
    
    //! Measurement type shortcut definition
    typedef  sport_sole::VAMeasurement<T> M;
    
    /**
     * @brief Constructor
     */
    VAMeasurementModel()
    {

        // Linear measurement model
        this->H.setZero();
        this->H.template block<3, 3>(M::VX, S::VX) = this->H.template block<3, 3>(M::AX, S::AX) = Kalman::SquareMatrix<T, 3>::Identity();

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
        measurement << x.v(), x.a();
        
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

} // namespace sport_sole

#endif