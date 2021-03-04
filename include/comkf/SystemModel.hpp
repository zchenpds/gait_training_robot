#ifndef COMKF_SYSTEMMODEL_HPP_
#define COMKF_SYSTEMMODEL_HPP_

#include <kalman/LinearizedSystemModel.hpp>
#include <iostream>

namespace comkf
{

template<typename T>
class KalmanFilter;

/**
 * @brief System state vector-type body CoM (Center of Mass)
 *
 * System consists of 4 states
 *
 * @param T Numeric scalar type
 */
template<typename T>
class State : public Kalman::Vector<T, 4>
{
public:
    KALMAN_VECTOR(State, T, 4)
    
    //! X-position
    static constexpr size_t PX = 0;
    //! Y-Position
    static constexpr size_t PY = 1;

    //! X-Velocity
    static constexpr size_t VX = 2;
    //! Y-Velocity
    static constexpr size_t VY = 3;

    // position
    Kalman::Vector<T, 2> p() const { return this->template segment<2>(PX); }
    T px()       const { return (*this)[ PX ]; }
    T py()       const { return (*this)[ PY ]; }
    
    // velocity
    Kalman::Vector<T, 2> v() const { return this->template segment<2>(VX); }
    T vx()       const { return (*this)[ VX ]; }
    T vy()       const { return (*this)[ VY ]; }
    
    T& px()      { return (*this)[ PX ]; }
    T& py()      { return (*this)[ PY ]; }

    T& vx()      { return (*this)[ VX ]; }
    T& vy()      { return (*this)[ VY ]; }

};

/**
 * @brief System control-input for the inverted pendulum model
 *
 * Consists of CoP displacement
 *
 * @param T Numeric scalar type
 */
template<typename T>
class Control : public Kalman::Vector<T, 2>
{
public:
    KALMAN_VECTOR(Control, T, 2)
    
    //! X-CoP
    static constexpr size_t COPX = 0;
    //! Y-CoP
    static constexpr size_t COPY = 1;

    Kalman::Vector<T, 2> cop() const { return this->template segment<2>(COPX); }    
    T copx()       const { return (*this)[ COPX ]; }
    T copy()       const { return (*this)[ COPY ]; }
    
    T& copx()       { return (*this)[ COPX ]; }
    T& copy()       { return (*this)[ COPY ]; }
};

/**
 * @brief System model for a inverted pedlum model
 *
 * The system model
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class SystemModel : public Kalman::LinearizedSystemModel<State<T>, Control<T>, CovarianceBase>
{
    friend class comkf::KalmanFilter<T>;
public:
    //! State type shortcut definition
	typedef State<T> S;
    
    //! Control type shortcut definition
    typedef Control<T> C;

    SystemModel(T dt = T(0.01)) : 
        dt_(dt),
        omega0sq_(10.0)
    {}

    void setSamplingPeriod(T dt) {dt_ = dt;}
    
    /**
     * @brief Definition of (non-linear) state transition function
     *
     * This function defines how the system state is propagated through time,
     * i.e. it defines in which state \f$\hat{x}_{k+1}\f$ is system is expected to 
     * be in time-step \f$k+1\f$ given the current state \f$x_k\f$ in step \f$k\f$ and
     * the system control input \f$u\f$.
     *
     * @param [in] x The system state in current time-step
     * @param [in] u The control vector input
     * @returns The (predicted) system state in the next time-step
     */
    S f(const S& x, const C& u) const
    {

        //! Acceleration in map frame
        Kalman::Vector<T, 2> acc = omega0sq_ * (x.p() - u.cop());

        //! Predicted state vector after transition
        S x_;
        x_.template segment<2>(S::PX) = x.p() + x.v() * dt_;// + 0.5 * acc * dt * dt;
        x_.template segment<2>(S::VX) = x.v() + acc * dt_;
        
        // Return transitioned state vector
        return x_;
    }
protected:
    // Sample period
    T dt_;
    T omega0sq_;
    
    
protected:
    /**
     * @brief Update jacobian matrices for the system state transition function using current state
     *
     * This will re-compute the (state-dependent) elements of the jacobian matrices
     * to linearize the non-linear state transition function \f$f(x,u)\f$ around the
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
    void updateJacobians( const S& x, const C& u )
    {        
        // F = df/dx (Jacobian of state transition w.r.t. the state)
        this->F.setZero();

        this->F.template block<2, 2>(S::PX, S::PX) = Kalman::Matrix<T, 2, 2>::Identity();
        this->F.template block<2, 2>(S::PX, S::VX) = Kalman::Matrix<T, 2, 2>::Identity() * dt_;

        this->F.template block<2, 2>(S::VX, S::VX) = Kalman::Matrix<T, 2, 2>::Identity();
        this->F.template block<2, 2>(S::VX, S::PX) = Kalman::Matrix<T, 2, 2>::Identity() * (omega0sq_ * dt_);
        
        // W = df/dw (Jacobian of state transition w.r.t. the process noise)
        this->W.setIdentity();
    }
};

} // namespace COMKF

#endif