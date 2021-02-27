#ifndef SPORT_SOLE_SYSTEMMODEL_HPP_
#define SPORT_SOLE_SYSTEMMODEL_HPP_

#include <kalman/LinearizedSystemModel.hpp>

namespace sport_sole
{

template<class T>
class ExtendedKalmanFilter;

constexpr size_t STATE_DIM = 22;
/**
 * @brief System state vector-type for a 6DOF sport_sole
 *
 * @param T Numeric scalar type
 */
template<typename T>
class State : public Kalman::Vector<T, STATE_DIM>
{
public:
    KALMAN_VECTOR(State, T, STATE_DIM)

    static constexpr size_t Q0 = 0;
    static constexpr size_t Q1 = 1;
    static constexpr size_t Q2 = 2;
    static constexpr size_t Q3 = 3;

    static constexpr size_t WX = 4;
    static constexpr size_t WY = 5;
    static constexpr size_t WZ = 6;

    static constexpr size_t PX = 7;
    static constexpr size_t PY = 8;
    static constexpr size_t PZ = 9;

    static constexpr size_t VX = 10;
    static constexpr size_t VY = 11;
    static constexpr size_t VZ = 12;

    static constexpr size_t AX = 13;
    static constexpr size_t AY = 14;
    static constexpr size_t AZ = 15;

    static constexpr size_t ABX = 16;
    static constexpr size_t ABY = 17;
    static constexpr size_t ABZ = 18;

    static constexpr size_t WBX = 19;
    static constexpr size_t WBY = 20;
    static constexpr size_t WBZ = 21;
    
    Kalman::Vector<T, 4> q() const { return this->template segment<4>(Q0); }
    T q0()   const { return (*this)[ Q0 ]; }
    T q1()   const { return (*this)[ Q1 ]; }
    T q2()   const { return (*this)[ Q2 ]; }
    T q3()   const { return (*this)[ Q3 ]; }
    
    Kalman::Vector<T, 3> w() const { return this->template segment<3>(WX); }
    T wx()   const { return (*this)[ WX ]; }
    T wy()   const { return (*this)[ WY ]; }
    T wz()   const { return (*this)[ WZ ]; }
    
    Kalman::Vector<T, 3> p() const { return this->template segment<3>(PX); }
    T px()   const { return (*this)[ PX ]; }
    T py()   const { return (*this)[ PY ]; }
    T pz()   const { return (*this)[ PZ ]; }
    
    Kalman::Vector<T, 3> v() const { return this->template segment<3>(VX); }
    T vx()   const { return (*this)[ VX ]; }
    T vy()   const { return (*this)[ VY ]; }
    T vz()   const { return (*this)[ VZ ]; }
    
    Kalman::Vector<T, 3> a() const { return this->template segment<3>(AX); }
    T ax()   const { return (*this)[ AX ]; }
    T ay()   const { return (*this)[ AY ]; }
    T az()   const { return (*this)[ AZ ]; }
    
    Kalman::Vector<T, 3> ab() const { return this->template segment<3>(ABX); }
    T abx()  const { return (*this)[ ABX ]; }
    T aby()  const { return (*this)[ ABY ]; }
    T abz()  const { return (*this)[ ABZ ]; }
    
    Kalman::Vector<T, 3> wb() const { return this->template segment<3>(WBX); }
    T wbx()  const { return (*this)[ WBX ]; }
    T wby()  const { return (*this)[ WBY ]; }
    T wbz()  const { return (*this)[ WBZ ]; }
    
    T& q0()   { return (*this)[ Q0 ]; }
    T& q1()   { return (*this)[ Q1 ]; }
    T& q2()   { return (*this)[ Q2 ]; }
    T& q3()   { return (*this)[ Q3 ]; }
    
    T& wx()   { return (*this)[ WX ]; }
    T& wy()   { return (*this)[ WY ]; }
    T& wz()   { return (*this)[ WZ ]; }
    
    T& px()   { return (*this)[ PX ]; }
    T& py()   { return (*this)[ PY ]; }
    T& pz()   { return (*this)[ PZ ]; }
    
    T& vx()   { return (*this)[ VX ]; }
    T& vy()   { return (*this)[ VY ]; }
    T& vz()   { return (*this)[ VZ ]; }
    
    T& ax()   { return (*this)[ AX ]; }
    T& ay()   { return (*this)[ AY ]; }
    T& az()   { return (*this)[ AZ ]; }
    
    T& abx()  { return (*this)[ ABX ]; }
    T& aby()  { return (*this)[ ABY ]; }
    T& abz()  { return (*this)[ ABZ ]; }
    
    T& wbx()  { return (*this)[ WBX ]; }
    T& wby()  { return (*this)[ WBY ]; }
    T& wbz()  { return (*this)[ WBZ ]; }
};

/**
 * @brief System control-input vector-type for a 6DOF
 * 
 * Should have been 0-dimensional 
 * 
 * @param T Numeric scalar type
 */
constexpr size_t CONTROL_DIM = 1;

template<typename T>
class Control : public Kalman::Vector<T, CONTROL_DIM>
{
public:
    KALMAN_VECTOR(Control, T, CONTROL_DIM)
};

/**
 * @brief System model for a 6DOF sport_sole
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class SystemModel : public Kalman::LinearizedSystemModel<State<T>, Control<T>, CovarianceBase>
{
    friend class sport_sole::ExtendedKalmanFilter<T>;
public:
    //! State type shortcut definition
	typedef sport_sole::State<T> S;
    
    //! Control type shortcut definition
    typedef sport_sole::Control<T> C;

    // Constructor
    SystemModel(T dt = T(0.005)) : 
        dt(dt)
    {}

    void setTimestep(const T dt)
    {
        this->dt = dt;
    }
    
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
        //! Predicted state vector after transition
        S x_ = x;
        
        x_.q0() += (- x.q1() * x.wx() - x.q2() * x.wy() - x.q3() * x.wz()) / 2 * dt;
        x_.q1() += (  x.q0() * x.wx() - x.q3() * x.wy() + x.q2() * x.wz()) / 2 * dt;
        x_.q2() += (  x.q3() * x.wx() + x.q0() * x.wy() - x.q1() * x.wz()) / 2 * dt;
        x_.q3() += (  x.q1() * x.wy() - x.q2() * x.wx() + x.q0() * x.wz()) / 2 * dt;

        // repair quaternion
        x_.template segment<4>(S::Q0).normalize();
        if (x_.q0() < 0) x_.template segment<4>(S::Q0) *= -1;

        x_.vx() += x.ax() * dt;
        x_.vy() += x.ay() * dt;
        x_.vz() += x.az() * dt;

        x_.px() += x.vx() * dt;
        x_.py() += x.vy() * dt;
        x_.pz() += x.vz() * dt;
        
        // Return transitioned state vector
        return x_;
    }
    
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
        this->F.setIdentity();

        Kalman::Matrix<T, 4, 4> Fq;
        Fq <<      0, -x.wx(), -x.wy(), -x.wz(),
              x.wx(),       0,  x.wz(), -x.wy(),
              x.wy(), -x.wz(),       0,  x.wx(),
              x.wz(),  x.wy(), -x.wx(),       0;
        this->F.template block<4, 4>(S::Q0, S::Q0) += Fq / 2 * dt;

        Kalman::Matrix<T, 4, 3> Fw;
        Fw << -x.q1(), -x.q2(), -x.q3(),
               x.q0(), -x.q3(),  x.q2(),
               x.q3(),  x.q0(), -x.q1(),
              -x.q2(),  x.q1(),  x.q0();
        this->F.template block<4, 3>(S::Q0, S::WX) += Fw / 2 * dt;

        this->F.template block<3, 3>(S::PX, S::VX) += Kalman::Matrix<T, 3, 3>::Identity() * dt;
        this->F.template block<3, 3>(S::VX, S::AX) += Kalman::Matrix<T, 3, 3>::Identity() * dt;
        
        // W = df/dw (Jacobian of state transition w.r.t. the noise)
        this->W.setIdentity();
        // TODO: more sophisticated noise modelling
        //       i.e. The noise affects the the direction in which we move as 
        //       well as the velocity (i.e. the distance we move)
    }

protected:
    // Sample period
    T dt;
};

} // namespace sport_sole

#endif