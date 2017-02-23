/**
* A Proportional Integral Derivative (PID) controller is the most commonly used
* method to control a process.
*
* A PID controller calculates an "error" value as the difference between the
* measured output (the process variable) and the reference input (the setpoint).
* The controller attempts to minimize the error by adjusting the process control
* inputs. The proportional value determines the reaction to the current error,
* the integral value determines the reaction based on the sum of recent errors,
* and the derivative value determines the reaction based on the rate at which
* the error has been changing.
*
* Algorithm
* The PID algorithm has been implemented by computing the Laplace transform of
* the PID expression in the parallel form, and then by computing the Z transform
* of the laplace expression using the backward Euler method. The resulting
* implementation of the discretized PID is known as the "velocity algorithm".
* This implementation has some advantages over simpler discrete PID algorithms,
* such as the elimination of the "reset windup" phenomenon or bumpless parameter
* changes. However, it is more affected by "derivative kicks": great changes of
* the control signal caused by sudden variations of the error (due to noise or
* steep changes of the setpoint). To avoid this problem, the derivative term of
* the implemented PID controller is modified to a LPF filter to make it less
* noisy, avoiding the "derivative kick".
*
* (The discretization of the PID with filtered derivative term has been taken from
* http://controlsystemslab.com/discrete-time-pid-controller-implementation/)
*
* PID implementation:
*    u[k] = -A1/A0*u[k-1] - A2/A0*u[k-2] + B0/A0*e[k] + B1/A0*e[k-1] + B2/A0*e[k-2]
*    B0 = Kp*(1 + N*Ts) + Ki*Ts*(1 + N*Ts) + Kd*N
*    B1 = -(Kp*(2 + N*Ts) + Ki*Ts + 2*Kd*N)
*    B2 = Kp + Kd*N
*    A0 = 1 + N*Ts
*    A1 = -(2 + N*Ts)
*    A2 = 1
*
* where u is the controller output, e is the error, Kp is the proportional gain,
* Ki is the integral gain, Kd the derivative gain, N is the filtered derivative
* constant and Ts is the sample time.
*/

#ifndef DISCRETEPID_H
#define DISCRETEPID_H

#include "Arduino.h"

class DiscretePID {
    public:
        /**
        * \brief        Class constructor, initializes the PID parameters.
        * \param[in]    Kp          The proportional gain.
        * \param[in]    Ki          The integral gain.
        * \param[in]    Kd          The derivative gain.
        * \param[in]    sampleTime  The execution period of the controller (in ms).
        * \Details
        * The constructor first calls setSampleTime function to set the sample
        * time to the passed value. Then calls setGains to compute the derived
        * gains B0, B1, B2, A0, A1, A2 from the passed proportional, integral
        * and derivative gains and the sample time. Finally sets the controller
        * output limits to a pair of predefined values and sets the state
        * variables to all zeros.
        */
        PID(double Kp, double Ki, double Kd, unsigned long sampleTime);
        /**
        * \brief        Executes the PID algorithm.
        * \param[in]    input       The measured value of the variable to control.
        * \param[in]    setpoint    The target value of the variable to control.
        * \return       output      The control signal.
        * \Details
        * The PID algorithm executes only when the elapsedTime variable is equal
        * or greater than the sample time value defined by the user. The PID
        * is implemented using the velocity form with a filtered derivative term.
        * The output computed by the PID algorithm is limited to a maximum and
        * minimum value, which is very useful to generate a PWM control signal.
        */
        double compute(double input, double setpoint);
        /**
        * \brief    Resets the state buffer to zeros.
        * \return   True value to indicate that the state has been reset.
        */
        bool reset();
        /**
        * \brief        Computes the derived gains B0, B1, B2, A0, A1, A2.
        * \param[in]    Kp      The proportional gain.
        * \param[in]    Ki      The integral gain.
        * \param[in]    Kd      The derivative gain.
        * \return       None.
        */
        void setGains(double Kp, double Ki, double Kd);
        /**
        * \brief        Sets the value of the sample time variable Ts.
        * \param[in]    sampleTime  User-defined sample time (in ms).
        * \return       None.
        */
        void setSampleTime(unsigned long sampleTime);
        /**
        * \brief        Sets the maxium and minimum values of controller output.
        * \param[in]    upperLimit  User-defined maximum output value.
        * \param[in]    lowerLimit  User-defined minimum output value.
        * \return       None.
        */
        void setOutputLimits(double upperLimit, double lowerLimit);
    private:
        double _Kp;            // Proportional gain
        double _Ki;            // Integral gain
        double _Kd;            // Derivative gain
        double B0;             // Derived gain B0 = Kp*(1 + N*Ts) + Ki*Ts*(1 + N*Ts) + Kd*N
    	double B1;             // Derived gain B1 = -(Kp*(2 + N*Ts) + Ki*Ts + 2*Kd*N)
    	double B2;             // Derived gain B2 = Kp + Kd*N
    	double A0;             // Derived gain A0 = 1 + N*Ts
    	double A1;             // Derived gain A1 = -(2 + N*Ts)
    	double A2;             // Derived gain A2 = 1
    	double state[4];       // State array of length 4 to store e[k-1], e[k-2], u[k-1] and u[k-2]
	    double Ts;             // Sample time
        double _upperLimit;    // Maximum output value of the controller
        double _lowerLimit;    // Minimum output value of the controller
};

#endif
