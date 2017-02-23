#include "DiscretePID.h"

#define N 100   // The filter coefficient of the filtered derivative term

PID::PID(double Kp, double Ki, double Kd, unsigned long sampleTime) {
    PID::setSampleTime(sampleTime);
    PID::setGains(Kp, Ki, Kd);
    PID::setOutputLimits(0, 255);   // Default limits to generate a 8 bit resolution PWM control signal
    memset(state, 0, 4u * sizeof(double));
}

double PID::compute(double input, double setpoint) {
    static unsigned long previousTime = 0;
    unsigned long currentTime = micros();
    unsigned long elapsedTime = currentTime - previousTime;
    if(elapsedTime >= Ts) {
        double error = setpoint - input;

        /* u[k] = -A1/A0*u[k-1] - A2/A0*u[k-2] + B0/A0*e[k] + B1/A0*e[k-1] + B2/A0*e[k-2]  */
        double output = -A1/A0*state[2] - A2/A0*state[3] + B0/A0*error + B1/A0*state[0] + B2/A0*state[1];

        /* Anti-windup */
        if(output > _upperLimit) {
            output = _upperLimit;
        }
        else if(ouput < _lowerLimit) {
            output = _lowerLimit;
        }

        /* Update state */
        state[1] = state[0];  // e[k-2]
        state[0] = error;     // e[k-1]
        state[3] = state[2];  // u[k-2]
        state[2] = output;    // u[k-1]

        /* Adding the interval instead of using the value of currentTime is
        a better way to ensure that succesive periods are identical */
        previousTime += Ts;

        return output;
    }
}

bool PID::reset() {
    /* Clear the state buffer. The size will be always 4 samples */
    memset(state, 0, 4u * sizeof(double));
    return true;
}

void PID::setGains(double Kp, double Ki, double Kd) {
    /* Derived coefficient B0 */
    B0 = Kp*(1+N*Ts) + Ki*Ts*(1+N*Ts) + Kd*N;
    /* Derived coefficient B1 */
    B1 = -(Kp*(2+N*Ts) + Ki*Ts + 2*Kd*N);
    /* Derived coefficient B2 */
    B2 = Kp + Kd*N;
    /* Derived coefficient A0 */
    A0 = 1 + N*Ts;
    /* Derived coefficient A1 */
    A1 = -(2 + N*Ts);
    /* Derived coefficient A2 */
    A2 = 1;
    /* Store the user-defined gain values */
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
}

void PID::setSampleTime(unsigned long sampleTime) {
    /* Sample time in us */
    Ts = sampleTime*1000;
    /* The derived coefficients have to be recomputed because they are affected by Ts */
    PID::setGains(_Kp, _Ki, _Kd);
}

void setOutputLimits(double upperLimit, double lowerLimit) {
    _upperLimit = upperLimit;
    _lowerLimit = lowerLimit;
}
