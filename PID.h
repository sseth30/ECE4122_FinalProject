// PID.h
#pragma once
#include <algorithm>

struct PID
{
    double Kp;
    double Ki;
    double Kd;

    double integral;
    double prevError;
    bool   first;

    double integralLimit;
    double outputLimit;

    PID(double p = 0.0,
        double i = 0.0,
        double d = 0.0,
        double iLimit = 100.0,
        double oLimit = 50.0)
        : Kp(p), Ki(i), Kd(d),
          integral(0.0),
          prevError(0.0),
          first(true),
          integralLimit(iLimit),
          outputLimit(oLimit)
    {
    }

    // error = (setpoint - processVariable)
    double update(double error, double dt)
    {
        if (dt <= 0.0) return 0.0;

        // Proportional
        double P = Kp * error;

        // Integral with anti-windup
        integral += error * dt;
        if (integral > integralLimit)  integral = integralLimit;
        if (integral < -integralLimit) integral = -integralLimit;
        double I = Ki * integral;

        // Derivative
        double derivative = 0.0;
        if (!first)
            derivative = (error - prevError) / dt;
        else
            first = false;
        double D = Kd * derivative;

        prevError = error;

        double out = P + I + D;

        // Output clamp
        if (out >  outputLimit) out =  outputLimit;
        if (out < -outputLimit) out = -outputLimit;

        return out;
    }

    void reset()
    {
        integral  = 0.0;
        prevError = 0.0;
        first     = true;
    }
};
