// PID.h
#pragma once
#include <algorithm>

// Simple PID controller used per axis (x, y, z)
struct PID
{
    double Kp;
    double Ki;
    double Kd;

    double integral;
    double prevError;
    bool   first;

    PID(double p = 0.0, double i = 0.0, double d = 0.0)
        : Kp(p), Ki(i), Kd(d),
          integral(0.0), prevError(0.0), first(true)
    {}

    // error = setpoint - processVariable
    double update(double error, double dt)
    {
        if (dt <= 0.0) return 0.0;

        // Proportional
        double P = Kp * error;

        // Integral with simple anti-windup
        integral += error * dt;
        const double Imax = 50.0;
        integral = std::max(std::min(integral, Imax), -Imax);
        double I = Ki * integral;

        // Derivative
        double derivative = 0.0;
        if (!first)
            derivative = (error - prevError) / dt;
        else
            first = false;

        double D = Kd * derivative;

        prevError = error;

        return P + I + D;
    }

    void reset()
    {
        integral   = 0.0;
        prevError  = 0.0;
        first      = true;
    }
};
