/*
Author: Your Name
Class: ECE4122 or ECE6122
Last Date Modified: 11/26/2025
Description:
Simple PID controller used to compute control forces on each axis.
*/

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

    PID(double p = 0.0, double i = 0.0, double d = 0.0)
        : Kp(p), Ki(i), Kd(d),
          integral(0.0), prevError(0.0), first(true)
    {}

    // error = setpoint - processVariable
    double update(double error, double dt)
    {
        if (dt <= 0.0)
            return 0.0;

        // proportional
        double P = Kp * error;

        // integral with simple anti windup
        integral += error * dt;
        const double Imax = 100.0;
        if (integral > Imax)  integral = Imax;
        if (integral < -Imax) integral = -Imax;
        double I = Ki * integral;

        // derivative
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
        integral  = 0.0;
        prevError = 0.0;
        first     = true;
    }
};
