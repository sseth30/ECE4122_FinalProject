#pragma once
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <vector>
#include "Vec3.h"

// Simple PID Controller Helper
struct PIDController
{
    double Kp, Ki, Kd;
    double integral;
    double prevError;

    PIDController(double p = 0.0, double i = 0.0, double d = 0.0)
        : Kp(p), Ki(i), Kd(d), integral(0.0), prevError(0.0) {}

    void reset()
    {
        integral = 0.0;
        prevError = 0.0;
    }

    double update(double error, double dt)
    {
        integral += error * dt;
        double deriv = (error - prevError) / dt;
        prevError = error;
        return (Kp * error) + (Ki * integral) + (Kd * deriv);
    }
};

class ECE_UAV
{
public:
    using Clock = std::chrono::steady_clock;

    // Flight States
    enum class Phase
    {
        WAIT,
        TO_CENTER,
        ON_SPHERE
    };

    ECE_UAV(int id, const Vec3& startPos, int totalUavs);
    ~ECE_UAV();

    void start();
    void stop();

    // Thread-safe getters/setters
    void getState(Vec3& pos, Vec3& vel) const;
    Vec3 getPosition() const { std::lock_guard<std::mutex> l(m_mutex); return m_position; }
    Vec3 getVelocity() const { std::lock_guard<std::mutex> l(m_mutex); return m_velocity; }
    void setVelocity(const Vec3& v);

private:
    void threadLoop();
    void updateWait(double dt);
    void updateToCenter(double dt);
    void updateOnSphere(double dt);

    int m_id;
    int m_totalUavs;

    // Kinematics
    mutable std::mutex m_mutex;
    Vec3 m_position;
    Vec3 m_velocity;
    Vec3 m_startPos;
    
    // Control / Physics
    Phase m_phase;
    double m_localTime;
    
    // Sphere / PID params
    bool m_countedOnSphere;
    Vec3 m_sphereCenter;
    Vec3 m_tangentVel;
    double m_targetSpeed;
    
    PIDController m_pidX;
    PIDController m_pidY;
    PIDController m_pidZ;

    // Threading
    std::thread m_thread;
    std::atomic<bool> m_running;

    // Static synchronization for "All On Sphere" condition
    static std::atomic<int> s_numOnSphere;
    static std::atomic<bool> s_allOnSphere;
    static Clock::time_point s_allOnSphereTime;

    // Constants
    static constexpr double kWaitTime = 5.0;
    static constexpr double kMaxSpeed = 8.0; // clamp launch speed
    static constexpr double kSphereRadius = 10.0;
    static constexpr double kMinSphereSpeed = 2.0;
    static constexpr double kMaxSphereSpeed = 10.0;
};