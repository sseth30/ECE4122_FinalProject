// ECE_UAV.h
#pragma once

#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include "Vec3.h"
#include "PID.h"

class ECE_UAV
{
public:
    ECE_UAV(int id, const Vec3& startPos, int totalUavs);
    ~ECE_UAV();

    void start();
    void stop();

    void getState(Vec3& pos, Vec3& vel) const;
    void setVelocity(const Vec3& v);  // used by collision code

private:
    enum class Phase { WAIT, TO_CENTER, ON_SPHERE };

    void threadLoop();
    void updateWait(double dt);
    void updateToCenter(double dt);
    void updateOnSphere(double dt);

    // identity / configuration
    int   m_id;
    int   m_totalUavs;

    // kinematics
    Vec3  m_position;
    Vec3  m_velocity;
    Vec3  m_startPos;

    // control state
    Phase  m_phase;
    double m_localTime;        // seconds since this UAV started
    bool   m_countedOnSphere;

    // rendezvous / sphere parameters
    Vec3   m_sphereCenter;

    // tangent motion on sphere
    Vec3   m_tangentVel;
    double m_targetSpeed;

    // PID controllers (position -> desired velocity)
    PID    m_pidX;
    PID    m_pidY;
    PID    m_pidZ;

    // threading
    mutable std::mutex m_mutex;
    std::thread        m_thread;
    std::atomic<bool>  m_running;

    // constants
    static constexpr double kWaitTime       = 5.0;   // seconds sitting on field
    static constexpr double kSphereRadius   = 10.0;  // meters
    static constexpr double kMaxSpeed       = 10.0;  // overall speed cap
    static constexpr double kMinSphereSpeed = 2.0;   // motion on sphere
    static constexpr double kMaxSphereSpeed = 10.0;

    // global tracking of when all UAVs reach the sphere
    using Clock = std::chrono::steady_clock;
    static std::atomic<int>  s_numOnSphere;
    static std::atomic<bool> s_allOnSphere;
    static Clock::time_point s_allOnSphereTime;
};
