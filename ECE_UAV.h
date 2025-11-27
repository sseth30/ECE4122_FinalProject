// ECE_UAV.h
#pragma once
#include <thread>
#include <atomic>
#include <mutex>
#include "Vec3.h"
#include "PID.h"

// One UAV controlled in its own thread.
// Phases:
//  1) Take off from field → rendezvous point above midfield
//  2) Orbit along the surface of the virtual sphere
class ECE_UAV
{
public:
    ECE_UAV(int id,
            const Vec3& startPos,
            int totalUAVs);

    ~ECE_UAV();

    void start();
    void stop();

    // Thread-safe getters/setters used by the render / collision code
    void getState(Vec3& pos, Vec3& vel) const;
    void setVelocity(const Vec3& v);

private:
    void run();   // thread loop

    int   m_id;
    int   m_total;

    Vec3  m_position;
    Vec3  m_velocity;
    Vec3  m_target;     // current position setpoint

    // PID controllers for each axis (Appendix B)
    PID   m_pidX;
    PID   m_pidY;
    PID   m_pidZ;

    // Trajectory state
    enum Phase { ASCENT_TO_RENDEZVOUS, SPHERE_FORMATION };
    Phase m_phase;
    double m_orbitAngle;
    double m_angleOffset;

    mutable std::mutex m_mutex;
    std::thread        m_thread;
    std::atomic<bool>  m_running;
};
