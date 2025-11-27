// ECE_UAV.cpp
#include "ECE_UAV.h"
#include <chrono>
#include <thread>
#include <cmath>

using Clock = std::chrono::steady_clock;

// Global formation parameters must match main.cpp
static const Vec3  kRendezvous(0.0, 0.0, 30.0);   // point above midfield
static const Vec3  kSphereCenter(0.0, 0.0, 40.0); // center of virtual sphere
static const double kSphereRadius = 12.0;

ECE_UAV::ECE_UAV(int id,
                 const Vec3& startPos,
                 int totalUAVs)
    : m_id(id),
      m_total(totalUAVs),
      m_position(startPos),
      m_velocity(0.0, 0.0, 0.0),
      m_target(kRendezvous),
      // PID gains tuned for smooth motion (Appendix B)
      m_pidX(0.8, 0.02, 0.25),
      m_pidY(0.8, 0.02, 0.25),
      m_pidZ(1.0, 0.03, 0.35),
      m_phase(ASCENT_TO_RENDEZVOUS),
      m_orbitAngle(0.0),
      m_angleOffset(0.0),
      m_running(false)
{
    if (m_total <= 0) m_total = 1;
    m_angleOffset = (2.0 * M_PI * m_id) / static_cast<double>(m_total);
}

ECE_UAV::~ECE_UAV()
{
    stop();
}

void ECE_UAV::start()
{
    if (m_running) return;
    m_running = true;
    m_thread = std::thread(&ECE_UAV::run, this);
}

void ECE_UAV::stop()
{
    if (!m_running) return;
    m_running = false;
    if (m_thread.joinable())
        m_thread.join();
}

void ECE_UAV::getState(Vec3& pos, Vec3& vel) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    pos = m_position;
    vel = m_velocity;
}

void ECE_UAV::setVelocity(const Vec3& v)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_velocity = v;
}

void ECE_UAV::run()
{
    auto lastTime = Clock::now();

    while (m_running)
    {
        auto now = Clock::now();
        double dt = std::chrono::duration<double>(now - lastTime).count();
        lastTime = now;
        if (dt > 0.05) dt = 0.05;          // clamp dt for stability
        if (dt <= 0.0) dt = 0.01;

        Vec3 pos, vel;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            pos = m_position;
            vel = m_velocity;
        }

        // ---------------- Trajectory generation ----------------
        if (m_phase == ASCENT_TO_RENDEZVOUS)
        {
            m_target = kRendezvous;

            double dist = (pos - kRendezvous).length();
            if (dist < 0.5)   // reached rendezvous
            {
                m_phase = SPHERE_FORMATION;
                m_pidX.reset();
                m_pidY.reset();
                m_pidZ.reset();
                m_orbitAngle = 0.0;
            }
        }
        else if (m_phase == SPHERE_FORMATION)
        {
            // Move on a horizontal great-circle around the sphere
            m_orbitAngle += 0.3 * dt;  // rad/s

            double theta = m_orbitAngle + m_angleOffset;

            Vec3 spherePoint(
                kSphereCenter.x + kSphereRadius * std::cos(theta),
                kSphereCenter.y + kSphereRadius * std::sin(theta),
                kSphereCenter.z
            );

            m_target = spherePoint;
        }

        // ---------------- PID control (Appendix B) ----------------
        Vec3 error = m_target - pos;

        // For each axis, PID output is a commanded velocity
        double vx_cmd = m_pidX.update(error.x, dt);
        double vy_cmd = m_pidY.update(error.y, dt);
        double vz_cmd = m_pidZ.update(error.z, dt);

        // Limit speed
        Vec3 cmd(vx_cmd, vy_cmd, vz_cmd);
        const double maxSpeed = 15.0; // m/s
        double speed = cmd.length();
        if (speed > maxSpeed)
            cmd = cmd * (maxSpeed / speed);

        pos += cmd * dt;
        vel  = cmd;

        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_position = pos;
            m_velocity = vel;
        }

        // 50 Hz update
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}
