// ECE_UAV.cpp
#include "ECE_UAV.h"
#include <random>
#include <iostream>

// static members
std::atomic<int>  ECE_UAV::s_numOnSphere{0};
std::atomic<bool> ECE_UAV::s_allOnSphere{false};
ECE_UAV::Clock::time_point ECE_UAV::s_allOnSphereTime;

ECE_UAV::ECE_UAV(int id, const Vec3& startPos, int totalUavs)
    : m_id(id),
      m_totalUavs(totalUavs),
      m_position(startPos),
      m_velocity(0.0, 0.0, 0.0),
      m_startPos(startPos),
      m_phase(Phase::WAIT),
      m_localTime(0.0),
      m_countedOnSphere(false),
      m_sphereCenter(0.0, 0.0, 50.0),   // sphere center as prof specified
      m_tangentVel(0.0, 0.0, 0.0),
      m_targetSpeed(3.0),
      // PID gains - fly to (0,0,50) smoothly without crazy overshoot
      m_pidX(0.6, 0.0, 0.15),
      m_pidY(0.6, 0.0, 0.15),
      m_pidZ(0.9, 0.0, 0.20),
      m_running(false)
{
}

ECE_UAV::~ECE_UAV()
{
    stop();
}

void ECE_UAV::start()
{
    if (m_running) return;
    m_running = true;
    m_thread  = std::thread(&ECE_UAV::threadLoop, this);
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
    // Used by collision code to swap velocities
    std::lock_guard<std::mutex> lock(m_mutex);
    m_velocity = v;
    if (m_phase == Phase::ON_SPHERE && v.length() > 1e-4)
        m_tangentVel = v;
}

void ECE_UAV::threadLoop()
{
    auto last = Clock::now();

    while (m_running)
    {
        auto now = Clock::now();
        double dt = std::chrono::duration<double>(now - last).count();
        last = now;

        if (dt <= 0.0) dt = 0.001;
        if (dt > 0.05) dt = 0.05;  // clamp for stability

        // After all UAVs are on the sphere and 60 s have passed, freeze them
        if (s_allOnSphere)
        {
            double elapsedAfterAll =
                std::chrono::duration<double>(now - s_allOnSphereTime).count();
            if (elapsedAfterAll > 60.0)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }
        }

        m_localTime += dt;

        switch (m_phase)
        {
        case Phase::WAIT:
            updateWait(dt);
            break;
        case Phase::TO_CENTER:
            updateToCenter(dt);
            break;
        case Phase::ON_SPHERE:
            updateOnSphere(dt);
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

void ECE_UAV::updateWait(double /*dt*/)
{
    // sit on the field for 5 seconds
    if (m_localTime < kWaitTime)
        return;

    std::lock_guard<std::mutex> lock(m_mutex);
    m_phase    = Phase::TO_CENTER;
    m_velocity = Vec3(0.0, 0.0, 0.0);
    m_pidX.reset();
    m_pidY.reset();
    m_pidZ.reset();
}

void ECE_UAV::updateToCenter(double dt)
{
    Vec3 pos, vel;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        pos = m_position;
        vel = m_velocity;
    }

    Vec3 target = m_sphereCenter;        // fly to (0,0,50)
    Vec3 error  = target - pos;

    // PID outputs desired velocity components
    double vx = m_pidX.update(error.x, dt);
    double vy = m_pidY.update(error.y, dt);
    double vz = m_pidZ.update(error.z, dt);

    Vec3 cmd(vx, vy, vz);

    // cap speed so launch speed never exceeds about 10 m/s
    double speed = cmd.length();
    if (speed > kMaxSpeed)
        cmd = cmd * (kMaxSpeed / speed);

    pos += cmd * dt;
    vel  = cmd;

    // did we reach the virtual sphere of radius 10 m around (0,0,50)?
    Vec3 diff = pos - m_sphereCenter;
    double distToCenter = diff.length();

    if (distToCenter <= kSphereRadius)
    {
        // project exactly onto surface
        Vec3 radial = distToCenter > 1e-6
                      ? diff / distToCenter
                      : Vec3(0.0, 0.0, 1.0);

        pos = m_sphereCenter + radial * kSphereRadius;

        // pick random tangent direction and speed between 2 and 10 m/s
        static thread_local std::mt19937 rng(std::random_device{}());
        std::uniform_real_distribution<double> uni(-1.0, 1.0);
        std::uniform_real_distribution<double> speedDist(kMinSphereSpeed,
                                                         kMaxSphereSpeed);

        Vec3 randDir(uni(rng), uni(rng), uni(rng));
        if (randDir.length() < 1e-4)
            randDir = Vec3(1.0, 0.0, 0.0);

        Vec3 tangent = randDir - radial * Vec3::dot(randDir, radial);
        if (tangent.length() < 1e-4)
            tangent = Vec3::cross(radial, Vec3(0.0, 1.0, 0.0));

        m_targetSpeed = speedDist(rng);
        Vec3 tangentVel = tangent.normalized() * m_targetSpeed;

        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_position   = pos;
            m_velocity   = tangentVel;
            m_tangentVel = tangentVel;
            m_phase      = Phase::ON_SPHERE;
        }

        // update global counters for "all on sphere" timer
        if (!m_countedOnSphere)
        {
            m_countedOnSphere = true;
            int newCount = ++s_numOnSphere;
            if (newCount == m_totalUavs)
            {
                s_allOnSphere     = true;
                s_allOnSphereTime = Clock::now();
                std::cout << "All UAVs reached the sphere. "
                             "Starting 60 second timer.\n";
            }
        }
    }
    else
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_position = pos;
        m_velocity = vel;
    }
}

void ECE_UAV::updateOnSphere(double dt)
{
    Vec3 pos, vel;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        pos = m_position;
        vel = m_velocity;
    }

    Vec3 radial = pos - m_sphereCenter;
    double rLen = radial.length();
    if (rLen < 1e-6)
        radial = Vec3(0.0, 0.0, 1.0);
    else
        radial = radial / rLen;

    static thread_local std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> noiseDist(-0.5, 0.5);

    // keep a tangent velocity and add a bit of noise so the path wiggles
    Vec3 tangentVel = m_tangentVel;
    tangentVel = tangentVel - radial * Vec3::dot(tangentVel, radial);

    Vec3 noise(noiseDist(rng), noiseDist(rng), noiseDist(rng));
    Vec3 noiseTangent = noise - radial * Vec3::dot(noise, radial);
    tangentVel = tangentVel + noiseTangent * 0.1;

    double speed = tangentVel.length();
    if (speed < 1e-4)
        tangentVel = Vec3::cross(radial, Vec3(1.0, 0.0, 0.0));

    tangentVel = tangentVel.normalized();

    // slowly vary target speed but keep inside [2,10] m/s
    m_targetSpeed += noiseDist(rng) * 0.2;
    if (m_targetSpeed < kMinSphereSpeed) m_targetSpeed = kMinSphereSpeed;
    if (m_targetSpeed > kMaxSphereSpeed) m_targetSpeed = kMaxSphereSpeed;

    tangentVel = tangentVel * m_targetSpeed;

    // integrate on the surface and project back to exact radius
    pos += tangentVel * dt;
    Vec3 diff = pos - m_sphereCenter;
    double d  = diff.length();
    if (d > 1e-6)
        pos = m_sphereCenter + diff * (kSphereRadius / d);
    else
        pos = m_sphereCenter + radial * kSphereRadius;

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_position   = pos;
        m_velocity   = tangentVel;
        m_tangentVel = tangentVel;
    }
}
