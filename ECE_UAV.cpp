/*
Author: Satchit Seth
Class: ECE4122
Last Date Modified: 12/01/2025

Description:
    UAVs execute a PID-controlled flight to a sphere then move tangentially on its surface.
    The system is multi-threaded and uses atomic variables for swarm synchronization.
*/

#include "ECE_UAV.h"
#include <random>
#include <iostream>

// static members definition
std::atomic<int>  ECE_UAV::s_numOnSphere{0};
std::atomic<bool> ECE_UAV::s_allOnSphere{false};
ECE_UAV::Clock::time_point ECE_UAV::s_allOnSphereTime;

/*
 * Constructor: ECE_UAV
 * Purpose    : Initialize a single UAV with its ID, starting position,
 *              and total number of UAVs in the simulation.
 * Input      : id        - unique integer identifier for this UAV.
 *              startPos  - initial position on the football field (meters).
 *              totalUavs - total UAV count, used to detect when all
 *                          UAVs reach the sphere.
 * Output     : Initializes member variables and PID controllers.
 * Return     : None.
 */
ECE_UAV::ECE_UAV(int id, const Vec3& startPos, int totalUavs)
    : m_id(id),
      m_totalUavs(totalUavs),
      m_position(startPos),
      m_velocity(0.0, 0.0, 0.0),
      m_startPos(startPos),
      m_phase(Phase::WAIT),
      m_localTime(0.0),
      m_countedOnSphere(false),
      m_sphereCenter(0.0, 0.0, 50.0),   
      m_tangentVel(0.0, 0.0, 0.0),
      m_targetSpeed(3.0),
      // PID gains (P, I, D)
      m_pidX(0.3, 0.0, 0.08),
      m_pidY(0.3, 0.0, 0.08),
      m_pidZ(0.45, 0.0, 0.10),
      m_running(false)
{
}

/*
 * Destructor: ~ECE_UAV
 * Purpose    : Ensure the UAV's worker thread is stopped cleanly.
 * Input      : None.
 * Output     : Joins the thread if it is still running.
 * Return     : None.
 */
ECE_UAV::~ECE_UAV()
{
    stop();
}

/*
 * Function: start
 * Purpose : Launch the UAV's simulation thread if it is not already running.
 * Input   : None.
 * Output  : Spawns std::thread executing threadLoop().
 * Return  : None.
 */
void ECE_UAV::start()
{
    if (m_running) return;
    m_running = true;
    m_thread  = std::thread(&ECE_UAV::threadLoop, this);
}

/*
 * Function: stop
 * Purpose : Stop the UAV simulation thread and join it.
 * Input   : None.
 * Output  : Sets m_running to false and joins the worker thread.
 * Return  : None.
 */
void ECE_UAV::stop()
{
    if (!m_running) return;
    m_running = false;
    if (m_thread.joinable())
        m_thread.join();
}

/*
 * Function: getState
 * Purpose : Thread-safe read of the UAV position and velocity.
 * Input   : References pos and vel, which will be written to.
 * Output  : pos - current UAV position.
 *           vel - current UAV velocity.
 * Return  : None.
 */
void ECE_UAV::getState(Vec3& pos, Vec3& vel) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    pos = m_position;
    vel = m_velocity;
}

/*
 * Function: setVelocity
 * Purpose : Thread-safe update of the UAV velocity, used for collision
 *           handling in the main thread.
 * Input   : v - new velocity vector to set.
 * Output  : Updates m_velocity and m_tangentVel when on the sphere.
 * Return  : None.
 */
void ECE_UAV::setVelocity(const Vec3& v)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_velocity = v;
    if (m_phase == Phase::ON_SPHERE && v.length() > 1e-4)
        m_tangentVel = v;
}

/*
 * Function: threadLoop
 * Purpose : Main worker loop for the UAV. This loop advances the
 *           simulation state for this UAV: waits on the ground,
 *           flies to the sphere center, then moves along the sphere.
 * Input   : None (runs on this object's thread).
 * Output  : Updates position, velocity, and state over time.
 * Return  : None (loop exits when m_running becomes false).
 */
void ECE_UAV::threadLoop()
{
    auto last = Clock::now();

    while (m_running)
    {
        auto now = Clock::now();
        double dt = std::chrono::duration<double>(now - last).count();
        last = now;

        if (dt <= 0.0) dt = 0.001;
        if (dt > 0.05) dt = 0.05;  // clamp maximum time step

        // After all UAVs are on the sphere, freeze motion after 60 s.
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

        // Local time since this UAV started running.
        m_localTime += dt;

        // Advance behavior according to current phase.
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

        // Run control loop at roughly 100 Hz.
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

/*
 * Function: updateWait
 * Purpose : Handle the WAIT phase where the UAV sits on the field.
 *           The UAV remains stationary until kWaitTime has elapsed.
 * Input   : dt - time step in seconds (unused in the current logic).
 * Output  : Once the wait time passes, transitions to TO_CENTER phase
 *           and resets PID controllers.
 * Return  : None.
 */
void ECE_UAV::updateWait(double /*dt*/)
{
    if (m_localTime < kWaitTime) return;

    std::lock_guard<std::mutex> lock(m_mutex);
    m_phase    = Phase::TO_CENTER;
    m_velocity = Vec3(0.0, 0.0, 0.0);
    m_pidX.reset();
    m_pidY.reset();
    m_pidZ.reset();
}

/*
 * Function: updateToCenter
 * Purpose : Handle motion from the starting spot on the field to the
 *           center of the virtual sphere at (0,0,50) using PID control.
 * Input   : dt - time step in seconds.
 * Output  : Updates UAV position and velocity, and switches to ON_SPHERE
 *           once the UAV reaches the sphere surface.
 * Return  : None.
 */
void ECE_UAV::updateToCenter(double dt)
{
    Vec3 pos, vel;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        pos = m_position;
        vel = m_velocity;
    }

    // Error is vector from current position to sphere center.
    Vec3 target = m_sphereCenter;        
    Vec3 error  = target - pos;

    // PID controllers generate command components along each axis.
    double vx = m_pidX.update(error.x, dt);
    double vy = m_pidY.update(error.y, dt);
    double vz = m_pidZ.update(error.z, dt);

    Vec3 cmd(vx, vy, vz);

    // Clamp commanded speed to the maximum allowed.
    double speed = cmd.length();
    if (speed > kMaxSpeed)
        cmd = cmd * (kMaxSpeed / speed);

    // Integrate position and velocity.
    pos += cmd * dt;
    vel  = cmd;

    // Compute distance to sphere center.
    Vec3 diff = pos - m_sphereCenter;
    double distToCenter = diff.length();

    if (distToCenter <= kSphereRadius)
    {
        // The UAV has intersected the sphere. Snap to surface.
        Vec3 radial = distToCenter > 1e-6 ? diff / distToCenter : Vec3(0.0, 0.0, 1.0);
        pos = m_sphereCenter + radial * kSphereRadius;

        // -------------------------------------------------------------------
        // Initialize random tangent motion along the sphere surface.
        // -------------------------------------------------------------------
        static thread_local std::mt19937 rng(std::random_device{}());
        std::uniform_real_distribution<double> uni(-1.0, 1.0);
        std::uniform_real_distribution<double> speedDist(kMinSphereSpeed, kMaxSphereSpeed);

        Vec3 randDir(uni(rng), uni(rng), uni(rng));
        if (randDir.length() < 1e-4) randDir = Vec3(1.0, 0.0, 0.0);

        // Remove radial component to get a tangent direction.
        Vec3 tangent = randDir - radial * Vec3::dot(randDir, radial);
        if (tangent.length() < 1e-4) tangent = Vec3::cross(radial, Vec3(0.0, 1.0, 0.0));

        // Choose a random speed between 2 and 10 m/s.
        m_targetSpeed = speedDist(rng);
        Vec3 tangentVel = tangent.normalized() * m_targetSpeed;

        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_position   = pos;
            m_velocity   = tangentVel;
            m_tangentVel = tangentVel;
            m_phase      = Phase::ON_SPHERE;
        }

        // Count how many UAVs have made it to the sphere.
        if (!m_countedOnSphere)
        {
            m_countedOnSphere = true;
            int newCount = ++s_numOnSphere;
            if (newCount == m_totalUavs)
            {
                s_allOnSphere     = true;
                s_allOnSphereTime = Clock::now();
                std::cout << "All UAVs on sphere. Starting 60s timer.\n";
            }
        }
    }
    else
    {
        // Still flying toward the center; store updated state.
        std::lock_guard<std::mutex> lock(m_mutex);
        m_position = pos;
        m_velocity = vel;
    }
}

/*
 * Function: updateOnSphere
 * Purpose : Handle motion of the UAV once it is constrained to the
 *           virtual sphere surface. The UAV travels along the surface
 *           with speed between kMinSphereSpeed and kMaxSphereSpeed,
 *           with slight random perturbations to create a show effect.
 * Input   : dt - time step in seconds.
 * Output  : Updates UAV position and tangent velocity on the sphere.
 * Return  : None.
 */
void ECE_UAV::updateOnSphere(double dt)
{
    Vec3 pos, vel;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        pos = m_position;
        vel = m_velocity;
    }

    // Radial direction from sphere center to the UAV.
    Vec3 radial = pos - m_sphereCenter;
    double rLen = radial.length();
    if (rLen < 1e-6) radial = Vec3(0.0, 0.0, 1.0);
    else radial = radial / rLen;

    static thread_local std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> noiseDist(-0.5, 0.5);

    Vec3 tangentVel = m_tangentVel;
    // Remove any radial component to keep motion on the surface.
    tangentVel = tangentVel - radial * Vec3::dot(tangentVel, radial);

    // Add small random noise to change direction over time.
    Vec3 noise(noiseDist(rng), noiseDist(rng), noiseDist(rng));
    Vec3 noiseTangent = noise - radial * Vec3::dot(noise, radial);
    tangentVel = tangentVel + noiseTangent * 0.1;

    double speed = tangentVel.length();
    if (speed < 1e-4) tangentVel = Vec3::cross(radial, Vec3(1.0, 0.0, 0.0));

    tangentVel = tangentVel.normalized();

    // Randomly vary target speed within the allowed range.
    m_targetSpeed += noiseDist(rng) * 0.2;
    if (m_targetSpeed < kMinSphereSpeed) m_targetSpeed = kMinSphereSpeed;
    if (m_targetSpeed > kMaxSphereSpeed) m_targetSpeed = kMaxSphereSpeed;

    tangentVel = tangentVel * m_targetSpeed;

    // Integrate position along the surface.
    pos += tangentVel * dt;
    
    // Project back onto exact sphere radius to avoid drift.
    Vec3 diff = pos - m_sphereCenter;
    double d  = diff.length();
    if (d > 1e-6) pos = m_sphereCenter + diff * (kSphereRadius / d);
    else pos = m_sphereCenter + radial * kSphereRadius;

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_position   = pos;
        m_velocity   = tangentVel;
        m_tangentVel = tangentVel;
    }
}