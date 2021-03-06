#ifndef AUTONOMOUS_ROBOT_STALKER
#define AUTONOMOUS_ROBOT_STALKER

#include "controller.hpp"
#include "configurationvalues.hpp"
#include "accelerationregulator.hpp"
#include "behaviours/behaviour-follow-robot.hpp"

class Stalker : public Controller {
public:
    Stalker(double const DT) noexcept
        : Controller(DT)
        , m_stepFront{}
        , m_stepRear{}
        , m_stepLeft{}
        , m_stepRight{}
        , m_stepDetectionAngle{}
        , m_stepDetectionDistance{}
        , m_conf{"/opt/stalker.conf"}
        , m_accelerationRegulator{m_conf, DT}
        , m_behaviourFollowRobot{m_conf, DT, &m_stepDetectionAngle, &m_stepDetectionDistance, &m_stepFront, &m_stepRear, &m_stepLeft, &m_stepRight}
        {
            
        }
    ~Stalker() override = default;

    bool step() noexcept override;

private:
    double m_stepFront;
    double m_stepRear;
    double m_stepLeft;
    double m_stepRight;
    double m_stepDetectionAngle;
    double m_stepDetectionDistance;

    ConfigurationValues m_conf;
    AccelerationRegulator m_accelerationRegulator;
    BehaviourFollowRobot m_behaviourFollowRobot;
};

#endif