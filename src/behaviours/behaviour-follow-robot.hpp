#ifndef BEHAVIOUR_FOLLOW_ROBOT
#define BEHAVIOUR_FOLLOW_ROBOT

#include "augmentedfinitestatemachine.hpp"
#include <cmath>

class BehaviourFollowRobot : public AugmentedFiniteStateMachine {
public:
    BehaviourFollowRobot(ConfigurationValues m_conf, double const DT, double *stepDetectionAngle, double *stepDetectionDistance, double *stepFront, double *stepRear, double *stepLeft, double *stepRight) noexcept
        : AugmentedFiniteStateMachine(m_conf, DT, stepFront, stepRear, stepLeft, stepRight)
        , m_pDetectionAngle{stepDetectionAngle}
        , m_pDetectionDistance{stepDetectionDistance}
        , m_pFrontDistance{stepFront}
        , m_pRearDistance{stepRear}
        , targetVisible{true}
        , MARGIN(m_conf.confMap.count("MARGIN") ? m_conf.confMap["MARGIN"] : 0.2) // If no value found in config file use default 0.2
        , PREFERRED_DISTANCE(m_conf.confMap.count("PREFERRED_DISTANCE") ? m_conf.confMap["PREFERRED_DISTANCE"] : 0.5)
        , SMOOTH_STEERING_THRESHOLD(m_conf.confMap.count("SMOOTH_STEERING_THRESHOLD") ? m_conf.confMap["SMOOTH_STEERING_THRESHOLD"] : 0.5236) // pi/6
        , ULTRASONIC_ANGLE_THRESHOLD(m_conf.confMap.count("ULTRASONIC_ANGLE_THRESHOLD") ? m_conf.confMap["ULTRASONIC_ANGLE_THRESHOLD"] : 0.2)
    {

    }
    ~BehaviourFollowRobot() noexcept = default;

    void universalState() noexcept override;
    void initialState() noexcept override;
    void atAlignedPositionState() noexcept;
    void followForward() noexcept;
    void followBackward() noexcept;

private:
    double const *m_pDetectionAngle;
    double const *m_pDetectionDistance;
    double const *m_pFrontDistance;
    double const *m_pRearDistance;
    bool targetVisible;
    double const MARGIN;
    double const PREFERRED_DISTANCE;
    double const SMOOTH_STEERING_THRESHOLD;
    double const ULTRASONIC_ANGLE_THRESHOLD;

    BehaviourFollowRobot(BehaviourFollowRobot const &) = delete;
    BehaviourFollowRobot &operator=(BehaviourFollowRobot const &) = delete;

};

#endif