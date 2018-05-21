#ifndef BEHAVIOUR_FOLLOW_ROBOT
#define BEHAVIOUR_FOLLOW_ROBOT

#include "augmentedfinitestatemachine.hpp"

class BehaviourFollowRobot : public AugmentedFiniteStateMachine {
public:
    BehaviourFollowRobot(ConfigurationValues m_conf, double const DT, double *stepDetectionAngle, double *stepDetectionDistance, double *stepFront, double *stepRear, double *stepLeft, double *stepRight) noexcept
        : AugmentedFiniteStateMachine(m_conf, DT, stepFront, stepRear, stepLeft, stepRight)
        , m_pDetectionAngle{stepDetectionAngle}
        , m_pDetectionDistance{stepDetectionDistance}
        , targetVisible{false}
    {

    }
    ~BehaviourFollowRobot() noexcept = default;

    void universalState() noexcept override;
    void initialState() noexcept override;
    void atTargetPositionState() noexcept;
    void followTurnLeft() noexcept;
    void followTurnRight() noexcept;

private:
    double const *m_pDetectionAngle;
    double const *m_pDetectionDistance;
    bool targetVisible;

    BehaviourFollowRobot(BehaviourFollowRobot const &) = delete;
    BehaviourFollowRobot &operator=(BehaviourFollowRobot const &) = delete;

};

#endif