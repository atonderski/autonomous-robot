#ifndef AUTONOMOUS_ROBOT_SUBSUMER
#define AUTONOMOUS_ROBOT_SUBSUMER

#include "controller.hpp"
#include <limits>
#include <iostream>
#include "accelerationregulator.hpp"
#include "behaviours/behaviour-reflex.hpp"
#include "behaviours/behaviour-avoid.hpp"
#include "behaviours/behaviour-motivation.hpp"


class Subsumer : public Controller {
public:
    Subsumer(double const DT) noexcept
        : Controller(DT)
        , m_stepFront{}
        , m_stepRear{}
        , m_stepLeft{}
        , m_stepRight{}
        , m_accelerationRegulator{DT}
        , m_behaviourReflex{DT, &m_stepFront, &m_stepRear, &m_stepLeft, &m_stepRight}
        , m_behaviourAvoid{DT, &m_stepFront, &m_stepRear, &m_stepLeft, &m_stepRight}
        , m_behaviourMotivation{DT, &m_stepFront, &m_stepRear, &m_stepLeft, &m_stepRight}
    {
    }
    ~Subsumer() override = default;

public:
    bool step() noexcept override;

private:
    double m_stepFront;
    double m_stepRear;
    double m_stepLeft;
    double m_stepRight;

    AccelerationRegulator m_accelerationRegulator;
    BehaviourReflex m_behaviourReflex;
    BehaviourAvoid m_behaviourAvoid;
    BehaviourMotivation m_behaviourMotivation;

};

#endif