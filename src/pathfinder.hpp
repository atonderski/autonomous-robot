#ifndef AUTONOMOUS_ROBOT_PATHFINDER
#define AUTONOMOUS_ROBOT_PATHFINDER

#include "controller.hpp"
#include "configurationvalues.hpp"
#include "path-maker.hpp"
#include "accelerationregulator.hpp"
#include "behaviours/behaviour-reflex.hpp"
#include "behaviours/behaviour-avoid.hpp"
#include "behaviours/behaviour-followpath.hpp"

class Pathfinder : public Controller {
public:
    Pathfinder(double const DT) noexcept
        : Controller(DT)
        , m_stepFront{}
        , m_stepRear{}
        , m_stepLeft{}
        , m_stepRight{}
        , m_stepX{}
        , m_stepY{}
        , m_stepYaw{}
        , m_conf{"/opt/pathfinder.conf"}
        , m_pathMaker{m_conf}
        , m_accelerationRegulator{m_conf, DT}
        , m_behaviourReflex{m_conf, DT, &m_stepFront, &m_stepRear, &m_stepLeft, &m_stepRight}
        , m_behaviourAvoid{m_conf, DT, &m_stepFront, &m_stepRear, &m_stepLeft, &m_stepRight}
        , m_behaviourFollowPath{m_pathMaker.path, &m_stepX, &m_stepY, &m_stepYaw, m_conf, DT, &m_stepFront, &m_stepRear, &m_stepLeft, &m_stepRight}
    {
    }
    ~Pathfinder() override = default;

public:
    bool step() noexcept override;

private:
    double m_stepFront;
    double m_stepRear;
    double m_stepLeft;
    double m_stepRight;
    double m_stepX;
    double m_stepY;
    double m_stepYaw;

    ConfigurationValues m_conf;
    PathMaker m_pathMaker;
    AccelerationRegulator m_accelerationRegulator;
    BehaviourReflex m_behaviourReflex;
    BehaviourAvoid m_behaviourAvoid;
    BehaviourFollowPath m_behaviourFollowPath;

};

#endif