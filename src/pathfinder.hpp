#ifndef AUTONOMOUS_ROBOT_PATHFINDER
#define AUTONOMOUS_ROBOT_PATHFINDER

#include "controller.hpp"
#include <list>
#include "configurationvalues.hpp"
#include "accelerationregulator.hpp"
#include "path-maker.hpp"
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
        , m_pathList{std::make_pair(-2.0769,-2.2308),std::make_pair(-1.9231,-2.2308),std::make_pair(-1.7692,-2.2308),std::make_pair(-1.6154,-2.2308),std::make_pair(-1.4615,-2.2308),std::make_pair(-1.3077,-2.2308),std::make_pair(-1.1538,-2.2308),std::make_pair(-1,-2.2308),std::make_pair(-0.84615,-2.2308),std::make_pair(-0.69231,-2.2308),std::make_pair(-0.53846,-2.2308),std::make_pair(-0.38462,-2.2308),std::make_pair(-0.23077,-2.2308),std::make_pair(-0.076923,-2.2308),std::make_pair(0.076923,-2.2308),std::make_pair(0.23077,-2.2308),std::make_pair(0.38462,-2.2308),std::make_pair(0.53846,-2.2308),std::make_pair(0.69231,-2.2308),std::make_pair(0.8359,-2.2205),std::make_pair(0.96923,-2.2),std::make_pair(1.0821,-2.159),std::make_pair(1.1744,-2.0974),std::make_pair(1.2462,-2.0154),std::make_pair(1.3077,-1.9231),std::make_pair(1.359,-1.8205),std::make_pair(1.4,-1.7077),std::make_pair(1.4308,-1.5846),std::make_pair(1.4513,-1.4513),std::make_pair(1.4615,-1.3077),std::make_pair(1.4615,-1.1538),std::make_pair(1.4615,-1),std::make_pair(1.4513,-0.85641),std::make_pair(1.4205,-0.73333),std::make_pair(1.359,-0.64103),std::make_pair(1.2667,-0.57949),std::make_pair(1.1436,-0.54872),std::make_pair(1,-0.53846),std::make_pair(0.84615,-0.53846),std::make_pair(0.69231,-0.53846),std::make_pair(0.53846,-0.53846),std::make_pair(0.38462,-0.53846),std::make_pair(0.23077,-0.53846),std::make_pair(0.076923,-0.53846),std::make_pair(-0.076923,-0.53846),std::make_pair(-0.23077,-0.53846),std::make_pair(-0.38462,-0.53846),std::make_pair(-0.53846,-0.53846),std::make_pair(-0.69231,-0.53846),std::make_pair(-0.8359,-0.52821),std::make_pair(-0.96923,-0.50769),std::make_pair(-1.0923,-0.47692),std::make_pair(-1.2051,-0.4359),std::make_pair(-1.2974,-0.37436),std::make_pair(-1.3692,-0.29231),std::make_pair(-1.4205,-0.18974),std::make_pair(-1.4513,-0.066667),std::make_pair(-1.4513,0.066667),std::make_pair(-1.4205,0.18974),std::make_pair(-1.3692,0.29231),std::make_pair(-1.2974,0.37436),std::make_pair(-1.2154,0.44615),std::make_pair(-1.1231,0.50769),std::make_pair(-1.041,0.57949),std::make_pair(-0.95897,0.65128),std::make_pair(-0.88718,0.73333),std::make_pair(-0.80513,0.80513),std::make_pair(-0.73333,0.88718),std::make_pair(-0.66154,0.96923),std::make_pair(-0.61026,1.0718),std::make_pair(-0.56923,1.1846),std::make_pair(-0.53846,1.3077),std::make_pair(-0.50769,1.4308),std::make_pair(-0.47692,1.5538),std::make_pair(-0.4359,1.6667),std::make_pair(-0.37436,1.759),std::make_pair(-0.29231,1.8308),std::make_pair(-0.18974,1.8821),std::make_pair(-0.066667,1.9128),std::make_pair(0.076923,1.9231),std::make_pair(0.23077,1.9231),std::make_pair(0.38462,1.9231),std::make_pair(0.53846,1.9231),std::make_pair(0.68205,1.9128),std::make_pair(0.81538,1.8923),std::make_pair(0.92821,1.8513),std::make_pair(1.0308,1.8),std::make_pair(1.1128,1.7282),std::make_pair(1.1949,1.6564),std::make_pair(1.2667,1.5744),std::make_pair(1.3487,1.5026),std::make_pair(1.4205,1.4205),std::make_pair(1.5026,1.3487),std::make_pair(1.5744,1.2667),std::make_pair(1.6564,1.1949),std::make_pair(1.7385,1.1231),std::make_pair(1.8205,1.0513),std::make_pair(1.9231,1)}
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

    std::list<std::pair<double,double>> m_pathList;

    ConfigurationValues m_conf;
    PathMaker m_pathMaker;
    AccelerationRegulator m_accelerationRegulator;
    BehaviourReflex m_behaviourReflex;
    BehaviourAvoid m_behaviourAvoid;
    BehaviourFollowPath m_behaviourFollowPath;

};

#endif