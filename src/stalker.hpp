#ifndef AUTONOMOUS_ROBOT_STALKER
#define AUTONOMOUS_ROBOT_STALKER

#include "controller.hpp"
#include "configurationvalues.hpp"
#include "accelerationregulator.hpp"
//#include "behaviourclass.hpp"

class Stalker : public Controller {
public:
    Stalker(double const DT) noexcept
        : Controller(DT)
        , m_stepFront{}
        , m_stepRear{}
        , m_stepLeft{}
        , m_stepRight{}
        , m_conf{"/opt/stalker.conf"}
        , m_accelerationRegulator{m_conf, DT}
        // Behaviour object initialized here
        {
            
        }
    ~Stalker() override = default;

    bool step() noexcept override;

private:
    double m_stepFront;
    double m_stepRear;
    double m_stepLeft;
    double m_stepRight;

    ConfigurationValues m_conf;
    AccelerationRegulator m_accelerationRegulator;
    // BehaviourObject1 declared here
};

#endif