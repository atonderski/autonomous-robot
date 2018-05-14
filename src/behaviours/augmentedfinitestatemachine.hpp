#ifndef FINITE_STATE_MACHINE
#define FINITE_STATE_MACHINE

#include <limits>
#include <iostream>
#include "configurationvalues.hpp"

class AugmentedFiniteStateMachine {
    public:
        AugmentedFiniteStateMachine(ConfigurationValues m_conf, double const DT, double *stepFront, double *stepRear, double *stepLeft, double *stepRight) noexcept
            : m_groundSteeringAngle{0}
            , m_pedalLogic{0}
            , m_dt{DT}
            , m_pFrontDistance{stepFront}
            , m_pRearDistance{stepRear}
            , m_pLeftDistance{stepLeft}
            , m_pRightDistance{stepRight}
            // Parameter space
            , MAX_STEERING_LEFT(m_conf.confMap.count("MAX_STEERING_LEFT") ? m_conf.confMap["MAX_STEERING_LEFT"] : 0.50)
            , MAX_STEERING_RIGHT(m_conf.confMap.count("MAX_STEERING_RIGHT") ? m_conf.confMap["MAX_STEERING_RIGHT"] : -0.50)
            , SCAN_STEERING_LEFT(m_conf.confMap.count("SCAN_STEERING_LEFT") ? m_conf.confMap["SCAN_STEERING_LEFT"] : 0.25)
            , SCAN_STEERING_RIGHT(m_conf.confMap.count("SCAN_STEERING_RIGHT") ? m_conf.confMap["SCAN_STEERING_RIGHT"] : -0.25)
            , MOTIVATION_ACT_FRONT(m_conf.confMap.count("MOTIVATION_ACT_FRONT") ? m_conf.confMap["MOTIVATION_ACT_FRONT"] : 0.40)
            , MOTIVATION_SCAN_SEMIPERIOD(m_conf.confMap.count("MOTIVATION_SCAN_SEMIPERIOD") ? m_conf.confMap["MOTIVATION_SCAN_SEMIPERIOD"] : 0.60)
            , REFLEX_CRITICAL_FRONT(m_conf.confMap.count("REFLEX_CRITICAL_FRONT") ? m_conf.confMap["REFLEX_CRITICAL_FRONT"] : 0.40)
            , REFLEX_STOP_FRONT(m_conf.confMap.count("REFLEX_STOP_FRONT") ? m_conf.confMap["REFLEX_STOP_FRONT"] : 0.70)
            , REFLEX_STOP_REAR(m_conf.confMap.count("REFLEX_STOP_REAR") ? m_conf.confMap["REFLEX_STOP_REAR"] : 0.40)
            , REFLEX_STOP_LEFTRIGHT(m_conf.confMap.count("REFLEX_STOP_LEFTRIGHT") ? m_conf.confMap["REFLEX_STOP_LEFTRIGHT"] : 0.30)
            , AVOID_ACT_FRONT(m_conf.confMap.count("AVOID_ACT_FRONT") ? m_conf.confMap["AVOID_ACT_FRONT"] : 0.90)
            , AVOID_ACT_LEFTRIGHT(m_conf.confMap.count("AVOID_ACT_LEFTRIGHT") ? m_conf.confMap["AVOID_ACT_LEFTRIGHT"] : 0.36)
            , AVOID_TURNTIME(m_conf.confMap.count("AVOID_TURNTIME") ? m_conf.confMap["AVOID_TURNTIME"] : 0.60)
            // Edge of space
            , activeState{0}
        {
            std::cout   << MAX_STEERING_LEFT << " "
                        << MAX_STEERING_RIGHT << " "
                        << SCAN_STEERING_LEFT << " "
                        << SCAN_STEERING_RIGHT << " "
                        << MOTIVATION_ACT_FRONT << " "
                        << MOTIVATION_SCAN_SEMIPERIOD << " "
                        << REFLEX_CRITICAL_FRONT << " "
                        << REFLEX_STOP_FRONT << " "
                        << REFLEX_STOP_REAR << " "
                        << REFLEX_STOP_LEFTRIGHT << " "
                        << AVOID_ACT_FRONT << " "
                        << AVOID_ACT_LEFTRIGHT << " "
                        << AVOID_TURNTIME << std::endl;
        }
        virtual ~AugmentedFiniteStateMachine() = default;        

        template <typename Container>
        void setState(void (Container::* newState)()) noexcept {
            activeState = static_cast<void(AugmentedFiniteStateMachine::*)()>(newState);
            update();
        }
        void setState() noexcept {
            activeState = 0;
        }
        
        void update() noexcept {
            if (activeState != 0) {
                (this->*activeState)();
            } else {
                initialState();
            }
        }

        bool getActivationStatus() noexcept {
            return activeState != 0;
        }

        virtual void initialState() noexcept = 0;

        double m_groundSteeringAngle;
        int m_pedalLogic;

    protected:
        double const m_dt;
        double const *m_pFrontDistance;
        double const *m_pRearDistance;
        double const *m_pLeftDistance;
        double const *m_pRightDistance;
        
        double const MAX_STEERING_LEFT;
        double const MAX_STEERING_RIGHT;
        double const SCAN_STEERING_LEFT;
        double const SCAN_STEERING_RIGHT;
        double const MOTIVATION_ACT_FRONT;
        double const MOTIVATION_SCAN_SEMIPERIOD;
        double const REFLEX_CRITICAL_FRONT;
        double const REFLEX_STOP_FRONT;
        double const REFLEX_STOP_REAR;
        double const REFLEX_STOP_LEFTRIGHT;
        double const AVOID_ACT_FRONT;
        double const AVOID_ACT_LEFTRIGHT;
        double const AVOID_TURNTIME;

    private:
        AugmentedFiniteStateMachine(AugmentedFiniteStateMachine const &) = delete;
        AugmentedFiniteStateMachine(AugmentedFiniteStateMachine &&) = delete;
        AugmentedFiniteStateMachine &operator=(AugmentedFiniteStateMachine const &) = delete;
        AugmentedFiniteStateMachine &operator=(AugmentedFiniteStateMachine &&) = delete;

        void (AugmentedFiniteStateMachine::* activeState)();
};

#endif