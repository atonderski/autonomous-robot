#ifndef FINITE_STATE_MACHINE
#define FINITE_STATE_MACHINE

#include <limits>

class AugmentedFiniteStateMachine {
    public:
        AugmentedFiniteStateMachine(double const DT, double *stepFront, double *stepRear, double *stepLeft, double *stepRight) noexcept
            : m_groundSteeringAngle{0}
            , m_pedalPosition{0}
            , m_dt{DT}
            , m_pFrontDistance{stepFront}
            , m_pRearDistance{stepRear}
            , m_pLeftDistance{stepLeft}
            , m_pRightDistance{stepRight}
            // Action
            , MAX_STEERING_LEFT{0.5}
            , MAX_STEERING_RIGHT{-0.5}
            , MIN_PEDAL_FORWARD{0.01}
            , MIN_PEDAL_BACKWARD{-0.16}
            // Motivation
            , MOTIVATION_ACT_FRONT{0.80f}
            // Reflex
            , REFLEX_CRITICAL_FRONT{0.35f}
            , REFLEX_STOP_FRONT{0.70f}
            , REFLEX_STOP_REAR{0.35f}
            // Avoid
            , AVOID_ACT_FRONT{0.80f}
            , AVOID_ACT_LEFTRIGHT{0.30f}
            , AVOID_STOP_REAR{1.00f}
            // Private
            , activeState{0}
        {
        }
        virtual ~AugmentedFiniteStateMachine() = default;        

        template <typename Container>
        void setState(void (Container::* newState)()) noexcept {
            activeState = static_cast<void(AugmentedFiniteStateMachine::*)()>(newState);
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
        double m_pedalPosition;

    protected:
        double const m_dt;
        double const *m_pFrontDistance;
        double const *m_pRearDistance;
        double const *m_pLeftDistance;
        double const *m_pRightDistance;
        
        double const MAX_STEERING_LEFT;
        double const MAX_STEERING_RIGHT;
        double const MIN_PEDAL_FORWARD;
        double const MIN_PEDAL_BACKWARD;
        float const MOTIVATION_ACT_FRONT;
        float const REFLEX_CRITICAL_FRONT;
        float const REFLEX_STOP_FRONT;
        float const REFLEX_STOP_REAR;
        float const AVOID_ACT_FRONT;
        float const AVOID_ACT_LEFTRIGHT;
        float const AVOID_STOP_REAR;

    private:
        AugmentedFiniteStateMachine(AugmentedFiniteStateMachine const &) = delete;
        AugmentedFiniteStateMachine(AugmentedFiniteStateMachine &&) = delete;
        AugmentedFiniteStateMachine &operator=(AugmentedFiniteStateMachine const &) = delete;
        AugmentedFiniteStateMachine &operator=(AugmentedFiniteStateMachine &&) = delete;

        void (AugmentedFiniteStateMachine::* activeState)();
};

#endif