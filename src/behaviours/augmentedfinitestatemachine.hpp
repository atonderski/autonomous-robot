#ifndef FINITE_STATE_MACHINE
#define FINITE_STATE_MACHINE

#include <limits>

class AugmentedFiniteStateMachine {
    public:
        AugmentedFiniteStateMachine(double const DT, double *stepFront, double *stepRear, double *stepLeft, double *stepRight) noexcept
            : m_groundSteeringAngle{0}
            , m_pedalLogic{0}
            , m_dt{DT}
            , m_pFrontDistance{stepFront}
            , m_pRearDistance{stepRear}
            , m_pLeftDistance{stepLeft}
            , m_pRightDistance{stepRight}
            // Parameter space
            , MAX_STEERING_LEFT             {0.50}
            , MAX_STEERING_RIGHT            {-0.50}
            , SCAN_STEERING_LEFT            {0.25}
            , SCAN_STEERING_RIGHT           {-0.25}
            , MIN_PEDAL_FORWARD             {1}
            , MIN_PEDAL_BACKWARD            {-1}
            , MOTIVATION_ACT_FRONT          {0.40f}
            , MOTIVATION_SCAN_SEMIPERIOD    {0.60f}
            , REFLEX_CRITICAL_FRONT         {0.40f}
            , REFLEX_STOP_FRONT             {0.70f}
            , REFLEX_STOP_REAR              {0.40f}
            , REFLEX_STOP_LEFTRIGHT         {0.30f}
            , AVOID_ACT_FRONT               {0.90f}
            , AVOID_ACT_LEFTRIGHT           {0.36f}
            , AVOID_TURNTIME                {0.60f}
            // Edge of space
            , activeState{0}
        {
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
        int const MIN_PEDAL_FORWARD;
        int const MIN_PEDAL_BACKWARD;
        float const MOTIVATION_ACT_FRONT;
        float const MOTIVATION_SCAN_SEMIPERIOD;
        float const REFLEX_CRITICAL_FRONT;
        float const REFLEX_STOP_FRONT;
        float const REFLEX_STOP_REAR;
        float const REFLEX_STOP_LEFTRIGHT;
        float const AVOID_ACT_FRONT;
        float const AVOID_ACT_LEFTRIGHT;
        float const AVOID_TURNTIME;

    private:
        AugmentedFiniteStateMachine(AugmentedFiniteStateMachine const &) = delete;
        AugmentedFiniteStateMachine(AugmentedFiniteStateMachine &&) = delete;
        AugmentedFiniteStateMachine &operator=(AugmentedFiniteStateMachine const &) = delete;
        AugmentedFiniteStateMachine &operator=(AugmentedFiniteStateMachine &&) = delete;

        void (AugmentedFiniteStateMachine::* activeState)();
};

#endif