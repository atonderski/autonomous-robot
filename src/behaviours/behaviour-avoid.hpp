#ifndef BEHAVIOUR_AVOID
#define BEHAVIOUR_AVOID

#include "augmentedfinitestatemachine.hpp"

class BehaviourAvoid : public AugmentedFiniteStateMachine {
    public:
        BehaviourAvoid(ConfigurationValues m_conf, double const DT, double *stepFront, double *stepRear, double *stepLeft, double *stepRight) noexcept
            : AugmentedFiniteStateMachine(m_conf, DT, stepFront, stepRear, stepLeft, stepRight)
            , previousTurnRight{}
            , m_countdown{}
        {
        }
        ~BehaviourAvoid() override = default;

        void initialState() noexcept override;
        void stateFrontTurnLeft() noexcept;
        void stateFrontTurnRight() noexcept;
        void stateSideTurnLeft() noexcept;
        void stateSideTurnRight() noexcept;

    private:
        bool previousTurnRight;
        double m_countdown;

};

#endif