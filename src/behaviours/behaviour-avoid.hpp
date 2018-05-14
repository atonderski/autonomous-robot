#ifndef BEHAVIOUR_AVOID
#define BEHAVIOUR_AVOID

#include "augmentedfinitestatemachine.hpp"

class BehaviourAvoid : public AugmentedFiniteStateMachine {
    public:
        BehaviourAvoid(ConfigurationValues m_conf, double const DT, double *stepFront, double *stepRear, double *stepLeft, double *stepRight) noexcept
            : AugmentedFiniteStateMachine(m_conf, DT, stepFront, stepRear, stepLeft, stepRight)
            , previousTurnRight{}
            , m_countdown{}
            , avoidingFront{}
        {
        }
        ~BehaviourAvoid() override = default;

        void initialState() noexcept override;
        void stateTurnLeft() noexcept;
        void stateTurnRight() noexcept;

    private:
        bool previousTurnRight;
        double m_countdown;
        bool avoidingFront;

};

#endif