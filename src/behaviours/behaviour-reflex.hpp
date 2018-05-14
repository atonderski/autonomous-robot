#ifndef BEHAVIOUR_REFLEX
#define BEHAVIOUR_REFLEX

#include "augmentedfinitestatemachine.hpp"

class BehaviourReflex : public AugmentedFiniteStateMachine {
    public:
        BehaviourReflex(ConfigurationValues m_conf, double const DT, double *stepFront, double *stepRear, double *stepLeft, double *stepRight) noexcept
            : AugmentedFiniteStateMachine(m_conf, DT, stepFront, stepRear, stepLeft, stepRight)
            , m_countdown{}
        {
        }
        ~BehaviourReflex() override = default;

        void initialState() noexcept override;
        void stateMoveBack() noexcept;

    private:
        double m_countdown;

};

#endif