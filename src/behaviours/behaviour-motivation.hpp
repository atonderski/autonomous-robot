#ifndef BEHAVIOUR_MOTIVATION
#define BEHAVIOUR_MOTIVATION

#include "augmentedfinitestatemachine.hpp"

class BehaviourMotivation : public AugmentedFiniteStateMachine {
    public:
        BehaviourMotivation(ConfigurationValues m_conf, double const DT, double *stepFront, double *stepRear, double *stepLeft, double *stepRight) noexcept
            : AugmentedFiniteStateMachine(m_conf, DT, stepFront, stepRear, stepLeft, stepRight)
            , m_countdown{0}
        {
        }
        ~BehaviourMotivation() override = default;

        void universalState() noexcept override;
        void initialState() noexcept override;
        void stateMotivated() noexcept;
        void stateScan() noexcept;

    private:
        double m_countdown;
};

#endif