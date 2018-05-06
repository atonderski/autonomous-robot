#include "augmentedfinitestatemachine.hpp"

class BehaviourMotivation : public AugmentedFiniteStateMachine {
    public:
        BehaviourMotivation(double const DT, double *stepFront, double *stepRear, double *stepLeft, double *stepRight) noexcept
            : AugmentedFiniteStateMachine(DT, stepFront, stepRear, stepLeft, stepRight)
        {
        }
        ~BehaviourMotivation() = default;

        void initialState() noexcept override;
        void stateMotivated() noexcept;

};