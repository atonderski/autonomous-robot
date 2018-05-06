#include "augmentedfinitestatemachine.hpp"

class BehaviourReflex : public AugmentedFiniteStateMachine {
    public:
        BehaviourReflex(double const DT, double *stepFront, double *stepRear, double *stepLeft, double *stepRight) noexcept
            : AugmentedFiniteStateMachine(DT, stepFront, stepRear, stepLeft, stepRight)
        {
        }
        ~BehaviourReflex() = default;

        void initialState() noexcept override;
        void stateMoveBack() noexcept;

};