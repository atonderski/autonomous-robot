#include "augmentedfinitestatemachine.hpp"

class BehaviourAvoid : public AugmentedFiniteStateMachine {
    public:
        BehaviourAvoid(double const DT, double *stepFront, double *stepRear, double *stepLeft, double *stepRight) noexcept
            : AugmentedFiniteStateMachine(DT, stepFront, stepRear, stepLeft, stepRight)
        {
        }
        ~BehaviourAvoid() = default;

        void initialState() noexcept override;
        void stateTurnLeft() noexcept;
        void stateTurnRight() noexcept;

};