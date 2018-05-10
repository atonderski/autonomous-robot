#include "augmentedfinitestatemachine.hpp"

class BehaviourAvoid : public AugmentedFiniteStateMachine {
    public:
        BehaviourAvoid(double const DT, double *stepFront, double *stepRear, double *stepLeft, double *stepRight) noexcept
            : AugmentedFiniteStateMachine(DT, stepFront, stepRear, stepLeft, stepRight)
            , previousTurnRight{}
            , m_countdown{AVOID_TURNTIME}
        {
        }
        ~BehaviourAvoid() = default;

        void initialState() noexcept override;
        void stateTurnLeft() noexcept;
        void stateTurnRight() noexcept;

    private:
        bool previousTurnRight;
        double m_countdown;

};