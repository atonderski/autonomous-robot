#ifndef BEHAVIOUR_AVOID
#define BEHAVIOUR_AVOID

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "augmentedfinitestatemachine.hpp"

class BehaviourAvoid : public AugmentedFiniteStateMachine {
    public:
        BehaviourAvoid(ConfigurationValues m_conf, double const DT, double *stepFront, double *stepRear, double *stepLeft, double *stepRight) noexcept
            : AugmentedFiniteStateMachine(m_conf, DT, stepFront, stepRear, stepLeft, stepRight)
            , previousTurnRight{}
            , m_countdown{}
            , m_randomizerCountdown{-1.0}
            , PREVIOUS_TURN_MEMORY{8.0}
        {
            srand(time(NULL));
        }
        ~BehaviourAvoid() override = default;

        void universalState() noexcept override;
        void initialState() noexcept override;
        void stateFrontTurnLeft() noexcept;
        void stateFrontTurnRight() noexcept;
        void stateSideTurnLeft() noexcept;
        void stateSideTurnRight() noexcept;

    private:
        bool previousTurnRight;
        double m_countdown;
        double m_randomizerCountdown;
        double const PREVIOUS_TURN_MEMORY;

};

#endif