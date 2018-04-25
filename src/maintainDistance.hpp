#ifndef AUTONOMOUS_ROBOT_MAINTAIN_DISTANCE
#define AUTONOMOUS_ROBOT_MAINTAIN_DISTANCE

#include "controller.hpp"
#include "finiteStateMachine.hpp"
#include <limits>
#include <iostream>

class MaintainDistanceStates : public FiniteStateMachine {
    public:
        MaintainDistanceStates() noexcept;
        ~MaintainDistanceStates() = default;

        void initialState() noexcept;
        void stateMoveTowards() noexcept;
        void stateMoveAway() noexcept;
        void stateStationary() noexcept;

    private:
        double const CRITICAL_DISTANCE;
        double const MARGIN;
};

class MaintainDistance : public Controller {
public:
    MaintainDistance() noexcept = default;
    ~MaintainDistance() override = default;

public:
    bool step(double) noexcept override;

private:
    MaintainDistanceStates m_stateMachine{};

};

#endif