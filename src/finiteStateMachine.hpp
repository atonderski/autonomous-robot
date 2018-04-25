#ifndef FINITE_STATE_MACHINE
#define FINITE_STATE_MACHINE

#include <limits>

class FiniteStateMachine {
    public:
        FiniteStateMachine() noexcept:
            m_frontDistance{std::numeric_limits<double>::max()},
            m_rearDistance{std::numeric_limits<double>::max()},
            m_leftDistance{std::numeric_limits<double>::max()},
            m_rightDistance{std::numeric_limits<double>::max()},
            m_groundSteeringAngle{},
            m_pedalPosition{},
            activeState{}
        {
        }
        ~FiniteStateMachine() = default;        

        template <typename Container>
        void setState(void (Container::* newState)()) noexcept {
            activeState = static_cast<void(FiniteStateMachine::*)()>(newState);
        }
        void update() noexcept {
            if (activeState != 0) {
                (this->*activeState)();
            }
        }

        // 'Chaotic Evil' public members
        double m_frontDistance;
        double m_rearDistance;
        double m_leftDistance;
        double m_rightDistance;
        double m_groundSteeringAngle;
        double m_pedalPosition;

    private:
        FiniteStateMachine(FiniteStateMachine const &) = delete;
        FiniteStateMachine(FiniteStateMachine &&) = delete;
        FiniteStateMachine &operator=(FiniteStateMachine const &) = delete;
        FiniteStateMachine &operator=(FiniteStateMachine &&) = delete;
        void (FiniteStateMachine::* activeState)();
};

#endif