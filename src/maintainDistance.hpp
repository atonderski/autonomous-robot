#ifndef AUTONOMOUS_ROBOT_MAINTAIN_DISTANCE
#define AUTONOMOUS_ROBOT_MAINTAIN_DISTANCE

#include "controller.hpp"
#include <limits>
#include <iostream>

class UnlimitedStateWorks{
    public:
        void initialState() noexcept;
        void update() noexcept;
        
        UnlimitedStateWorks() noexcept;
        ~UnlimitedStateWorks() = default;

        void stateCloseDistance() noexcept;
        void stateMakeDistance() noexcept;
        void stateCorrectDistance() noexcept;

        double m_frontDistance;
        double m_rearDistance;
        double m_leftDistance;
        double m_rightDistance;
        double m_groundSteeringAngle;
        double m_pedalPosition;
    
    private:
        UnlimitedStateWorks(UnlimitedStateWorks const &) = delete;
        UnlimitedStateWorks(UnlimitedStateWorks &&) = delete;
        UnlimitedStateWorks &operator=(UnlimitedStateWorks const &) = delete;
        UnlimitedStateWorks &operator=(UnlimitedStateWorks &&) = delete;
        
        void (UnlimitedStateWorks::* activeState)();
        double const CRITICAL_DISTANCE;
        double const MARGIN;
};

class MaintainDistance : public Controller {
public:
    MaintainDistance() noexcept = default;
    ~MaintainDistance() override = default;

public:
    void step(double) noexcept override;

private:
    UnlimitedStateWorks m_satanicMill{};

};

#endif