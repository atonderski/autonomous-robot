//
// Created by Adam Tonderski on 2018-04-16.
//

#ifndef AUTONOMOUS_ROBOT_MANUALCONTROLLER_H
#define AUTONOMOUS_ROBOT_MANUALCONTROLLER_H


#include "controller.hpp"

class ManualController : public Controller {

public:
    ManualController(double, double, double) noexcept;
    ~ManualController() override = default;

    bool step(double) noexcept override;

private:
    double m_manualPedalPosition;
    double m_manualSteeringAngle;
    double m_timeToRun;
    double m_currentTime;
};

#endif //AUTONOMOUS_ROBOT_MANUALCONTROLLER_H