//
// Created by Adam Tonderski on 2018-04-16.
//

#ifndef AUTONOMOUS_ROBOT_MANUALCONTROLLER_H
#define AUTONOMOUS_ROBOT_MANUALCONTROLLER_H


#include "controller.hpp"

class ManualController : public Controller {

public:
    ManualController(double pedalPosition, double steeringAngle, double timeToRun);

    ~ManualController() override = default;

private:
    bool step(double d) noexcept override;

private:
    double pedalPosition;
    double steeringAngle;
    double timeToRun;
    double currentTime;
};


#endif //AUTONOMOUS_ROBOT_MANUALCONTROLLER_H
