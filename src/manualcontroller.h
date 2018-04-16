//
// Created by Adam Tonderski on 2018-04-16.
//

#ifndef AUTONOMOUS_ROBOT_MANUALCONTROLLER_H
#define AUTONOMOUS_ROBOT_MANUALCONTROLLER_H


#include "controller.h"

class ManualController : public Controller {

public:
    ManualController(double pedalPosition, double steeringAngle, double timeToRun);
    virtual ~ManualController() = default;

private:
    void step(double d) noexcept override;

private:
    double pedalPosition;
    double steeringAngle;
    double timeToRun;
    double currentTime;
};


#endif //AUTONOMOUS_ROBOT_MANUALCONTROLLER_H
