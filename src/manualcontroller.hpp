//
// Created by Adam Tonderski on 2018-04-16.
//

#ifndef AUTONOMOUS_ROBOT_MANUALCONTROLLER_H
#define AUTONOMOUS_ROBOT_MANUALCONTROLLER_H


#include "controller.hpp"
#include <iostream>
#include <iomanip>

class ManualController : public Controller {

public:
    ManualController(double const DT, double pedalPosition, double steeringAngle, double timeToRun, bool printSensorValues) noexcept
        : Controller(DT)
        , m_manualPedalPosition{pedalPosition}
        , m_manualSteeringAngle{steeringAngle}
        , m_timeToRun{timeToRun}
        , m_currentTime{0.0}
        , m_printSensorValues{printSensorValues}
        , m_delayTimer{0}
    {
    }
    ~ManualController() override = default;

    bool step() noexcept override;
    void printSensorValues() noexcept;

private:
    double m_manualPedalPosition;
    double m_manualSteeringAngle;
    double m_timeToRun;
    double m_currentTime;
    bool m_printSensorValues;
    double m_delayTimer;
};

#endif //AUTONOMOUS_ROBOT_MANUALCONTROLLER_H