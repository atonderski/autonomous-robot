//
// Created by Adam Tonderski on 2018-04-16.
//

#include "manualcontroller.hpp"


ManualController::ManualController(double pedalPosition, double steeringAngle, double timeToRun) :
        pedalPosition(pedalPosition), steeringAngle(steeringAngle), timeToRun(timeToRun), currentTime(0.0) {}


void ManualController::step(double dt) noexcept {
    // Here we should make decisions and send instructions
    if (currentTime < timeToRun) {
        setPedalPosition(pedalPosition);
        setGrounSteeringAngle(steeringAngle);
    } else {
        setPedalPosition(0.0);
        setGrounSteeringAngle(0.0);
    }
    currentTime += dt;
}
