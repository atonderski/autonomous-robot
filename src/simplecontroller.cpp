//
// Created by Adam Tonderski on 2018-04-16.
//

#include "simplecontroller.hpp"

void SimpleController::step(double) noexcept {

    double frontDistance = getFrontDistance();
    double rearDistance = getRearDistance();
    double leftDistance = getLeftDistance();
    double rightDistance = getRightDistance();

    double pedalPosition = 0.14;
    double groundSteeringAngle = 0.3;
    if (frontDistance < 0.3) {
        pedalPosition = 0.0;
    } else {
        if (rearDistance < 0.3) {
            pedalPosition = 0.0;
        }
    }

    if (leftDistance < rightDistance) {
        if (leftDistance < 0.2) {
            groundSteeringAngle = -0.2;
        }
    } else {
        if (rightDistance < 0.2) {
            groundSteeringAngle = 0.2;
        }
    }

    setGrounSteeringAngle(groundSteeringAngle);
    setPedalPosition(pedalPosition);
}
