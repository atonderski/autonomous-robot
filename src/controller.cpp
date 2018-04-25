//
// Created by Adam Tonderski on 2018-04-16.
//

#include "controller.hpp"

Controller::Controller() noexcept:
    m_frontDistance{std::numeric_limits<double>::max()},
    m_rearDistance{std::numeric_limits<double>::max()},
    m_leftDistance{std::numeric_limits<double>::max()},
    m_rightDistance{std::numeric_limits<double>::max()},
    m_groundSteeringAngle{},
    m_pedalPosition{},
    m_frontDistanceMutex{},
    m_rearDistanceMutex{},
    m_leftDistanceMutex{},
    m_rightDistanceMutex{},
    m_groundSteeringAngleMutex{},
    m_pedalPositionMutex{}
{
}

float Controller::getGroundSteeringAngle() noexcept {
    std::lock_guard<std::mutex> lock(m_groundSteeringAngleMutex);
    return static_cast<float>(m_groundSteeringAngle);
}

float Controller::getPedalPosition() noexcept {
    std::lock_guard<std::mutex> lock(m_pedalPositionMutex);
    return static_cast<float>(m_pedalPosition);
}

void Controller::setFrontUltrasonic(float frontUltrasonicReading) noexcept {
    std::lock_guard<std::mutex> lock(m_frontDistanceMutex);
    m_frontDistance = frontUltrasonicReading;
}

void Controller::setRearUltrasonic(float rearUltrasonicReading) noexcept {
    std::lock_guard<std::mutex> lock(m_rearDistanceMutex);
    m_rearDistance = rearUltrasonicReading;
}

void Controller::setLeftIr(float leftIrReading) noexcept {
    std::lock_guard<std::mutex> lock(m_leftDistanceMutex);
    m_leftDistance = convertIrVoltageToDistance(leftIrReading);
}

void Controller::setRightIr(float rightIrReading) noexcept {
    std::lock_guard<std::mutex> lock(m_rightDistanceMutex);
    m_rightDistance = convertIrVoltageToDistance(rightIrReading);
}


double Controller::getFrontDistance() noexcept {
    std::lock_guard<std::mutex> lock(m_frontDistanceMutex);
    return m_frontDistance;
}

double Controller::getRearDistance() noexcept {
    std::lock_guard<std::mutex> lock(m_rearDistanceMutex);
    return m_rearDistance;
}

double Controller::getLeftDistance() noexcept {
    std::lock_guard<std::mutex> lock(m_leftDistanceMutex);
    return m_leftDistance;
}

double Controller::getRightDistance() noexcept {
    std::lock_guard<std::mutex> lock(m_rightDistanceMutex);
    return m_rightDistance;
}

void Controller::setPedalPosition(double newPedalPosition) noexcept {
    newPedalPosition = scalePedalPosition(newPedalPosition);
    std::lock_guard<std::mutex> lock(m_pedalPositionMutex);
    m_pedalPosition = newPedalPosition;
}

void Controller::setGroundSteeringAngle(double newGroundSteeringAngle) noexcept {
    std::lock_guard<std::mutex> lock(m_groundSteeringAngleMutex);
    m_groundSteeringAngle = newGroundSteeringAngle;
}

/*
    scalePedalPosition fits the requested pedal position value given in a range
    from -1 to +1 to effective range of the robot motors.
    Forward range: [0.12,0.20]
    Reverse range: [-0.46,-0.80]
    Stationary requires exactly 0
*/
double Controller::scalePedalPosition(double logicPedalPosition) const noexcept {
    if (logicPedalPosition > 0) {
        return logicPedalPosition * 0.08 + 0.12;
    } else if (logicPedalPosition < 0) {
        return logicPedalPosition * 0.34 - 0.46;
    } else {
        return 0;
    }
}

double Controller::convertIrVoltageToDistance(float voltage) const noexcept {
    // double voltageDividerR1 = 1000.0;
    // double voltageDividerR2 = 1000.0;
    // double sensorVoltageLowerLimit = 0.3;
    // double sensorVoltageUpperLimit = 3.0;
    // double sensorVoltage = (voltageDividerR1 + voltageDividerR2) / voltageDividerR2 * voltage;

    double sensorVoltage = 2.0 * voltage;

    double distance;
    if (sensorVoltage < 0.3) {
      distance = std::numeric_limits<double>::max();
    } else if (sensorVoltage > 3.0) {
      distance = 0.0;
    } else {
      distance = (13.3113/sensorVoltage - 0.5616)/100;
    }

    return distance;
}
