//
// Created by Adam Tonderski on 2018-04-16.
//

#include "controller.hpp"


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
    std::lock_guard<std::mutex> lock(m_pedalPositionMutex);
    m_pedalPosition = newPedalPosition;
}

void Controller::setGrounSteeringAngle(double newGroundSteeringAngle) noexcept {
    std::lock_guard<std::mutex> lock(m_groundSteeringAngleMutex);
    m_groundSteeringAngle = newGroundSteeringAngle;
}

// TODO: This is a rough estimate, improve by looking into the sensor specifications.
double Controller::convertIrVoltageToDistance(float voltage) const noexcept {
    double voltageDividerR1 = 1000.0;
    double voltageDividerR2 = 1000.0;
    double sensorVoltageLowerLimit = 0.3;
    double sensorVoltageUpperLimit = 3.0;

    double sensorVoltage = (voltageDividerR1 + voltageDividerR2) / voltageDividerR2 * voltage;

    double distance; // meters
    if (sensorVoltage < sensorVoltageLowerLimit) {
      distance = std::numeric_limits<double>::max();
    } else if (sensorVoltage > sensorVoltageUpperLimit) {
      distance = 0;
    } else {
      distance = (13.3113/sensorVoltage - 0.5616)/100;
    }

    return distance;
}
