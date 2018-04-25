//
// Created by Adam Tonderski on 2018-04-16.
//

#ifndef AUTONOMOUS_ROBOT_CONTROLLER_H
#define AUTONOMOUS_ROBOT_CONTROLLER_H

#include <mutex>
#include <limits>

#include "opendlv-standard-message-set.hpp"


class Controller {

public:
    Controller() noexcept;
    virtual ~Controller() = default;

    virtual bool step(double) noexcept = 0;

    float getGroundSteeringAngle() noexcept;
    float getPedalPosition() noexcept;
    void setFrontUltrasonic(float) noexcept;
    void setRearUltrasonic(float) noexcept;
    void setLeftIr(float) noexcept;
    void setRightIr(float) noexcept;

protected:
    double getFrontDistance() noexcept;
    double getRearDistance() noexcept;
    double getLeftDistance() noexcept;
    double getRightDistance() noexcept;
    void setPedalPosition(double) noexcept;
    void setGroundSteeringAngle(double) noexcept;

private:
    double scalePedalPosition(double) const noexcept;
    double convertIrVoltageToDistance(float) const noexcept;

    double m_frontDistance;
    double m_rearDistance;
    double m_leftDistance;
    double m_rightDistance;
    double m_groundSteeringAngle;
    double m_pedalPosition;
    std::mutex m_frontDistanceMutex;
    std::mutex m_rearDistanceMutex;
    std::mutex m_leftDistanceMutex;
    std::mutex m_rightDistanceMutex;
    std::mutex m_groundSteeringAngleMutex;
    std::mutex m_pedalPositionMutex;
};


#endif //AUTONOMOUS_ROBOT_CONTROLLER_H
