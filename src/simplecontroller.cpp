//
// Created by Adam Tonderski on 2018-04-16.
//

#include "simplecontroller.hpp"

bool SimpleController::step(double) noexcept {

    double frontDistance = getFrontDistance();
    double rearDistance = getRearDistance();
    double leftDistance = getLeftDistance();
    double rightDistance = getRightDistance();

    float avoidanceSpeedFactor{0.0f};
    // Change speed based on proximity in front/back
    if (frontDistance < 0.5f) {
        avoidanceSpeedFactor -= 0.1f * (0.5f - frontDistance) / 0.5f;
    }
    if (rearDistance < 0.5f) {
        avoidanceSpeedFactor += 0.1f * (0.5f - rearDistance) / 0.5f;
    }
    // add the default speed
    float speed = avoidanceSpeedFactor + 0.15f;

    float avoidanceTurningFactor{0.f};

    // Avoid obstacles in front and back
    if (frontDistance < 0.5f || rearDistance < 0.5f) {
        if (leftDistance < rightDistance) {
            m_preferedDirection = -fabs(m_preferedDirection);
            std::cout << "Preferred direction: " << m_preferedDirection << std::endl;
        } else {
            m_preferedDirection = fabs(m_preferedDirection);
            std::cout << "Preferred direction: " << m_preferedDirection << std::endl;
        }
        avoidanceTurningFactor = m_preferedDirection;
    }

    bool oldIsFollowingWall = m_isFollowingWall;
    // Avoid obstacles on the sides
    double sideAvoidanceThreshold{0.25f};
    double sideDetectionThreshold{0.32f};
    m_isFollowingWall = false;
    if (leftDistance < sideDetectionThreshold) {
        std::cout << "Close on the left: " << leftDistance << std::endl;
        m_isFollowingWall = true;
        if (leftDistance < sideAvoidanceThreshold) {
            avoidanceTurningFactor -= 0.4 * (1 - leftDistance / sideAvoidanceThreshold);
        }
    }
    if (rightDistance < sideDetectionThreshold) {
        std::cout << "Close on the right: " << rightDistance << std::endl;
        m_isFollowingWall = true;
        if (rightDistance < sideAvoidanceThreshold) {
            avoidanceTurningFactor += 0.4f * (1 - rightDistance / sideAvoidanceThreshold);
        }
    }

    // if we aren't close to a wall, we have a chance to change direction
    if (!m_isFollowingWall) {
        if (oldIsFollowingWall || random() % 50 == 0) {
            m_preferedDirection = -m_preferedDirection;
            std::cout << "Changed direction to " << m_preferedDirection << std::endl;
        }
    }
    float groundSteeringAngle = avoidanceTurningFactor == 0.0f ? m_preferedDirection : avoidanceTurningFactor;

    setGroundSteeringAngle(groundSteeringAngle);
    setPedalPosition(speed);

    return true;
}
