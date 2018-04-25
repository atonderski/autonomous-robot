//
// Created by Adam Tonderski on 2018-04-16.
//

#include "simplecontroller.hpp"

bool SimpleController::step(double) noexcept {

    double frontDistance = getFrontDistance();
    double rearDistance = getRearDistance();
    double leftDistance = getLeftDistance();
    double rightDistance = getRightDistance();

//    std::cout << "front: " << frontDistance << ", rear: " << rearDistance << ", left: " << leftDistance << ", right: " << rightDistance << std::endl;

    double defaultSpeed{0.5};
    double avoidanceSpeedFactor{0.0};
    // Change speed based on proximity in front/back
    if (frontDistance < 0.5) {
        avoidanceSpeedFactor -= 1.5 * defaultSpeed * (0.5 - frontDistance) / 0.5;
    }
    if (rearDistance < 0.25) {
        avoidanceSpeedFactor += 1.5 * defaultSpeed * (0.25 - rearDistance) / 0.25;
    }
    // fall back to default speed
    double speed = defaultSpeed + avoidanceSpeedFactor;

    double avoidanceTurningFactor{0.0};

    double sideAvoidanceThreshold{0.25};
    double sideDetectionThreshold{0.32};
    // Avoid obstacles in front and back
    if (frontDistance <= 0.5 || rearDistance < 0.3) {
        if (leftDistance < rightDistance) {
            m_preferedDirection = -fabs(m_preferedDirection);
            std::cout << "Preferred direction: " << m_preferedDirection << std::endl;
        } else if (rightDistance < leftDistance) {
            m_preferedDirection = fabs(m_preferedDirection);
            std::cout << "Preferred direction: " << m_preferedDirection << std::endl;
        }
        avoidanceTurningFactor = m_preferedDirection;
    }

    bool oldIsFollowingWall = m_isFollowingWall;
    // Avoid obstacles on the sides
    m_isFollowingWall = false;
    if (leftDistance < sideDetectionThreshold) {
        std::cout << "Close on the left: " << leftDistance << std::endl;
        m_isFollowingWall = true;
        if (leftDistance < sideAvoidanceThreshold) {
            avoidanceTurningFactor -= 0.6 * (1 - leftDistance / sideAvoidanceThreshold);
        }
    }
    if (rightDistance < sideDetectionThreshold) {
        std::cout << "Close on the right: " << rightDistance << std::endl;
        m_isFollowingWall = true;
        if (rightDistance < sideAvoidanceThreshold) {
            avoidanceTurningFactor += 0.6f * (1 - rightDistance / sideAvoidanceThreshold);
        }
    }

    // If we aren't close to a wall, we have a chance to change direction
    if (!m_isFollowingWall) {
        if (oldIsFollowingWall || random() % 100 == 0) {
            m_preferedDirection = -m_preferedDirection;
            std::cout << "Changed direction to " << m_preferedDirection << std::endl;
        }
    }
    double groundSteeringAngle = fabs(avoidanceTurningFactor) < 0.05 ? m_preferedDirection : avoidanceTurningFactor;

    setGroundSteeringAngle(groundSteeringAngle);
    setPedalPosition(speed);

    return true;
}
