
#include "maintainDistance.hpp"

void MaintainDistance::step(double) noexcept
{
    double frontDistance = getFrontDistance();
    //double rearDistance = getRearDistance();
    //double leftDistance = getLeftDistance();
    //double rightDistance = getRightDistance();

    double groundSteeringAngle{0.0};
    double speed;

    double const CRITICAL_DISTANCE{0.4};
    double const MARGIN{0.1};

    if (frontDistance > (CRITICAL_DISTANCE + MARGIN)) {
        speed = 0.16;
    } else if (frontDistance < (CRITICAL_DISTANCE - MARGIN)) {
        speed = -0.16;
    } else {
        speed = 0.0;
    }

    setGrounSteeringAngle(groundSteeringAngle);
    setPedalPosition(speed);

}