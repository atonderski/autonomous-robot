
#include "pathfinder.hpp"


bool Pathfinder::step() noexcept {

    runFilter();

    m_stepFront = getFrontDistance();
    m_stepRear = getRearDistance();
    m_stepLeft = getLeftDistance();
    m_stepRight = getRightDistance();

    m_stepX = getX();
    m_stepY = getY();
    m_stepYaw = getYaw();

    m_behaviourReflex.update();
    m_behaviourAvoid.update();
    m_behaviourFollowPath.update();

    int pedalLogic;
    if (m_behaviourReflex.getActivationStatus()) {
        setGroundSteeringAngle(m_behaviourReflex.m_groundSteeringAngle);
        pedalLogic = m_behaviourReflex.m_pedalLogic;
    } else if (m_behaviourAvoid.getActivationStatus()) {
        setGroundSteeringAngle(m_behaviourAvoid.m_groundSteeringAngle);
        pedalLogic = m_behaviourAvoid.m_pedalLogic;
    } else if (m_behaviourFollowPath.getActivationStatus()) {
        setGroundSteeringAngle(m_behaviourFollowPath.m_groundSteeringAngle);
        pedalLogic = m_behaviourFollowPath.m_pedalLogic;
    } else {
        setGroundSteeringAngle(0);
        pedalLogic = 0;
    }

    m_accelerationRegulator.setPedalLogic(pedalLogic);
    setPedalPositionUnscaled(m_accelerationRegulator.getPedalPosition());
    return true;
}


void Pathfinder::runFilter() noexcept {

    auto measurements = getMeasurements();
    auto m_absoluteMeasurements = Eigen::VectorXd(measurements);

    // Fix relative velocities
    double yaw = getYaw();
    m_absoluteMeasurements(3) = measurements(3) * std::cos(yaw) - measurements(4) * std::sin(yaw);
    m_absoluteMeasurements(4) = measurements(3) * std::sin(yaw) + measurements(4) * std::cos(yaw);

    // Add noise
    for (int i = 0; i<6; i++) {
        m_absoluteMeasurements(i) += m_distribution(m_generator);
    }

    m_filter.update(m_absoluteMeasurements);
    auto state = m_filter.state();
    std::cout << "x: " << state(0) << " ;  y: " << state(1) << std::endl;
    setX(state(0));
    setY(state(1));
    setYaw(state(2));
}
