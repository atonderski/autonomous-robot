#include "behaviour-follow-robot.hpp"

void BehaviourFollowRobot::universalState() noexcept {
    // this state is executed every cycle regardless of conditions
    // this state is executed BEFORE any of the other states
    // suitable for things like counting down with timesteps
    // can be left blank but NOT deleted

    // If angle is greater than a constant, turn maximum, otherwise try to steer proportional to angle (smoothly)
    if (std::abs(*m_pDetectionAngle) > SMOOTH_STEERING_THRESHOLD) {
        if (*m_pDetectionAngle > 0) {
            m_groundSteeringAngle = 0.5;
        } else {
            m_groundSteeringAngle = -0.5;
        } 
    } else {
        m_groundSteeringAngle = (*m_pDetectionAngle) * 0.5 / SMOOTH_STEERING_THRESHOLD; // 0.5 is max steering signal magnitude
    }
    
}

void BehaviourFollowRobot::initialState() noexcept {
    // Initial State is also counted as an inactive part of the behaviour.
    // When inactive, a behaviour will pass controll to the next active one in the hiearchy.
    if (targetVisible) { setState(&BehaviourFollowRobot::atAlignedPositionState); }
}

// Maintain preferred ditance within a margin
void BehaviourFollowRobot::atAlignedPositionState() noexcept {
    m_pedalLogic = 0;

    double estimated_distance;
    if (std::abs(*m_pDetectionAngle) < ULTRASONIC_ANGLE_THRESHOLD)
        estimated_distance = *m_pFrontDistance;
    else
        estimated_distance = *m_pDetectionDistance;

    if (estimated_distance < PREFERRED_DISTANCE - MARGIN) {
        setState(&BehaviourFollowRobot::followBackward);
    } else if ((estimated_distance > PREFERRED_DISTANCE + MARGIN)) {
        setState(&BehaviourFollowRobot::followForward);
    }
}

void BehaviourFollowRobot::followForward() noexcept {
    m_pedalLogic = 1;
    if (*m_pDetectionDistance < PREFERRED_DISTANCE + MARGIN) {
        setState(&BehaviourFollowRobot::atAlignedPositionState);
    }
}

void BehaviourFollowRobot::followBackward() noexcept {
    m_pedalLogic = -1;
    m_groundSteeringAngle = (-1) * m_groundSteeringAngle; // Flip direction of steering because we're going in reverse. Steering is calculated every turn in the universal state.
    if (*m_pDetectionDistance > PREFERRED_DISTANCE - MARGIN) {
        setState(&BehaviourFollowRobot::atAlignedPositionState); // Stop reversing as soon as we are within the margin of preferred distance
    }
}
