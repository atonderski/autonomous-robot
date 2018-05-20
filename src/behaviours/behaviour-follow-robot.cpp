#include "behaviour-follow-robot.hpp"

void BehaviourFollowRobot::universalState() noexcept {
    // this state is executed every cycle regardless of conditions
    // suitable for things like counting down with timesteps
    // can be left blank but NOT deleted
}

void BehaviourFollowRobot::initialState() noexcept {
    // Initial State is also counted as an inactive part of the behaviour.
    // When inactive, a behaviour will pass controll to the next active one in the hiearchy.
    if (targetVisible) { setState(&BehaviourFollowRobot::atTargetPositionState); }
}

void BehaviourFollowRobot::atTargetPositionState() noexcept {

}

void BehaviourFollowRobot::followTurnLeft() noexcept {

}

void BehaviourFollowRobot::followTurnRight() noexcept {

}