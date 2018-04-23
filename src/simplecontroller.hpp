//
// Created by Adam Tonderski on 2018-04-16.
//

#ifndef AUTONOMOUS_ROBOT_SIMPLECONTROLLER_H
#define AUTONOMOUS_ROBOT_SIMPLECONTROLLER_H

#include <iostream>
#include <cmath>

#include "controller.hpp"

class SimpleController : public Controller {
public:
    SimpleController() noexcept = default;

    ~SimpleController() override = default;

public:
    bool step(double) noexcept override;

private:
    float m_preferedDirection{0.5f};
    bool m_isFollowingWall{false};

};


#endif //AUTONOMOUS_ROBOT_SIMPLECONTROLLER_H
