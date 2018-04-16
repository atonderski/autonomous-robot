//
// Created by Adam Tonderski on 2018-04-16.
//

#ifndef AUTONOMOUS_ROBOT_SIMPLECONTROLLER_H
#define AUTONOMOUS_ROBOT_SIMPLECONTROLLER_H


#include "controller.h"

class SimpleController : public Controller {
public:
    SimpleController() noexcept = default;
    virtual ~SimpleController() = default;

public:
    void step(double) noexcept override;


};


#endif //AUTONOMOUS_ROBOT_SIMPLECONTROLLER_H
