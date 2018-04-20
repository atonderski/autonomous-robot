#ifndef AUTONOMOUS_ROBOT_MAINTAIN_DISTANCE
#define AUTONOMOUS_ROBOT_MAINTAIN_DISTANCE

#include "controller.hpp"
#include <iostream>

class MaintainDistance : public Controller {
public:
    MaintainDistance() noexcept = default;
    ~MaintainDistance() override = default;

public:
    void step(double) noexcept override;

private:
    

};

#endif