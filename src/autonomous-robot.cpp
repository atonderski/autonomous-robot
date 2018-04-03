#include <chrono>
#include <iostream>

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

int32_t main(int32_t , char **) {
    std::cout << "Hello! Activating super pro vehicle control " << std::endl;

    float latestDistance;
    float latestVoltage;
    cluon::OD4Session od4{111,
        [&latestDistance, &latestVoltage](cluon::data::Envelope &&envelope) {
            // Here we should handle input data
            if (envelope.dataType() == 1090) {
                opendlv::proxy::GroundSteeringRequest steeringMsg = cluon::extractMessage<opendlv::proxy::GroundSteeringRequest>(std::move(envelope));
                std::cout << "Got ground steering request (probably from me): " << steeringMsg.groundSteering() << std::endl;
            }
            else if (envelope.dataType() == 1086) {
                opendlv::proxy::PedalPositionRequest pedalPositonMsg = cluon::extractMessage<opendlv::proxy::PedalPositionRequest>(std::move(envelope));
                std::cout << "Got pedal position request (probably from me): " << pedalPositonMsg.position() << std::endl;
            }
            else if (envelope.dataType() == 1039) {
                opendlv::proxy::DistanceReading distanceMsg = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(envelope));
                latestDistance = distanceMsg.distance();
                std::cout << "Got distance message (from ultrasonic): " << distanceMsg.distance() << std::endl;
            }
            else if (envelope.dataType() == 1037) {
                opendlv::proxy::VoltageReading voltageMsg = cluon::extractMessage<opendlv::proxy::VoltageReading>(std::move(envelope));
                latestVoltage = voltageMsg.voltage();
                std::cout << "Got voltage message (from IR): " << voltageMsg.voltage() << std::endl;
            }
            else {
                std::cout << "Got some unknown data!" << std::endl;
            }
        }};

    while (od4.isRunning()) {
        std::this_thread::sleep_for(std::chrono::duration<double>(1.0));
        // Here we should make decisions and send instructions

        opendlv::proxy::GroundSteeringRequest steeringMsg;
        float nextSteering = latestDistance < 1 ? 1 : 0;
        steeringMsg.groundSteering(nextSteering);
        od4.send(steeringMsg);

        opendlv::proxy::PedalPositionRequest pedalPositionMsg;
        float nextPosition = latestDistance < 1 ? 0.1 : 1;
        pedalPositionMsg.position(nextPosition);
        od4.send(pedalPositionMsg);
    }

    return 0;
}
