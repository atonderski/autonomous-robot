#include <chrono>
#include <iostream>

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

int32_t main(int32_t argc, char **argv) {
    std::cout << "Hello! Activating super pro vehicle control " << std::endl;
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    float latestDistance;
    float latestVoltage;
    cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"])),
        [&latestDistance, &latestVoltage](cluon::data::Envelope &&envelope) {
            // Here we should handle input data
            uint32_t id = envelope.senderStamp();
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
                if (id == 1) {
                    std::cout << "Got front distance message (from ultrasonic): " << distanceMsg.distance() << std::endl;
                    latestDistance = distanceMsg.distance();
                } else if (id == 2) {
                    std::cout << "Got back distance message (from ultrasonic): " << distanceMsg.distance() << std::endl;
                } else{
                    std::cout << "Got unknown distance message (from ultrasonic): " << distanceMsg.distance() << std::endl;
                }
            }
            else if (envelope.dataType() == 1037) {
                opendlv::proxy::VoltageReading voltageMsg = cluon::extractMessage<opendlv::proxy::VoltageReading>(std::move(envelope));
                if (id==1){
                    std::cout << "Got left voltage message (from IR): " << voltageMsg.voltage() << std::endl;
                    latestVoltage = voltageMsg.voltage();
                } else if (id==2){
                    std::cout << "Got right voltage message (from IR): " << voltageMsg.voltage() << std::endl;
                } else{
                    std::cout << "Got unknown voltage message (from IR): " << voltageMsg.voltage() << std::endl;
                }
            }
            else {
                std::cout << "Got some unknown data!" << std::endl;
            }
        }};

    float pedalMagnitude = std::stof(commandlineArguments["pedal"]); 
    float steeringMagnitude = std::stof(commandlineArguments["steering"]); 
    float runTime = std::stof(commandlineArguments["time"]); 

    float currentTime = 0;
    while (od4.isRunning()) {
        std::this_thread::sleep_for(std::chrono::duration<double>(0.1));
        currentTime += 0.1;
        // Here we should make decisions and send instructions

        opendlv::proxy::GroundSteeringRequest steeringMsg;
        steeringMsg.groundSteering(steeringMagnitude);
        od4.send(steeringMsg);

        opendlv::proxy::PedalPositionRequest pedalPositionMsg;
        pedalPositionMsg.position(pedalMagnitude);
        od4.send(pedalPositionMsg);
        if (currentTime > runTime){
            pedalPositionMsg.position(0.0);
            od4.send(pedalPositionMsg);
            steeringMsg.groundSteering(0.0);
            od4.send(steeringMsg);
            break;
        }
    }

    return 0;
}
