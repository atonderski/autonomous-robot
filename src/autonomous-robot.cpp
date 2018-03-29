#include <chrono>
#include <iostream>

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

int32_t main(int32_t , char **) {
    std::cout << "Hello! Activating super pro vehicle control " << std::endl;

    cluon::OD4Session od4{111,
        [](cluon::data::Envelope &&envelope) {
            std::cout << "Hello! Got some data!" << std::endl;
            // Here we should handle input data
            if (envelope.dataType() == 1039) {
                opendlv::proxy::DistanceReading distanceMsg = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(envelope));
                distanceMsg.distance();
            }
            if (envelope.dataType() == 1037) {
                opendlv::proxy::VoltageReading voltageMsg = cluon::extractMessage<opendlv::proxy::VoltageReading>(std::move(envelope));
                voltageMsg.voltage();
            }
        }};
    
    while (od4.isRunning()) {
        std::this_thread::sleep_for(std::chrono::duration <double >(1.0));
        // Here we should make decisions and send instructions
        // od4.send(msg);
        opendlv::proxy::GroundSteeringRequest steeringMsg;
        steeringMsg.groundSteering(1);
        od4.send(steeringMsg);

        opendlv::proxy::PedalPositionRequest pedalPositionMsg;
        pedalPositionMsg.position(0);
        od4.send(pedalPositionMsg);
    }
    
    return 0;
}
