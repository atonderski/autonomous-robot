#include <chrono>
#include <iostream>
#include <chrono>

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include "controller.hpp"
#include "manualcontroller.hpp"
#include "subsumer.hpp"
#include "pathfinder.hpp"
#include "stalker.hpp"
#include "cluon-complete-v0.0.74.hpp"


Controller& initializeController(std::map<std::string, std::string> commandlineArguments, double const DT) {
    if (commandlineArguments.count("manual") != 0) {
        double pedalMagnitude = 0.0;
        if (commandlineArguments.count("pedal") != 0) {
            pedalMagnitude = std::stod(commandlineArguments["pedal"]);
        }
        double steeringMagnitude = 0.0;
        if (commandlineArguments.count("steering") != 0) {
            steeringMagnitude = std::stod(commandlineArguments["steering"]);
        }
        double runTime = std::stod(commandlineArguments["time"]);
        bool printSensorValues = commandlineArguments.count("print") != 0;
        return *(new ManualController(DT, pedalMagnitude, steeringMagnitude, runTime, printSensorValues));
    } else if (commandlineArguments.count("pathfinder") != 0) {
        return *(new Pathfinder(DT));
    } else if (commandlineArguments.count("stalker") != 0) {
        return *(new Stalker(DT));
    } else {
        return *(new Subsumer(DT));
    }
}

int32_t main(int32_t argc, char **argv) {
    std::cout << "Hello! Activating super pro vehicle control " << std::endl;

    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if (0 == commandlineArguments.count("cid") || 0 == commandlineArguments.count("freq")) {
        std::cerr << argv[0] << " runs the super pro vehicle control by sending actuation commands and reacting to sensor input."
                  << std::endl;
        std::cerr << "Usage:   " << argv[0]
                  << " --freq=<frequency> --cid=<OpenDaVINCI session> [--verbose] " << std::endl;
        std::cerr << "Example: " << argv[0] << " --freq=10 --cid=111" << std::endl;
        return 1;
    }
    bool const VERBOSE{commandlineArguments.count("verbose") != 0};
    uint16_t const CID = static_cast<const uint16_t>(std::stoi(commandlineArguments["cid"]));
    float const FREQ = std::stof(commandlineArguments["freq"]);
    double const DT = 1.0 / FREQ;
    bool scalePedalToSimulation = commandlineArguments.count("sim") != 0;
    bool measureTimeOfStep = commandlineArguments.count("meter") != 0;

    Controller& controller = initializeController(commandlineArguments, DT);

    auto onDistanceReading{[&controller, &VERBOSE](cluon::data::Envelope &&envelope) {
        uint32_t const senderStamp = envelope.senderStamp();
        auto distanceReading = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(envelope));
        if (senderStamp == 0) {
            if (VERBOSE) {
                std::cout << "Got front distance message (from ultrasonic): " << distanceReading.distance() << std::endl;
            }
            controller.setFrontUltrasonic(distanceReading.distance());
        } else if (senderStamp == 1) {
            if (VERBOSE) {
                std::cout << "Got back distan§ce message (from ultrasonic): " << distanceReading.distance() << std::endl;
            }
            controller.setRearUltrasonic(distanceReading.distance());
        } else {
            if (VERBOSE) {
                std::cout << "Got unknown distance message (from ultrasonic): " << distanceReading.distance() << std::endl;
            }
        }
    }};
    auto onVoltageReading{[&controller, &VERBOSE](cluon::data::Envelope &&envelope) {
        uint32_t const senderStamp = envelope.senderStamp();
        auto voltageReading = cluon::extractMessage<opendlv::proxy::VoltageReading>(std::move(envelope));
        if (senderStamp == 0) {
            if (VERBOSE) {
                std::cout << "Got left voltage message (from IR): " << voltageReading.voltage() << std::endl;
            }
            controller.setLeftIr(voltageReading.voltage());
        } else if (senderStamp == 1) {
            if (VERBOSE) {
                std::cout << "Got right voltage message (from IR): " << voltageReading.voltage() << std::endl;
            }
            controller.setRightIr(voltageReading.voltage());
        } else {
            if (VERBOSE) {
                std::cout << "Got unknown voltage message (from IR): " << voltageReading.voltage() << std::endl;
            }
        }
    }};
    auto onFrame{[&controller](cluon::data::Envelope &&envelope) {
        auto frame = cluon::extractMessage<opendlv::sim::Frame>(std::move(envelope));
        controller.xMeasurment(frame.x());
        controller.yMeasurment(frame.y());
        controller.yawMeasurment(frame.yaw());
    }};
    auto onKinematicState{[&controller](cluon::data::Envelope &&envelope) {
        auto kinematicState = cluon::extractMessage<opendlv::sim::KinematicState>(std::move(envelope));
        controller.vxMeasurment(kinematicState.vx());
        controller.vyMeasurment(kinematicState.vy());
        controller.yawRateMeasurment(kinematicState.yawRate());
    }};

    cluon::OD4Session od4{CID};
    od4.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistanceReading);
    od4.dataTrigger(opendlv::proxy::VoltageReading::ID(), onVoltageReading);
    od4.dataTrigger(opendlv::sim::Frame::ID(), onFrame);
    od4.dataTrigger(opendlv::sim::KinematicState::ID(), onKinematicState);

    auto atFrequency{[&VERBOSE, &controller, &od4, &measureTimeOfStep, &scalePedalToSimulation]() -> bool {
        
        bool keepRunning;

        if (measureTimeOfStep) {
            auto start{std::chrono::steady_clock::now()};
            keepRunning = controller.step();
            auto end{std::chrono::steady_clock::now()};
            auto diff = end - start;
            std::cout << std::chrono::duration<double,std::micro>(diff).count() << " us" << std::endl;
        } else {
            keepRunning = controller.step();
        }

        opendlv::proxy::GroundSteeringRequest steeringMsg;
        steeringMsg.groundSteering(controller.getGroundSteeringAngle());
        
        opendlv::proxy::PedalPositionRequest pedalPositionMsg;
        if (scalePedalToSimulation) {
            pedalPositionMsg.position(3.0f * controller.getPedalPosition());
        } else {
            pedalPositionMsg.position(controller.getPedalPosition());
        }

        cluon::data::TimeStamp sampleTime{cluon::time::now()};
        od4.send(steeringMsg, sampleTime, 0);
        od4.send(pedalPositionMsg, sampleTime, 0);
        if (VERBOSE) {
            std::cout << "Ground steering angle is " << steeringMsg.groundSteering()
                      << " and pedal position is " << pedalPositionMsg.position() << std::endl;
        }

        return keepRunning;
    }};

    od4.timeTrigger(FREQ, atFrequency);

    return 0;
}
