#include <chrono>
#include <iostream>

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

int32_t main(int32_t , char **) {
    std::cout << "Hello! Activating super pro vehicle control " << std::endl;

    cluon::OD4Session od4{111,
        [](cluon::data::Envelope &&envelope) {
            if (envelope.dataType() == 2001) {
                std::cout << "Hello! Got some data!" << std::endl;
                // Here we should handle input data
            }
        }};
    
    while (od4.isRunning()) {
        std::this_thread::sleep_for(std::chrono::duration <double >(1.0));
        // Here we should make decisions and send instructions
        // od4.send(msg);
    }
    
    return 0;
}
