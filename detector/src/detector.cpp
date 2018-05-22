/*
 * Copyright (C) 2018 Ola Benderius
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <cstdint>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include "CarFinder.hpp"


int32_t main(int32_t argc, char **argv) {
    int32_t retCode{0};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ((0 == commandlineArguments.count("name")) || (0 == commandlineArguments.count("cid"))) {
        std::cerr << argv[0] << " accesses video data using shared memory provided using the command line parameter --name=." << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid=<OpenDaVINCI session> --name=<name for the associated shared memory> [--id=<sender stamp>] [--verbose]" << std::endl;
        std::cerr << "         --name:    name of the shared memory to use" << std::endl;
        std::cerr << "         --verbose: when set, more information is printed" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=111 --name=cam0" << std::endl;
        retCode = 1;
    } else {
        bool const VERBOSE{commandlineArguments.count("verbose") != 0};
        uint32_t const ID{(commandlineArguments["id"].size() != 0) ? static_cast<uint32_t>(std::stoi(commandlineArguments["id"])) : 0};
        double const scale{(commandlineArguments["scale"].size() != 0) ? static_cast<double>(std::stod(commandlineArguments["scale"])) : 1.1};
        uint32_t const numNeighbours{(commandlineArguments["num-neighbours"].size() != 0) ? static_cast<uint32_t>(std::stoi(commandlineArguments["num-neighbours"])) : 30};
        std::string const NAME{(commandlineArguments["name"].size() != 0) ? commandlineArguments["name"] : "/cam0"};
        std::string const TRACKER{(commandlineArguments["tracker"].size() != 0) ? commandlineArguments["tracker"] : "kcf"};
        uint32_t const SCALED_WIDTH{(commandlineArguments["scaled-w"].size() != 0) ? static_cast<uint32_t>(std::stoi(commandlineArguments["scaled-w"])) : 640};
        uint32_t const SCALED_HEIGHT{(commandlineArguments["scaled-h"].size() != 0) ? static_cast<uint32_t>(std::stoi(commandlineArguments["scaled-h"])) : 480};

        uint32_t const WIDTH{1280};
        uint32_t const HEIGHT{960};
        uint32_t const BPP{24};

        cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};


        std::unique_ptr <cluon::SharedMemory> sharedMemory(new cluon::SharedMemory{NAME});
        if (sharedMemory && sharedMemory->valid()) {
            std::clog << argv[0] << ": Found shared memory '" << sharedMemory->name() << "' (" << sharedMemory->size() << " bytes)." << std::endl;

            CvSize size;
            size.width = WIDTH;
            size.height = HEIGHT;

            IplImage *image = cvCreateImageHeader(size, IPL_DEPTH_8U, BPP / 8);
            sharedMemory->lock();
            image->imageData = sharedMemory->data();
            image->imageDataOrigin = image->imageData;
            sharedMemory->unlock();

            CarFinder carFinder{SCALED_WIDTH, SCALED_HEIGHT, scale, numNeighbours, TRACKER, VERBOSE};

            int32_t i = 0;
            while (od4.isRunning()) {
                sharedMemory->wait();

                // Make a scaled copy of the original image.
                cv::Mat scaledImage;
                {
                    sharedMemory->lock();
                    cv::Mat sourceImage = cv::cvarrToMat(image, false);
                    cv::resize(sourceImage, scaledImage, cv::Size(SCALED_WIDTH, SCALED_HEIGHT), 0, 0, cv::INTER_NEAREST);
                    sharedMemory->unlock();
                }

                // Make an estimation.
                bool objectFound = carFinder.findCar(scaledImage);

                if (objectFound) {
                    float estimatedDetectionAngle = carFinder.getAngle();
                    float estimatedDetectionDistance = carFinder.getDistance();

                    if (VERBOSE) {
//                        std::string const FILENAME = std::to_string(i) + ".jpg";
//                        cv::imwrite(FILENAME, scaledImage);
//                        i++;
//                        std::this_thread::sleep_for(std::chrono::seconds(1));
                        std::cout << "The target was found at angle " << estimatedDetectionAngle
                                  << " and distance " << estimatedDetectionDistance << std::endl;
                    }

                    // In the end, send a message that is received by the control logic.
                    opendlv::logic::sensation::Point detection;
                    detection.azimuthAngle(estimatedDetectionAngle);
                    detection.distance(estimatedDetectionDistance);

                    od4.send(detection, cluon::time::now(), ID);
                }
            }

            cvReleaseImageHeader(&image);
        } else {
            std::cerr << argv[0] << ": Failed to access shared memory '" << NAME << "'." << std::endl;
        }
    }
    return retCode;
}

