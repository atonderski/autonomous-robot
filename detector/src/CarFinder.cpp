//
// Created by Adam Tonderski on 2018-05-20.
//

#include "CarFinder.hpp"

bool CarFinder::detect(cv::Mat &frame) {
    std::vector<cv::Rect> detections;
    m_classifier.detectMultiScale(frame, detections, m_scale, m_numNeighbours, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(60, 40), cv::Size(300, 200));
    uint32_t numDetections = detections.size();
    if (numDetections == 1) {
        m_bbox = detections[0];
        return true;
    } else {
        if (VERBOSE){
            std::cout << "Found " << numDetections << " objects. Detection failed..." << std::endl;
        }
        return false;
    }
}

bool CarFinder::track(cv::Mat &frame) {
    if (!m_tracker) {
        return false;
    }
    bool ok = m_tracker->update(frame, m_bbox);
    return ok;
}

float CarFinder::getAngle() {
    return static_cast<float>(1.0 - (m_bbox.x + m_bbox.width * .5) / (IMAGE_WIDTH * 0.5));
}

float CarFinder::getDistance() {
    //this could be a tuning constant
    double lowestPoint = (m_bbox.y + m_bbox.height) / IMAGE_HEIGHT;
    return static_cast<float>((DISTANCE_THRESHOLD - lowestPoint)/DISTANCE_SCALING);
}

bool CarFinder::findCar(cv::Mat frame) {
    bool success = track(frame);
    if (success) {
        if (VERBOSE)
            std::cout << "Tracking successful!" << std::endl;
    }
    if (!success || m_framesSinceLastDetection > MAX_FRAMES_WITHOUT_DETECTION) {
        if (VERBOSE){
            std::cout << "Running full detection";
            if (success) {
                std::cout << " because it's time to update the detection" << std::endl;
            } else{
                std::cout << " because tracking failed" << std::endl;
            }
        }
        bool detectionSuccess = detect(frame);
        if (VERBOSE) {
            std::cout << "Detection completed. Success = " << detectionSuccess << std::endl;
        }
        if (detectionSuccess) {
            m_framesSinceLastDetection = 0;
            initTracker(frame);
            success = true;
        }
    }
    m_framesSinceLastDetection ++;
    if (VERBOSE && success)
        std::cout << "Target is described by bounding box: " << m_bbox << std::endl;
    return success;
}

void CarFinder::initTracker(cv::Mat frame) {
    if(m_tracker)
        m_tracker->clear();
    if (TRACKER_TYPE == "kcf")
        m_tracker = cv::TrackerKCF::create(m_kcfParams);
    else if (TRACKER_TYPE == "goturn")
        m_tracker = cv::TrackerGOTURN::create();
    else
        std::cout << "WARNING! tracker type not supported" << std::endl;
    m_tracker->init(frame, m_bbox);
}
