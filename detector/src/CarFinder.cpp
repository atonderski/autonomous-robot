//
// Created by Adam Tonderski on 2018-05-20.
//

#include "CarFinder.hpp"

bool CarFinder::detect(cv::Mat &frame) {
    std::vector<cv::Rect> detections;
    m_classifier.detectMultiScale(frame, detections, m_scale, m_numNeighbours, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(60, 40), cv::Size(300, 200));
    if (!detections.empty()) {
        m_bbox = detections[0];
        return true;
    } else {
        return false;
    }
}

bool CarFinder::track(cv::Mat &frame) {
    bool ok = m_tracker->update(frame, m_bbox);
    return ok;
}

float CarFinder::getAngle() {
    return static_cast<float>(1.0 - (m_bbox.x + m_bbox.width * .5) / (IMAGE_WIDTH * 0.5));
}

float CarFinder::getDistance() {
    return static_cast<float>(m_bbox.height / IMAGE_HEIGHT);
}

bool CarFinder::findCar(cv::Mat &frame) {
    bool found = false;
    if (m_isTracking) {
        found = track(frame);
        if (found) {
            m_trackingRetries = 0;
            if (VERBOSE)
                std::cout << "Tracking successful!" << std::endl;
        } else {
            m_trackingRetries++;
            if (m_trackingRetries < _m_maxTrackingRetries)
                m_isTracking = false;
                if (VERBOSE)
                    std::cout << "Tracking failed, retrying next frame!" << std::endl;
            else if (VERBOSE)
                std::cout << "Tracking failed, no more retries!" << std::endl;
        }
    }
    if (!m_isTracking) {
        if (VERBOSE){
            std::cout << "Running full detection" << std::endl;
        }
        found = detect(frame);
        if (VERBOSE) {
            std::cout << "Detection completed. Success = " << found << std::endl;
        }
    }
    if (found) {
        if (VERBOSE)
            std::cout << "Target is described by bounding box: " << m_bbox << std::endl;
        initTracker(frame);
        m_trackingRetries = 0;
        m_isTracking = true;
    }
    return found;
}

void CarFinder::initTracker(cv::Mat &frame) {
    if (TRACKER_TYPE == "kcf")
        m_tracker = cv::TrackerKCF::create();
    else if (TRACKER_TYPE == "goturn")
        m_tracker = cv::TrackerGOTURN::create();
    else
        std::cout << "WARNING! tracker type not supported" << std::endl;
    m_tracker->init(frame, m_bbox);

}
