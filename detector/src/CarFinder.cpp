//
// Created by Adam Tonderski on 2018-05-20.
//

#include "CarFinder.hpp"

bool CarFinder::detect(cv::Mat &frame) {
    std::vector<cv::Rect> detections;
    m_classifier.detectMultiScale(frame, detections, m_scale, m_numNeighbours, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(60, 40), cv::Size(300, 200));
    if (!detections.empty()) {
        m_bbox = detections[0];
        m_tracker->init(frame, m_bbox);
        m_isTracking = true;
        return true;
    } else {
        return false;
    }
}

bool CarFinder::track(cv::Mat &frame) {
    bool ok = m_tracker->update(frame, m_bbox);
    m_isTracking = ok;
    return ok;
}

float CarFinder::getAngle() {
    return static_cast<float>((m_bbox.x + m_bbox.width * .5) / (IMAGE_WIDTH * 0.5));
}

float CarFinder::getDistance() {
    return 0;
}

bool CarFinder::findCar(cv::Mat frame) {
    bool found = false;
    if (m_isTracking) {
        found = track(frame);
        if (VERBOSE) {
            if (found) {
                std::cout << "Tracking successful!" << std::endl;
            } else {
                std::cout << "Tracking failed!" << std::endl;
            }
        }
    }
    if (!found) {
        found = detect(frame);
        if (VERBOSE) {
            std::cout << "Detection completed. Success = " << found << std::endl;
        }
    }
    if (VERBOSE && found) {
        std::cout << "Target is described by bounding box: " << m_bbox << std::endl;
    }
    return found;
}
