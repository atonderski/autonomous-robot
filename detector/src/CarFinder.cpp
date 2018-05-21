//
// Created by Adam Tonderski on 2018-05-20.
//

#include "CarFinder.h"

CarFinder::CarFinder(uint32_t imageWidth) : m_bbox{}, m_isTracking{false}, IMAGE_WIDTH{imageWidth} {
    m_tracker = cv::TrackerKCF::create();
    m_classifier.load("../data/cascade.xml");
}

bool CarFinder::detect(cv::Mat &frame) {
    std::vector<cv::Rect> detections;
    m_classifier.detectMultiScale(frame, detections, 1.1, 30, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(60, 40), cv::Size(300, 200));
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
    }
    if (!found) {
        found = detect(frame);
    }
    return found;

}
