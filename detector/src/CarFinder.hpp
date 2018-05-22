//
// Created by Adam Tonderski on 2018-05-20.
//

#ifndef DETECTOR_CARFRINDER_H
#define DETECTOR_CARFRINDER_H

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/ocl.hpp>

class CarFinder {
public:
    CarFinder(uint32_t imageWidth, uint32_t imageHeight, double scale, uint32_t numNeighbours, uint32_t maxTrackingRetries, std::string trackerType, std::string cascadeFile, bool verbose) noexcept
            : m_bbox{}
            , m_isTracking{false}
            , m_scale{scale}
            , m_numNeighbours{numNeighbours}
            , m_trackingRetries{}
            , MAX_TRACKING_RETRIES{maxTrackingRetries}
            , TRACKER_TYPE{trackerType}
            , IMAGE_WIDTH{imageWidth}
            , IMAGE_HEIGHT{imageHeight}
            , VERBOSE{verbose}
    {
        m_classifier.load(cascadeFile);
    }

    float getAngle();

    float getDistance();

    bool findCar(cv::Mat);

protected:
    bool track(cv::Mat &);

    bool detect(cv::Mat &);

    void initTracker(cv::Mat);
private:
    cv::Ptr<cv::Tracker> m_tracker;
    cv::CascadeClassifier m_classifier;
    cv::Rect2d m_bbox;
    bool m_isTracking;
    double m_scale;
    uint32_t m_numNeighbours;
    uint32_t m_trackingRetries;
    std::string const TRACKER_TYPE;
    uint32_t const MAX_TRACKING_RETRIES;
    uint32_t const IMAGE_WIDTH;
    uint32_t const IMAGE_HEIGHT;
    bool const VERBOSE;
};


#endif //DETECTOR_CARFRINDER_H
