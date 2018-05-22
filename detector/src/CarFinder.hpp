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
    CarFinder(uint32_t imageWidth, double scale, uint32_t numNeighbours, string trackerType, bool verbose) noexcept
            : m_bbox{}
            , m_isTracking{false}
            , m_scale{scale}
            , m_numNeighbours{numNeighbours}
            , IMAGE_WIDTH{imageWidth}
            , VERBOSE{verbose}
    {
        if (trackerType == "kcf")
            m_tracker = TrackerKCF::create();
        else if (trackerType == "goturn")
            m_tracker = TrackerGOTURN::create();
        else
            throw std::invalid_argument("tracker type not supported");

        m_classifier.load("/usr/share/cascade.xml");
    }

    float getAngle();

    float getDistance();

    bool findCar(cv::Mat);

protected:
    bool track(cv::Mat &);

    bool detect(cv::Mat &);

private:
    cv::Ptr<cv::Tracker> m_tracker;
    cv::CascadeClassifier m_classifier;
    cv::Rect2d m_bbox;
    bool m_isTracking;
    double m_scale;
    uint32_t m_numNeighbours;
    uint32_t const IMAGE_WIDTH;
    bool const VERBOSE;
};


#endif //DETECTOR_CARFRINDER_H
