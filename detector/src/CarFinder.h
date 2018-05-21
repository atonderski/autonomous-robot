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

    CarFinder(uint32_t);

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
    const uint32_t IMAGE_WIDTH;
};


#endif //DETECTOR_CARFRINDER_H
