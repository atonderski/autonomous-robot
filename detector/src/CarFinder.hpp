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
    CarFinder(uint32_t imageWidth, uint32_t imageHeight, double scale, uint32_t numNeighbours, uint32_t maxFramesWithoutDetection, double distanceThreshold, double distanceScaling,
              std::string trackerType, std::string cascadeFile, bool verbose, float kcf_detect_thresh, float kcf_interp_factor, uint32_t kcf_compressed_size ) noexcept
            : m_bbox{}
            , m_scale{scale}
            , m_numNeighbours{numNeighbours}
            , m_framesSinceLastDetection{}
            , m_kcfParams{}
            , MAX_FRAMES_WITHOUT_DETECTION{maxFramesWithoutDetection}
            , DISTANCE_THRESHOLD{distanceThreshold}
            , DISTANCE_SCALING{distanceScaling}
            , TRACKER_TYPE{trackerType}
            , IMAGE_WIDTH{imageWidth}
            , IMAGE_HEIGHT{imageHeight}
            , VERBOSE{verbose}
    {
        m_classifier.load(cascadeFile);
        m_kcfParams.detect_thresh = kcf_detect_thresh;
        m_kcfParams.interp_factor = kcf_interp_factor;
        m_kcfParams.compressed_size = kcf_compressed_size;
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
    cv::TrackerKCF::Params m_kcfParams;
    cv::CascadeClassifier m_classifier;
    cv::Rect2d m_bbox;
    double m_scale;
    uint32_t m_numNeighbours;
    uint32_t m_framesSinceLastDetection;
    std::string const TRACKER_TYPE;
    uint32_t const MAX_FRAMES_WITHOUT_DETECTION;
    double const DISTANCE_THRESHOLD;
    double const DISTANCE_SCALING;
    uint32_t const IMAGE_WIDTH;
    uint32_t const IMAGE_HEIGHT;
    bool const VERBOSE;
};


#endif //DETECTOR_CARFRINDER_H
