//
// Created by Adam Tonderski on 2018-04-16.
//

#ifndef AUTONOMOUS_ROBOT_CONTROLLER_H
#define AUTONOMOUS_ROBOT_CONTROLLER_H

#include <mutex>
#include <limits>
#include <iostream>

#include "opendlv-standard-message-set.hpp"
#include "kalman.hpp"

class Controller {
public:
    Controller(double const DT) noexcept
    : m_dt{DT}
    , m_frontDistance{std::numeric_limits<double>::max()}
    , m_rearDistance{std::numeric_limits<double>::max()}
    , m_leftVoltage{}
    , m_rightVoltage{}
    , m_groundSteeringAngle{}
    , m_pedalPosition{}
    , m_x{}
    , m_y{}
    , m_yaw{}
    , m_measurments{}
    , m_filter{}
    , m_frontDistanceMutex{}
    , m_rearDistanceMutex{}
    , m_leftVoltageMutex{}
    , m_rightVoltageMutex{}
    , m_groundSteeringAngleMutex{}
    , m_pedalPositionMutex{}
    , m_measurmentsMutex{}
    {
        int n = 6; // Number of states
        int m = 6; // Number of measurements

        m_measurments = Eigen::VectorXd(m); // measurement vector

        Eigen::MatrixXd A(n, n); // System dynamics matrix
        Eigen::MatrixXd C(m, n); // Output matrix
        Eigen::MatrixXd Q(n, n); // Process noise covariance
        Eigen::MatrixXd R(m, m); // Measurement noise covariance
        Eigen::MatrixXd P(n, n); // Estimate error covariance

        // Discrete LTI projectile motion, measuring position only
        A <<    1, 0, 0, DT, 0, 0,
                0, 1, 0, 0, DT, 0,
                0, 0, 1, 0, 0, DT,
                0, 0, 0, 1, 0, 0,
                0, 0, 0, 0, 1, 0,
                0, 0, 0, 0, 0, 1;

        // Output matrix is diagonal since we measure all quantities directly
        C = Eigen::MatrixXd::Identity(m,n);

        // Assume diagonal noise matrices
        Q = Eigen::MatrixXd::Identity(n,n) * .05;
        R = Eigen::MatrixXd::Identity(m,m) * 2;
        P = Eigen::MatrixXd::Identity(n,n) * 2;

        std::cout << "A: \n" << A << std::endl;
        std::cout << "C: \n" << C << std::endl;
        std::cout << "Q: \n" << Q << std::endl;
        std::cout << "R: \n" << R << std::endl;
        std::cout << "P: \n" << P << std::endl;

        // Construct and init the filter
        m_filter = KalmanFilter(DT, A, C, Q, R, P);
        m_filter.init();
    }

    virtual ~Controller() = default;

    virtual bool step() noexcept = 0;

    float getGroundSteeringAngle() noexcept {
        std::lock_guard<std::mutex> lock(m_groundSteeringAngleMutex);
        return static_cast<float>(m_groundSteeringAngle);
    }

    float getPedalPosition() noexcept {
        std::lock_guard<std::mutex> lock(m_pedalPositionMutex);
        return static_cast<float>(m_pedalPosition);
    }

    void setFrontUltrasonic(float frontUltrasonicReading) noexcept {
        std::lock_guard<std::mutex> lock(m_frontDistanceMutex);
        m_frontDistance = frontUltrasonicReading;
    }

    void setRearUltrasonic(float rearUltrasonicReading) noexcept {
        std::lock_guard<std::mutex> lock(m_rearDistanceMutex);
        m_rearDistance = rearUltrasonicReading;
    }

    void setLeftIr(float leftIrReading) noexcept {
        std::lock_guard<std::mutex> lock(m_leftVoltageMutex);
        m_leftVoltage = leftIrReading;
    }

    void setRightIr(float rightIrReading) noexcept {
        std::lock_guard<std::mutex> lock(m_rightVoltageMutex);
        m_rightVoltage = rightIrReading;
    }

    void xMeasurment(float xReading) noexcept {
        std::lock_guard<std::mutex> lock(m_measurmentsMutex);
        m_measurments(0) = xReading;
    }

    void yMeasurment(float yReading) noexcept {
        std::lock_guard<std::mutex> lock(m_measurmentsMutex);
        m_measurments(1) = yReading;
    }

    void yawMeasurment(float yawReading) noexcept {
        std::lock_guard<std::mutex> lock(m_measurmentsMutex);
        m_measurments(2) = yawReading;
    }

    void vxMeasurment(float vxReading) noexcept {
        std::lock_guard<std::mutex> lock(m_measurmentsMutex);
        m_measurments(3) = vxReading;
    }

    void vyMeasurment(float vyReading) noexcept {
        std::lock_guard<std::mutex> lock(m_measurmentsMutex);
        m_measurments(4) = vyReading;
    }

    void yawRateMeasurment(float yawRateReading) noexcept {
        std::lock_guard<std::mutex> lock(m_measurmentsMutex);
        m_measurments(5) = yawRateReading;
    }

    void runFilter() noexcept {
        std::lock_guard<std::mutex> lock(m_measurmentsMutex);
        m_filter.update(m_measurments);
        auto state = m_filter.state();
        m_x = state(0);
        m_y = state(1);
        m_yaw = state(2);
    }

protected:
    inline double getFrontDistance() noexcept {
        std::lock_guard<std::mutex> lock(m_frontDistanceMutex);
        return m_frontDistance;
    }

    inline double getRearDistance() noexcept {
        std::lock_guard<std::mutex> lock(m_rearDistanceMutex);
        return m_rearDistance;
    }

    inline double getLeftDistance() noexcept {
        float leftVoltageTmp;
        {
            std::lock_guard<std::mutex> lock(m_leftVoltageMutex);
            leftVoltageTmp = m_leftVoltage;
        }
        return convertIrVoltageToDistance(leftVoltageTmp);
    }

    inline double getRightDistance() noexcept {
        float rightVoltageTmp;
        {
            std::lock_guard<std::mutex> lock(m_rightVoltageMutex);
            rightVoltageTmp = m_rightVoltage;
        }
        return convertIrVoltageToDistance(rightVoltageTmp);
    }

    inline double getX() noexcept {
        return m_x;
    }

    inline double getY() noexcept {
        return m_y;
    }

    inline double getYaw() noexcept {
        return m_yaw;
    }

    inline void setPedalPosition(double newPedalPosition) noexcept {
        newPedalPosition = scalePedalPosition(newPedalPosition);
        std::lock_guard<std::mutex> lock(m_pedalPositionMutex);
        m_pedalPosition = newPedalPosition;
    }

    inline void setPedalPositionUnscaled(double newPedalPosition) noexcept {
        std::lock_guard<std::mutex> lock(m_pedalPositionMutex);
        m_pedalPosition = newPedalPosition;
    }

    inline void setGroundSteeringAngle(double newGroundSteeringAngle) noexcept {
        std::lock_guard<std::mutex> lock(m_groundSteeringAngleMutex);
        m_groundSteeringAngle = newGroundSteeringAngle;
    }

    double const m_dt;

private:
    inline double convertIrVoltageToDistance(float voltage) const noexcept {
        double distance;
        if (voltage < 0.15f) {
            distance = 100.0;
        } else if (voltage > 1.5f) {
            distance = 0;
        } else {
            distance = (6.65565 / voltage - 0.5616) / 100;
        }
        return distance;
    }

    double scalePedalPosition(double logicPedalPosition) const noexcept {
        if (logicPedalPosition > 0) {
            return logicPedalPosition * 0.09 + 0.11;
        } else if (logicPedalPosition < 0) {
            return logicPedalPosition * 0.18 - 0.45;
        } else {
            return 0;
        }
    }

    double m_frontDistance;
    double m_rearDistance;
    float m_leftVoltage;
    float m_rightVoltage;
    double m_groundSteeringAngle;
    double m_pedalPosition;
    double m_x;
    double m_y;
    double m_yaw;
    Eigen::VectorXd m_measurments;
    KalmanFilter m_filter;
    std::mutex m_frontDistanceMutex;
    std::mutex m_rearDistanceMutex;
    std::mutex m_leftVoltageMutex;
    std::mutex m_rightVoltageMutex;
    std::mutex m_groundSteeringAngleMutex;
    std::mutex m_pedalPositionMutex;
    std::mutex m_measurmentsMutex;
};


#endif //AUTONOMOUS_ROBOT_CONTROLLER_H
