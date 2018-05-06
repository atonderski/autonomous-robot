//
// Created by Adam Tonderski on 2018-04-16.
//

#ifndef AUTONOMOUS_ROBOT_CONTROLLER_H
#define AUTONOMOUS_ROBOT_CONTROLLER_H

#include <mutex>
#include <limits>

#include "opendlv-standard-message-set.hpp"


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
        , m_frontDistanceMutex{}
        , m_rearDistanceMutex{}
        , m_leftVoltageMutex{}
        , m_rightVoltageMutex{}
        , m_groundSteeringAngleMutex{}
        , m_pedalPositionMutex{}
        {
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
                distance = (6.65565/voltage - 0.5616)/100;
            }
            return distance;
        }
        double scalePedalPosition(double logicPedalPosition) const noexcept {
            if (logicPedalPosition > 0) {
                return logicPedalPosition * 0.08 + 0.12;
            } else if (logicPedalPosition < 0) {
                return logicPedalPosition * 0.34 - 0.46;
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
        std::mutex m_frontDistanceMutex;
        std::mutex m_rearDistanceMutex;
        std::mutex m_leftVoltageMutex;
        std::mutex m_rightVoltageMutex;
        std::mutex m_groundSteeringAngleMutex;
        std::mutex m_pedalPositionMutex;
};


#endif //AUTONOMOUS_ROBOT_CONTROLLER_H
