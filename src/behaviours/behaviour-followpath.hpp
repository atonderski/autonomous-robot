#ifndef BEHAVIOUR_FOLLOWPATH
#define BEHAVIOUR_FOLLOWPATH

#include "augmentedfinitestatemachine.hpp"
#include <cmath>
#include <iostream>
#include <vector>
#include <list>

class BehaviourFollowPath : public AugmentedFiniteStateMachine {
    public:
        BehaviourFollowPath(std::list<std::pair<double,double>> pathList, double *m_stepX, double *m_stepY, double *m_stepYaw, ConfigurationValues m_conf, double const DT, double *stepFront, double *stepRear, double *stepLeft, double *stepRight) noexcept
            : AugmentedFiniteStateMachine(m_conf, DT, stepFront, stepRear, stepLeft, stepRight)
            , m_pX{m_stepX}
            , m_pY{m_stepY}
            , m_pYaw{m_stepYaw}
            , m_path{ std::begin(pathList), std::end(pathList) }
            , m_previewPointOffset{0.55}
            , m_finalPathIndex{}
            , m_stopRadius{0.3}
            , m_counter{10}
        {
            m_finalPathIndex = m_path.size() - 1;
        }
        ~BehaviourFollowPath() override = default;

        void initialState() noexcept override;
        void FollowPathState() noexcept;

    private:
        double getSteeringSignal() noexcept;
        std::pair<double,double> getPreviewPoint() noexcept;
        double distanceToPathpoint(int) noexcept;
        int signum(double) noexcept;

        BehaviourFollowPath(BehaviourFollowPath const &) = delete;
        BehaviourFollowPath(BehaviourFollowPath &&) = delete;
        BehaviourFollowPath &operator=(BehaviourFollowPath const &) = delete;
        BehaviourFollowPath &operator=(BehaviourFollowPath &&) = delete;

        double const *m_pX;
        double const *m_pY;
        double const *m_pYaw;
        std::vector<std::pair<double,double>> m_path;
        double m_previewPointOffset;
        int m_finalPathIndex;
        double m_stopRadius;
        double m_counter;

};

#endif