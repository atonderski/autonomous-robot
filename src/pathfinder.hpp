#ifndef AUTONOMOUS_ROBOT_PATHFINDER
#define AUTONOMOUS_ROBOT_PATHFINDER

#include <random>
#include "controller.hpp"
#include "configurationvalues.hpp"
#include "path-maker.hpp"
#include "accelerationregulator.hpp"
#include "behaviours/behaviour-reflex.hpp"
#include "behaviours/behaviour-avoid.hpp"
#include "behaviours/behaviour-followpath.hpp"



class Pathfinder : public Controller {
public:
    Pathfinder(double const DT) noexcept
        : Controller(DT)
        , m_stepFront{}
        , m_stepRear{}
        , m_stepLeft{}
        , m_stepRight{}
        , m_stepX{}
        , m_stepY{}
        , m_stepYaw{}
        , m_filter{}
        , m_conf{"/opt/pathfinder.conf"}
        , m_pathMaker{m_conf}
        , m_accelerationRegulator{m_conf, DT}
        , m_behaviourReflex{m_conf, DT, &m_stepFront, &m_stepRear, &m_stepLeft, &m_stepRight}
        , m_behaviourAvoid{m_conf, DT, &m_stepFront, &m_stepRear, &m_stepLeft, &m_stepRight}
        , m_behaviourFollowPath{m_pathMaker.path, &m_stepX, &m_stepY, &m_stepYaw, m_conf, DT, &m_stepFront, &m_stepRear, &m_stepLeft, &m_stepRight}
        , m_generator{}
        , m_distribution{0, 1.0}
    {
        int n = 6; // Number of states
        int m = 6; // Number of measurements

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
        R = Eigen::MatrixXd::Identity(m,m) * 1;

        P = Eigen::MatrixXd::Identity(n,n) * 1;

        std::cout << "A: \n" << A << std::endl;
        std::cout << "C: \n" << C << std::endl;
        std::cout << "Q: \n" << Q << std::endl;
        std::cout << "R: \n" << R << std::endl;
        std::cout << "P: \n" << P << std::endl;

        // Construct and init the filter
        m_filter = KalmanFilter(DT, A, C, Q, R, P);
        m_filter.init();
    }
    ~Pathfinder() override = default;

public:
    bool step() noexcept override;

protected:
    void runFilter() noexcept;

private:
    double m_stepFront;
    double m_stepRear;
    double m_stepLeft;
    double m_stepRight;
    double m_stepX;
    double m_stepY;
    double m_stepYaw;
    KalmanFilter m_filter;

    ConfigurationValues m_conf;
    PathMaker m_pathMaker;
    AccelerationRegulator m_accelerationRegulator;
    BehaviourReflex m_behaviourReflex;
    BehaviourAvoid m_behaviourAvoid;
    BehaviourFollowPath m_behaviourFollowPath;

    std::default_random_engine m_generator;
    std::normal_distribution<double> m_distribution;

};

#endif
