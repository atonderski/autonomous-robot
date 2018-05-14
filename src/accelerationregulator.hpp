#ifndef ACCELERATION_REGULATOR
#define ACCELERATION_REGULATOR

class AccelerationRegulator {
    public:
        AccelerationRegulator(ConfigurationValues m_conf, double const in_DT) noexcept
        : m_currentPedalLogic{0}
        , m_counter{0}
        , DT{in_DT}
        , TIME_SPLIT(m_conf.confMap.count("TIME_SPLIT") ? m_conf.confMap["TIME_SPLIT"] : 0.6)
        , FORWARD_PEDAL_HIGH(m_conf.confMap.count("FORWARD_PEDAL_HIGH") ? m_conf.confMap["FORWARD_PEDAL_HIGH"] : 0.13)
        , FORWARD_PEDAL_LOW(m_conf.confMap.count("FORWARD_PEDAL_LOW") ? m_conf.confMap["FORWARD_PEDAL_LOW"] : 0.105)
        , BACKWARD_PEDAL_HIGH(m_conf.confMap.count("BACKWARD_PEDAL_HIGH") ? m_conf.confMap["BACKWARD_PEDAL_HIGH"] : -0.54)
        , BACKWARD_PEDAL_LOW(m_conf.confMap.count("BACKWARD_PEDAL_LOW") ? m_conf.confMap["BACKWARD_PEDAL_LOW"] : -0.45)
        {
            std::cout   << TIME_SPLIT << " "
                        << FORWARD_PEDAL_HIGH << " "
                        << FORWARD_PEDAL_LOW << " "
                        << BACKWARD_PEDAL_HIGH << " "
                        << BACKWARD_PEDAL_LOW << std::endl;
        }
        ~AccelerationRegulator() = default;

        void setPedalLogic(int newPedalLogic) noexcept {
            if (newPedalLogic != m_currentPedalLogic) {
                m_currentPedalLogic = newPedalLogic;
                m_counter = 0;
            }
        }

        double getPedalPosition() noexcept {
            if (m_currentPedalLogic == 1) {
                if (m_counter < TIME_SPLIT) {
                    m_counter += DT;
                    return FORWARD_PEDAL_HIGH;
                } else {
                    return FORWARD_PEDAL_LOW;
                }
            } else if (m_currentPedalLogic == -1) {
                if (m_counter < TIME_SPLIT) {
                    m_counter += DT;
                    return BACKWARD_PEDAL_HIGH;
                } else {
                    return BACKWARD_PEDAL_LOW;
                }
            } else {
                return 0.0;
            }
        }

    private:
        int m_currentPedalLogic;
        double m_counter;
        double const DT;
        double const TIME_SPLIT;
        double const FORWARD_PEDAL_HIGH;
        double const FORWARD_PEDAL_LOW;
        double const BACKWARD_PEDAL_HIGH;
        double const BACKWARD_PEDAL_LOW;
};

#endif