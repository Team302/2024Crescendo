#include <vector>
#include <cmath>
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/length.h"
#include "units/velocity.h"

class TractionControlController
{
public:
    enum State
    {
        DISABLED = 0,
        ENABLED = 1
    };

    TractionControlController(double staticCoF, double dynamicCoF, double optimalSlipRatio, double mass, double maxLinearSpeed);
    void update(double speed, double angle);
    double calculate(double velocityRequest, double inertialVelocity, double wheelSpeed);
    bool isSlipping() const;
    void toggleTractionControl();
    void enableTractionControl();
    void disableTractionControl();
    bool isEnabled() const;

private:
    static constexpr double VELOCITY_CORRECTION_SCALAR = 0.7;
    static constexpr double MIN_SLIP_RATIO = 0.01;
    static constexpr double MAX_SLIP_RATIO = 0.40;
    static constexpr int SIGMOID_K = 10;
    static constexpr double INERTIAL_VELOCITY_THRESHOLD = 0.01;
    static constexpr double MIN_SLIPPING_TIME = 1.1;
    static constexpr double m_robotLoopRate_Hz = 50.0;

    double m_optimalSlipRatio;
    double m_mass;
    double m_maxLinearSpeed;
    double m_staticCoF;
    double m_dynamicCoF;
    double m_maxPredictedSlipRatio;
    bool m_isSlipping;
    frc::Debouncer m_slippingDebouncer;
    State m_state;
};
