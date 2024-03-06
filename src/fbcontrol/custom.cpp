#include "custom.h"
#include <stdexcept>
#include <format>

static std::vector<double> static_rpm_vec   =  {2000.0, 2500.0, 3000.0, 3500.0, 4000.0, 4500.0, 5000.0, 5500.0, 6000.0, 6500.0, 7000.0, 7500.0, 8000.0};
static std::vector<double> static_pwm_left  =  {41.4,   45.95,  49.9,   53.92,  56.6,   71.6,   82.0,   85.5,   88.5,   91.2,   95.0,   99.91,   99.999};
static std::vector<double> static_pwm_right =  {38.9,   44.1,   48.25,  51.30,  54.5,   60.2,   79.0,   84.0,   87.5,   90.2,   93.5,   98.00,   99.999};

std::vector<double>& get_rpm()
{
    return static_rpm_vec;
}

std::vector<double>& get_pwm(MotorSide side)
{
    std::vector<double>& pwm = (side == MotorSide::left) ? static_pwm_left : static_pwm_right;
    return pwm;
}

FirstGuessFunction make_first_guess_function(MotorSide side)
{
    std::vector<double>& pwm = (side == MotorSide::left) ? static_pwm_left : static_pwm_right;
    std::vector<double>& rpm_vec = static_rpm_vec;
    return [&](double xvalue) {
        auto neg_flag = (xvalue < 0.0);
        auto abs_xvalue = neg_flag ? -xvalue: xvalue;
        if(std::abs(rpm_vec.back() - abs_xvalue) < 0.1) {
            return pwm.back();
        }
        if(abs_xvalue < rpm_vec[0] or rpm_vec.back() < abs_xvalue) {
            throw std::runtime_error(std::format("first_guess_function abs_xvalue is out of range {}", abs_xvalue));
        }
        std::optional<int> lower_index = std::nullopt;
        for(int i = 0; i < rpm_vec.size() - 1; i++) {
            if(rpm_vec[i] <= abs_xvalue && abs_xvalue < rpm_vec[i+1]) {
                lower_index = i;
            }
        }
        if(!lower_index) {
            throw std::runtime_error(std::format("first_guess_function failed to find the interval for abs_xvalue {}", abs_xvalue));
        } else {
            auto li = lower_index.value();
            auto x0 = rpm_vec[li];
            auto x1 = rpm_vec[li + 1];
            auto y0 = pwm[li];
            auto y1 = pwm[li + 1];
            auto y = ((y1 - y0) / (x1 - x0)) * (abs_xvalue - x0) + y0;
            return y;
        }
    };
}
/**
 * MotorProfile =======================================================================================================
 */
MotorProfile::MotorProfile(MotorSide side, std::vector<double>& pwm, std::vector<double>& rpm)
:m_pwm(pwm), m_rpm(rpm)
{
    m_side = side;
    m_min_pwm = 30.0;
    m_max_pwm = 99.9;
    m_max_target = 7500.0;
    m_min_target = 2000.0;
    m_error_tolerance = 50.0; //+/- 25 rpm either side
    m_fixed_step_size = 0.5;  // remember this is a pwm value and they are in range 0 .. 100.0
    m_proportional_error_tolerance_factor = 0.02; // 2%
    m_proportional_up_multiplier = 1.05;          // this is a steps of say 50% pwm to 52.5% pwm
    m_proportional_down_multiplier = 0.95;        // this is a step of say  50% pwn to 47.5
}
double MotorProfile::guess_pwm(double rpm)
{
    double return_value;
    if(rpm < m_min_target) {
        return 0.0;
    }
    double xvalue =  (rpm >= m_max_target) ? m_max_target: rpm;
    std::vector<double>& pwm = m_pwm;
    std::vector<double>& rpm_vec_local = m_rpm;
    auto neg_flag = (xvalue < 0.0);
    auto abs_xvalue = neg_flag ? -xvalue: xvalue;
    if(std::abs(rpm_vec_local.back() - abs_xvalue) < 0.1) {
        return_value = pwm.back();
    }
    if(abs_xvalue < rpm_vec_local[0] or rpm_vec_local.back() < abs_xvalue) {
        throw std::runtime_error(std::format("guess_pwm function abs_xvalue is out of range {}", abs_xvalue));
    }
    std::optional<int> lower_index = std::nullopt;
    for(int i = 0; i < rpm_vec_local.size() - 1; i++) {
        if(rpm_vec_local[i] <= abs_xvalue && abs_xvalue < rpm_vec_local[i+1]) {
            lower_index = i;
        }
    }
    if(!lower_index) {
        throw std::runtime_error(std::format("guess_pwm failed to find the interval for abs_xvalue {}", abs_xvalue));
    } else {
        auto li = lower_index.value();
        auto x0 = rpm_vec_local[li];
        auto x1 = rpm_vec_local[li + 1];
        auto y0 = pwm[li];
        auto y1 = pwm[li + 1];
        auto y = ((y1 - y0) / (x1 - x0)) * (abs_xvalue - x0) + y0;
        return_value = y;
    }
    return return_value;
}
std::optional<int> MotorProfile::find_pwm_index(double pwm)
{
    if(pwm < 0.0) {
        throw std::runtime_error("MotorProfile::find_pwm_index  pwm value is negative");
    }
    std::optional<int> lower_index = std::nullopt;
    for(int i = 0; i < m_pwm.size() - 1; i++) {
        if(m_pwm[i] <= pwm && pwm < m_pwm[i+1]) {
            lower_index = i;
        }
    }
    return lower_index;
}
std::optional<int> MotorProfile::find_rpm_index(double rpm)
{
    if(rpm < 0.0) {
        throw std::runtime_error("MotorProfile::find_rpm_index  rpm value is negative");
    }
    std::optional<int> lower_index = std::nullopt;
    for(int i = 0; i < m_rpm.size() - 1; i++) {
        if(m_rpm[i] <= rpm && rpm < m_rpm[i+1]) {
            lower_index = i;
        }
    }
    // assert 0 <= lower_index < (not equal) m_rpm.size()-1
    return lower_index;
}


double MotorProfile::slope_at_pwm(double pwm)
{
    auto ix0 = find_pwm_index(pwm);
    if(!ix0) {
        throw std::runtime_error("MotorProfile.slope_at_pwm  failed to find index");
    }
    auto x0 = m_pwm.at(ix0.value());
    auto x1 = m_pwm[ix0.value()+1];
    auto y0 = m_rpm[ix0.value()];
    auto y1 = m_rpm[ix0.value()+1];
    auto slope = (y1 - y0) / (x1 - x0);
    return slope;
}
double MotorProfile::slope_at_rpm(double rpm)
{
    auto ix0 = find_rpm_index(rpm);
    if(!ix0) {
        throw std::runtime_error("MotorProfile.slope_at_rpm  failed to find index");
    }
    auto x0 = m_pwm[ix0.value()];
    auto x1 = m_pwm[ix0.value()+1];
    auto y0 = m_rpm[ix0.value()];
    auto y1 = m_rpm[ix0.value()+1];
    auto slope = (y1 - y0) / (x1 - x0);
    return slope;
}


/**
 * ValueWithDirection==================================================================================================
 */
ValueWithDirection::ValueWithDirection()
{
    m_raw_value = 0.0;
}
ValueWithDirection::ValueWithDirection(double v)
{
    m_raw_value = v;
}
[[nodiscard]] double ValueWithDirection::abs_value() const
{
    return m_raw_value < 0 ? -m_raw_value : m_raw_value;
}
void ValueWithDirection::update_abs_value(double abs_value)
{
    if(m_raw_value < 0)
        m_raw_value = -abs_value;
    else
        m_raw_value = abs_value;
}
[[nodiscard]] double ValueWithDirection::signed_value() const
{
    return m_raw_value;
}
double ValueWithDirection::sign() const
{
    return (m_raw_value) ? 1.0: -1.0;
}


/**
 * CustomController  ==================================================================================================
 */

CustomController::CustomController(Algorithm algorithm, MotorSide side):
        m_target(ValueWithDirection{}), m_algorithm(algorithm), m_profile(MotorProfile(side, get_pwm(side), get_rpm()))
{
    m_error = 0.0;
    m_side = side;
    m_previous_error = 0.0;
    m_yn_1 = 0.0;
    m_yn = 0.0;
    m_xn_1 = 0.0;
    m_xn = 0.0;
    m_en = 0.0;
    m_en_1 = 0.0;
    m_direction = 1;
    m_count;
}
auto CustomController::set_target(ValueWithDirection new_target) -> ValueWithDirection
{
    m_count = 0;
    m_target = new_target;
    auto new_x = m_profile.guess_pwm(m_target.abs_value());
    m_xn = new_x;
    m_yn = 0.0;
    auto r = new_x < 100.0 ? new_x: 100.0;
    m_count = 1;
    return ValueWithDirection(new_x);

}
auto CustomController::get_target() -> ValueWithDirection
{
    return m_target;
}
auto CustomController::update(ValueWithDirection latest_y) -> ValueWithDirection
{
    double new_x = 0.0;
    if(m_target.abs_value() == 0.0) {
        return ValueWithDirection{};
    }
    if(m_count == 0) {
        m_error = m_target.abs_value() - latest_y.abs_value();
        new_x = m_profile.guess_pwm(m_target.abs_value());
        m_xn = new_x;
        m_yn = 0.0;
        m_count++;
    } else if(1 <= m_count && m_count <= 3) {
        new_x = m_xn;
        m_count++;
    } else {
        m_yn = latest_y.abs_value();
        m_error = m_target.abs_value() - latest_y.abs_value();
        double error_percent = m_error / m_target.abs_value();
        switch(m_algorithm) {
            case Algorithm::linear:{
                    if(std::abs(m_error) >= m_profile.m_error_tolerance) {
                        double slope1 = (m_yn - m_yn_1) / (m_xn - m_xn_1);
                        double slope2 = m_profile.slope_at_rpm(m_target.abs_value());
                        double slope3 = m_profile.slope_at_pwm(m_xn);
                        double slope = slope2/* one of these*/;
                        new_x =  (m_target.abs_value() - m_yn) / slope + m_xn;
                        m_xn_1 = m_xn;
                        m_yn_1 = m_yn;
                        m_xn = new_x;
                    } else {
                        new_x = m_xn;
                    }
                }
                break;
            case Algorithm::fixed_step:{
                if(std::abs(m_error) < 0.5 * std::abs(m_profile.m_error_tolerance)) {
                    printf("HALF m_error %f tolerance: %f\n", m_error, m_profile.m_error_tolerance);
                } else if(std::abs(m_error) < std::abs(m_profile.m_error_tolerance)) {
                    printf("FULL m_error %f tolerance: %f\n", m_error, m_profile.m_error_tolerance);
                } else if(std::abs(m_error) < 2.0 * std::abs(m_profile.m_error_tolerance)) {
                    printf("TWICE m_error %f tolerance: %f\n", m_error, m_profile.m_error_tolerance);
                }

                if(m_error > 2 * m_profile.m_error_tolerance) {
                    new_x = m_xn + 2 * m_profile.m_fixed_step_size;
                } else if(m_error < -2.0 * m_profile.m_error_tolerance) {
                    new_x = m_xn - 2 * m_profile.m_fixed_step_size;

                } else if(m_error > m_profile.m_error_tolerance) {
                    new_x = m_xn + m_profile.m_fixed_step_size;
                } else if(m_error < -1.0 * m_profile.m_error_tolerance) {
                    new_x = m_xn - m_profile.m_fixed_step_size;

                } else if(m_error > 0.5 * m_profile.m_error_tolerance) {
                    new_x = m_xn + 0.5 * m_profile.m_fixed_step_size;
                } else if(m_error < -0.5 * m_profile.m_error_tolerance) {
                    new_x = m_xn - 0.5 * m_profile.m_fixed_step_size;

                } else {
                    new_x = m_xn;
                }
                m_xn = new_x;
            }
            break;
            case Algorithm::proportional:{
                if(std::abs(m_error) / m_target.abs_value() > m_profile.m_proportional_error_tolerance_factor) {
                    new_x = m_xn * m_profile.m_proportional_up_multiplier;
                } else if(m_error / m_target.abs_value() < -0.1) {
                    new_x = m_xn * m_profile.m_proportional_down_multiplier;
                } else {
                    new_x = m_xn;
                }

            }
            break;
        }
        m_previous_error = m_error;
        new_x = (new_x > m_profile.m_min_pwm) ? new_x : m_profile.m_min_pwm;
    }
    double r = (new_x <= m_profile.m_max_pwm) ? new_x : m_profile.m_max_pwm;
    ValueWithDirection  v(m_target.sign() * r);
    return v;
}
