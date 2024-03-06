#ifndef H_fbcontrol_custom_h
#define H_fbcontrol_custom_h
#include <vector>
#include <optional>
#include <functional>
#include <memory>

using FirstGuessFunction = std::function<double(double)>;

enum class MotorSide {left, right};

std::vector<double>& get_rpm();

std::vector<double>& get_pwm(MotorSide side);

FirstGuessFunction make_first_guess_function(MotorSide side);



struct ValueWithDirection
{
    double m_raw_value;
    ValueWithDirection();
    ValueWithDirection(double v);
    [[nodiscard]] double abs_value() const;
    void update_abs_value(double abs_value);
    [[nodiscard]] double signed_value() const;
    double sign() const;
};

struct MotorProfile
{
    using UPtr = std::unique_ptr<MotorProfile>;
    MotorProfile(MotorSide side, std::vector<double>& pwm, std::vector<double>& rpm);
    double guess_pwm(double rpm);
    double slope_at_pwm(double pwm);
    double slope_at_rpm(double rpm);
    std::optional<int> find_pwm_index(double pwm);
    std::optional<int> find_rpm_index(double pwm);


    MotorSide            m_side;
    std::vector<double>& m_pwm;
    std::vector<double>& m_rpm;
    float m_min_pwm;
    float m_max_pwm;
    float m_max_target;
    float m_min_target;
    float m_error_tolerance;
    float m_fixed_step_size;
    float m_proportional_error_tolerance_factor;
    float m_proportional_up_multiplier;
    float m_proportional_down_multiplier;

};

struct CustomController
{
    using FirstGuessFunction = std::function<double(double)>;
    enum class Algorithm {linear, proportional, fixed_step};

    CustomController(Algorithm algorithm, MotorSide side);

    auto set_target(ValueWithDirection new_target) -> ValueWithDirection;
    auto get_target() -> ValueWithDirection;
    auto update(ValueWithDirection latest_y) -> ValueWithDirection;

    double              m_error;
    MotorSide           m_side;
    MotorProfile        m_profile;
    ValueWithDirection  m_target;
    Algorithm           m_algorithm;
    int                 m_count;
    double              m_previous_error;
    double              m_xn_1;
    double              m_yn_1;
    double              m_xn;
    double              m_yn;
    double              m_en_1;
    double              m_en;
    int                 m_direction;
    FirstGuessFunction*  m_fg_func;
};


#endif