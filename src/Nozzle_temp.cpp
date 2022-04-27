//! Simulate nozzle temperature
//!
//! @file
//! @author Marek Bel
//! @date 26. April 2022

#include <iostream>
#include <math.h>

//! Marlin boilerplate
//! @{
#define sq(x)        ((x) * (x))
#define SQRT(x)     sqrtf(x)
#define PID_MAX 255
#define E_NAME
#define HOTEND_INDEX  0
static constexpr unsigned HOTEND_TEMPS = 1;

template <class L, class R> static inline constexpr auto _MAX(const L lhs, const R rhs) -> decltype(lhs + rhs) {
  return lhs > rhs ? lhs : rhs;
}

static float fan_speed[1]; //!< Print fan PWM 0 .. 255

struct Temp_hotend {
   float target; //!< Nozzle target temp in deg celsius
};
static Temp_hotend temp_hotend[HOTEND_TEMPS] = {};

//! @}

//! Model parameters specific to MINI hotend
//!
//! This is not average printer but worst case:
//! - deformed fan shroud
//! - heater block covered with charred filament residue
//!
//! Deformed fan shroud increases fan cooling and so FAN_COOLING_TERM
//! Blackening of aluminum block increases cooling and so LINEAR_COOLING_TERM
//! @{

static constexpr float STEADY_STATE_HOTEND_LINEAR_COOLING_TERM = 0.422f;
static constexpr float STEADY_STATE_HOTEND_QUADRATIC_COOLING_TERM = 0.00027f;
static constexpr float STEADY_STATE_HOTEND_FAN_COOLING_TERM = 4.f;
static constexpr float deg_per_second = 3.58f; //!< temperature rise at full power at zero cooling loses
static constexpr float transport_delay_seconds = 5.60f; //!< transport delay between heater and thermistor

//! Target for less than full power, so regulator can catch
//! with generated temperature curve at minimum voltage
//! (rated - 5%) = 22.8 V and maximum heater resistance
//! of 15.1 Ohm. P = 22.8/15.1*22.8 = 34.43 W
//! = 86% P(rated)
constexpr float target_heater_pwm = PID_MAX * 0.8607f;

//! @}

static constexpr float ambient_temp = 21.0f;
static constexpr float sample_frequency = 6.25f; //!< in Hz

//! @brief Get steady state output needed to compensate hotend cooling
//!
//! steady state output:
//! ((target_temp - ambient_temp) * STEADY_STATE_HOTEND_LINEAR_COOLING_TERM
//! + (target_temp - ambient_temp)^2 * STEADY_STATE_HOTEND_QUADRATIC_COOLING_TERM * (1 - print_fan))
//! * SQRT(1 + print_fan * STEADY_STATE_HOTEND_FAN_COOLING_TERM)
//! temperatures in degrees (Celsius or Kelvin)
//! @param target_temp target temperature in degrees Celsius
//! @param print_fan print fan power in range 0.0 .. 1.0
//! @return hotend PWM in range 0 .. 255

static float steady_state_hotend(float target_temp, float print_fan) {
  static_assert(PID_MAX == 255, "PID_MAX == 255 expected");
  // TODO Square root computation can be mostly avoided by if stored and updated only on print_fan change
  const float tdiff = target_temp - ambient_temp;
  const float retval = (tdiff * STEADY_STATE_HOTEND_LINEAR_COOLING_TERM
          + sq(tdiff) * STEADY_STATE_HOTEND_QUADRATIC_COOLING_TERM * (1 - print_fan))
          * SQRT(1 + print_fan * STEADY_STATE_HOTEND_FAN_COOLING_TERM);
  return _MAX(retval, 0);
}

//! @brief Get model output hotend
//!
//! @param last_target Target temperature for this cycle
//! (Can not be measured due to transport delay)
//! @param expected Expected measurable hotend temperature in this cycle
//! @param E_NAME hotend index

static float get_model_output_hotend(float &last_target, float &expected, const uint8_t E_NAME) {
  const uint8_t ee = HOTEND_INDEX;

  enum class Ramp : uint_least8_t {
    Up,
    Down,
    None,
  };

  constexpr float epsilon = 0.01f;
  constexpr int transport_delay_cycles = transport_delay_seconds * sample_frequency;
  constexpr float transport_delay_cycles_inv = 1.0f / transport_delay_cycles;
  constexpr float deg_per_cycle = deg_per_second / sample_frequency;
  constexpr float pid_max_inv = 1.0f / PID_MAX;

  float hotend_pwm = 0;
  static int delay = transport_delay_cycles;
  static Ramp state = Ramp::None;

  if (temp_hotend[ee].target > (last_target + epsilon)) {
    if (state != Ramp::Up) {
      delay = transport_delay_cycles;
      expected = last_target;
      state = Ramp::Up;
    }

    const float temp_diff = deg_per_cycle * pid_max_inv
        * (target_heater_pwm - steady_state_hotend(last_target, fan_speed[0] * pid_max_inv));
    last_target += temp_diff;
    if (delay > 1) --delay;
    expected += temp_diff / delay;
    if (last_target > temp_hotend[ee].target) last_target = temp_hotend[ee].target;
    hotend_pwm = target_heater_pwm;
  }
  else if (temp_hotend[ee].target < (last_target - epsilon)) {
    if (state != Ramp::Down) {
      delay = transport_delay_cycles;
      expected = last_target;
      state = Ramp::Down;
    }
    const float temp_diff = deg_per_cycle * pid_max_inv
        * steady_state_hotend(last_target, fan_speed[0] * pid_max_inv);
    last_target -= temp_diff;
    if (delay > 1) --delay;
    expected -= temp_diff / delay;
    if (last_target < temp_hotend[ee].target) last_target = temp_hotend[ee].target;
    hotend_pwm = 0;
  }
  else {
    state = Ramp::None;
    last_target = temp_hotend[ee].target;
    const float remaining = last_target - expected;
    if (expected > (last_target + epsilon)) {
      float diff = remaining * transport_delay_cycles_inv;
      if (abs(diff) < epsilon) diff = -epsilon;
      expected += diff;
    }
    else if (expected < (last_target - epsilon)) {
      float diff = remaining * transport_delay_cycles_inv;
      if (abs(diff) < epsilon) diff = epsilon;
      expected += diff;
    }
    else expected = last_target;
    hotend_pwm = steady_state_hotend(last_target, fan_speed[0] * pid_max_inv);
  }
  return hotend_pwm;
}


int main() {
    using namespace std;
    float expected_temp = ambient_temp;
    float last_target = ambient_temp;

    // heat up at 86% P(rated) with disabled fan
    temp_hotend[0].target = 215.f;
    fan_speed[0] = 0.f;
    for (int i = 0; i < sample_frequency * 110; ++i) {
        (void) get_model_output_hotend(last_target, expected_temp, 0);
        cout << last_target << endl;
    }

    // cool down with full print fan
    temp_hotend[0].target = 0.f;
    fan_speed[0] = 255.f;
    for (int i = 0; i < sample_frequency * 110; ++i) {
        (void) get_model_output_hotend(last_target, expected_temp, 0);
        cout << last_target << endl;
    }

    // heat up at 86% P(rated) with disabled fan
    temp_hotend[0].target = 215.f;
    fan_speed[0] = 0.f;
    for (int i = 0; i < sample_frequency * 110; ++i) {
        (void) get_model_output_hotend(last_target, expected_temp, 0);
        cout << last_target << endl;
    }

	return 0;
}
