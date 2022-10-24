#ifndef JOY_UTIL_H
#define JOY_UTIL_H

#include <Arduino.h>
#include <BleGamepad.h>
#include <Preferences.h>

class JoyUtil {
  private:
    const float SLOPE = 0.414214;

    uint16_t ble_axis_min = 0;
    uint16_t ble_axis_max = 4095;

    BleGamepadConfiguration ble_gamepad_cfg;
    BleGamepad ble_gamepad;

    std::string name;
    std::string manufacturer;

    int16_t map_axis_range_ble(float state);

    byte button_states[11] = { HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH };
    byte dpad_state = DPAD_CENTERED;

    const float axis_states_raised[6] = { 0.0, 0.0, 0.0, 0.0, -1.0, -1.0 };

    float axis_states[6] = { 0.0, 0.0, 0.0, 0.0, -1.0, -1.0 };
    float axis_states_raw[6] = { 0.0, 0.0, 0.0, 0.0, -1.0, -1.0 };

    float axis_min[6] = { -1.0, -1.0, -1.0, -1.0, -1.0, -1.0 };
    float axis_mid[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    float axis_max[6] = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };

  public:
    Preferences preferences;

    static const byte BUTTON_COUNT = 11;
    static const byte AXIS_COUNT = 6;

    static const byte BUTTON_A = 0;
    static const byte BUTTON_B = 1;
    static const byte BUTTON_X = 2;
    static const byte BUTTON_Y = 3;
    static const byte BUTTON_LB = 4;
    static const byte BUTTON_RB = 5;

    static const byte BUTTON_LSB = 6;
    static const byte BUTTON_RSB = 7;

    static const byte BUTTON_START = 8;
    static const byte BUTTON_SELECT = 9;
    static const byte BUTTON_HOME = 10;

    static const byte AXIS_LX = 0;
    static const byte AXIS_LY = 1;
    static const byte AXIS_RX = 2;
    static const byte AXIS_RY = 3;
    static const byte AXIS_LT = 4;
    static const byte AXIS_RT = 5;

    float deadzone = 0.2;

    const char * button_names[11] = { "A", "B", "X", "Y", "LB", "RB", "LSB", "RSB", "START", "SELECT", "HOME" };
    const char * axis_names[6] = { "LX", "LY", "RX", "RY", "LT", "RT" };

    const char * axis_min_names[6] = { "lx-axis-min", "ly-axis-min", "rx-axis-min", "ry-axis-min", "lt-axis-min", "lt-axis-min" };
    const char * axis_mid_names[6] = { "lx-axis-mid", "ly-axis-mid", "rx-axis-mid", "ry-axis-mid", "lt-axis-mid", "lt-axis-mid" };
    const char * axis_max_names[6] = { "lx-axis-max", "ly-axis-max", "rx-axis-max", "ry-axis-max", "rt-axis-max", "rt-axis-max" };

    JoyUtil (std::string name, std::string manufacturer, float deadzone);

    void connect ();

    void prefs_init ();
    void prefs_read ();
    void prefs_write ();

    static float map_range (float value, float old_min, float old_max, float new_min, float new_max);

    float dz_scaled_radial (float input);

    void configure_gamepad ();
    void set_axis_state (byte axis, float state);
    float clean_axis_value (byte axis, float state);

    void set_button_state (byte btn, byte state);
    void set_dpad_state(byte dpad_up, byte dpad_right, byte dpad_down, byte dpad_left);
    void set_dpad_analog_state(byte axis_x, byte axis_y, float value_x, float value_y);

    byte get_button_state(byte btn);
    float get_axis_state(byte axis);
    float get_axis_state_raw(byte axis);
    byte get_dpad_state();

    void set_axis_min (byte axis, float min);
    void set_axis_mid (byte axis, float mid);
    void set_axis_max (byte axis, float max);

    float get_axis_min (byte axis);
    float get_axis_mid (byte axis);
    float get_axis_max (byte axis);

    bool is_any_pressed ();

    void raise_inputs ();
    void report ();
    bool is_connected ();
};

#endif
