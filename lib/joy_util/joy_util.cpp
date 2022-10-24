#include <Arduino.h>

#include "joy_util.h"

JoyUtil::JoyUtil(std::string name, std::string manufacturer, float deadzone)
: ble_gamepad(name, manufacturer, 100) {

  this->name = name;
  this->manufacturer = name;
  this->deadzone = deadzone;
};

void JoyUtil::connect () {
  configure_gamepad();
  ble_gamepad.begin(&ble_gamepad_cfg);
}

void JoyUtil::prefs_init () {
  preferences.begin("joy-util", false);
}

void JoyUtil::prefs_read () {
  for (byte axis = 0; axis < AXIS_COUNT; axis++) {
    axis_min[axis] = preferences.getFloat(axis_min_names[axis], -1.0);
    axis_mid[axis] = preferences.getFloat(axis_mid_names[axis], 0.0);
    axis_max[axis] = preferences.getFloat(axis_max_names[axis], 1.0);
  }
}

void JoyUtil::prefs_write () {
  for (byte axis = 0; axis < JoyUtil::AXIS_COUNT; axis++) {
    preferences.putFloat(axis_min_names[axis], axis_min[axis]);
    preferences.putFloat(axis_mid_names[axis], axis_mid[axis]);
    preferences.putFloat(axis_max_names[axis], axis_max[axis]);
  }
}

// https://github.com/Minimuino/thumbstick-deadzones

float JoyUtil::map_range (float value, float old_min, float old_max, float new_min, float new_max) {
  if (value < old_min) value = old_min;
  if (value > old_max) value = old_max;

  return (new_min + (new_max - new_min) * (value - old_min) / (old_max - old_min));
}

// https://github.com/Minimuino/thumbstick-deadzones

float JoyUtil::dz_scaled_radial (float input) {
  const float input_abs = abs(input);
  if (input_abs < deadzone) {
    return 0.0;
  }

  const float sign = input / input_abs;

  return sign * map_range(input_abs, deadzone, 1.0, 0.0, 1.0);
}

void JoyUtil::configure_gamepad () {
  ble_gamepad_cfg.setAutoReport(false);
  ble_gamepad_cfg.setControllerType(CONTROLLER_TYPE_GAMEPAD);

  ble_gamepad_cfg.setHatSwitchCount(1);

  ble_gamepad_cfg.setIncludeStart(true);
  ble_gamepad_cfg.setIncludeSelect(true);
  ble_gamepad_cfg.setIncludeHome(true);

  ble_gamepad_cfg.setButtonCount(BUTTON_COUNT);

  ble_gamepad_cfg.setWhichAxes(false, false, false, false, false, false, false, false);

  ble_gamepad_cfg.setIncludeXAxis(true);
  ble_gamepad_cfg.setIncludeYAxis(true);
  ble_gamepad_cfg.setIncludeZAxis(true);
  ble_gamepad_cfg.setIncludeRzAxis(true);

  ble_gamepad_cfg.setIncludeRxAxis(true);
  ble_gamepad_cfg.setIncludeRyAxis(true);

  ble_gamepad_cfg.setAxesMin(ble_axis_min); // 0 --> int16_t - 16 bit signed integer - Can be in decimal or hexadecimal
  ble_gamepad_cfg.setAxesMax(ble_axis_max); // 32767 --> int16_t - 16 bit signed integer - Can be in decimal or hexadecimal
}

int16_t JoyUtil::map_axis_range_ble (float state) {
  return map_range(state, -1.0, 1.0, ble_axis_min, ble_axis_max);
}

float JoyUtil::clean_axis_value (byte axis, float state) {
  axis_states_raw[axis] = state;

  float state_calibrated = 0.0;

  if (state > axis_mid[axis]) {
    state_calibrated = map_range(state, axis_mid[axis], axis_max[axis], 0.0, 1.0);
  } else if (state < axis_mid[axis]) {
    state_calibrated = map_range(state, axis_min[axis], axis_mid[axis], -1.0, 0.0);
  }

  return dz_scaled_radial(state_calibrated);
}

void JoyUtil::set_axis_state (byte axis, float state) {
  axis_states[axis] = clean_axis_value(axis, state);
}

float JoyUtil::get_axis_state (byte axis) {
  return axis_states[axis];
}

float JoyUtil::get_axis_min (byte axis) {
  return axis_min[axis];
}

float JoyUtil::get_axis_mid (byte axis) {
  return axis_mid[axis];
}

float JoyUtil::get_axis_max (byte axis) {
  return axis_max[axis];
}

float JoyUtil::get_axis_state_raw (byte axis) {
  return axis_states_raw[axis];
}

byte JoyUtil::get_dpad_state () {
  return dpad_state;
}

void JoyUtil::set_axis_min (byte axis, float min) {
  axis_min[axis] = min;
}

void JoyUtil::set_axis_mid (byte axis, float mid) {
  axis_mid[axis] = mid;
}

void JoyUtil::set_axis_max (byte axis, float max) {
  axis_max[axis] = max;
}

void JoyUtil::raise_inputs () {
  for (byte btn = 0; btn < BUTTON_COUNT; btn++) {
    button_states[btn] = HIGH;
  }
  for (byte axis = 0; axis < AXIS_COUNT; axis++) {
    axis_states[axis] = 0.0;
  }
  dpad_state = DPAD_CENTERED;
}

void JoyUtil::set_button_state (byte btn, byte state) {
  button_states[btn] = state;
}

byte JoyUtil::get_button_state (byte btn) {
  return button_states[btn];
}

void JoyUtil::set_dpad_state (byte dpad_up, byte dpad_right, byte dpad_down, byte dpad_left) {
  dpad_state = DPAD_CENTERED;

  if (dpad_up == LOW) {
    if (dpad_right == LOW) {
      dpad_state = DPAD_UP_RIGHT;
    } else if (dpad_left == LOW) {
      dpad_state = DPAD_UP_LEFT;
    } else {
      dpad_state = DPAD_UP;
    }
  } else if (dpad_down == LOW) {
    if (dpad_right == LOW) {
      dpad_state = DPAD_DOWN_RIGHT;
    } else if (dpad_left == LOW) {
      dpad_state = DPAD_DOWN_LEFT;
    } else {
      dpad_state = DPAD_DOWN;
    }
  } else if (dpad_left == LOW) {
    dpad_state = DPAD_LEFT;
  } else if (dpad_right == LOW) {
    dpad_state = DPAD_RIGHT;
  }
}

// https://gamingprojects.wordpress.com/2017/08/04/converting-analog-joystick-to-digital-joystick-signals/

void JoyUtil::set_dpad_analog_state (byte axis_x, byte axis_y, float value_x, float value_y) {
  byte dpad_up = HIGH;
  byte dpad_down = HIGH;
  byte dpad_left = HIGH;
  byte dpad_right = HIGH;

  // squared deadzone?

  const float x = clean_axis_value(axis_x, value_x);
  const float y = clean_axis_value(axis_y, value_y);

  const float slope_y = SLOPE * y;
  const float slope_x = SLOPE * x;

  if (x > 0.0) {
    if (x > slope_y) dpad_right = LOW;
  } else if (x < 0.0) {
    if (x < slope_y) dpad_left = LOW;
  }

  if (y > 0.0) {
    if (y > slope_x) dpad_down = LOW;
  } else if (y < 0.0) {
    if (y < slope_x) dpad_up = LOW;
  }

  set_dpad_state(dpad_up, dpad_right, dpad_down, dpad_left);
}

bool JoyUtil::is_connected () {
  return ble_gamepad.isConnected();
}

void JoyUtil::report () {
  if (!is_connected()) return;

  for (byte btn = 0; btn < BUTTON_COUNT; btn++) {
    byte ble_button;
    bool is_special = false;

    switch (btn) {
      case BUTTON_START:
        ble_button = START_BUTTON;
        is_special = true;
        break;
      case BUTTON_SELECT:
        ble_button = SELECT_BUTTON;
        is_special = true;
        break;
      case BUTTON_HOME:
        ble_button = HOME_BUTTON;
        is_special = true;
        break;
      case BUTTON_A:
        ble_button = BUTTON_1;
        break;
      case BUTTON_B:
        ble_button = BUTTON_2;
        break;
      case BUTTON_X:
        ble_button = BUTTON_3;
        break;
      case BUTTON_Y:
        ble_button = BUTTON_4;
        break;

      case BUTTON_LB:
        ble_button = BUTTON_5;
        break;
      case BUTTON_RB:
        ble_button = BUTTON_6;
        break;
      case BUTTON_LSB:
        ble_button = BUTTON_7;
        break;
      case BUTTON_RSB:
        ble_button = BUTTON_8;
        break;
    }

    if (button_states[btn] == LOW) {
      if (is_special) ble_gamepad.pressSpecialButton(ble_button);
      else ble_gamepad.press(ble_button);
    } else {
      if (is_special) ble_gamepad.releaseSpecialButton(ble_button);
      else ble_gamepad.release(ble_button);
    }
  }

  ble_gamepad.setHat(dpad_state);

  ble_gamepad.setLeftThumb(map_axis_range_ble(axis_states[AXIS_LX]), map_axis_range_ble(axis_states[AXIS_LY]));
  ble_gamepad.setRightThumb(map_axis_range_ble(axis_states[AXIS_RX]), map_axis_range_ble(axis_states[AXIS_RY]));
  ble_gamepad.setLeftTrigger(map_axis_range_ble(axis_states[AXIS_LT]));
  ble_gamepad.setRightTrigger (map_axis_range_ble(axis_states[AXIS_RT]));

  ble_gamepad.sendReport();
}

bool JoyUtil::is_any_pressed () {
  for (byte btn = 0; btn < BUTTON_COUNT; btn++) {
    if (button_states[btn] == LOW) return true;
  }

  // these are at 0.0 in the resting position
  byte mid_axes[4] = { AXIS_LX, AXIS_LY, AXIS_RX, AXIS_RY };
  // these are at -1.0 in the resting position
  byte min_axes[2] = { AXIS_LT, AXIS_RT };

  byte i;
  for (i = 0; i < 4; i++) {
    if (axis_states[mid_axes[i]] > 0.5 || axis_states[mid_axes[i]] < -0.5) return true;
  }
  for (i = 0; i < 2; i++) {
    if (axis_states[min_axes[i]] > -0.5) return true;
  }
  return false;
}
