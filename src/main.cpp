#include <Arduino.h>
#include <TickTwo.h>
#include <NintendoExtensionCtrl.h>

#include "joy_util.h"

// Green → SDA -> 19
// Red → 3v3
// White → GND
// Yellow → SCL -> 23

/* config */

// #define JOY_DEBUG 1

const uint REPORT_RATE = 1000; // micros, 1ms
const uint CLASSIC_POLL_RATE = 1000; // micros, 1ms

const float DEADZONE = 0.20;

const uint SLEEP_TIMEOUT = 5; // minutes
const uint SLEEP_BUTTON_TIMEOUT = 5; // seconds

// pins

const byte LED_PIN = 22;
const byte HOME_PIN = 4;

/* end config */

TaskHandle_t core0_handle;

JoyUtil joy("Wii Classic Gamepad", "Nintendo", DEADZONE);

bool IS_BLE_CONNECTED = false;
bool IS_CLASSIC_CONNECTED = false;

byte LED_STATE = HIGH;

bool CALIBRATION_MODE = false;

ClassicController classic;

void report ();
void report_calibrate ();
void toggle_led ();
void deep_sleep ();
void ble_conn_check ();
void i2c_conn_check ();
void classic_poll();

#ifdef JOY_DEBUG
void debug_common ();
TickTwo debug_interval(debug_common, 1000, 0); // 1 second
#endif

TickTwo report_interval(report, REPORT_RATE, 0, MICROS_MICROS);
TickTwo classic_poll_interval(classic_poll, CLASSIC_POLL_RATE, 0, MICROS_MICROS);
TickTwo ble_conn_check_interval(ble_conn_check, 1000, 0); // 1 second
TickTwo i2c_conn_check_interval(i2c_conn_check, 1000, 0); // 1 second
TickTwo calibrate_interval(report_calibrate, REPORT_RATE, 0, MICROS_MICROS);
TickTwo nc_led_interval(toggle_led, 1000, 0); // 1 second
TickTwo sleep_timeout(deep_sleep, SLEEP_TIMEOUT * 60000, 0);
TickTwo button_sleep_timeout(deep_sleep, SLEEP_BUTTON_TIMEOUT * 1000, 1);

void stop_all_timers () {
  report_interval.stop();
  i2c_conn_check_interval.stop();
  ble_conn_check_interval.stop();
  calibrate_interval.stop();
  nc_led_interval.stop();
  sleep_timeout.stop();
  button_sleep_timeout.stop();

  classic_poll_interval.stop();
}

void update_core1_timers () {
  report_interval.update();
  i2c_conn_check_interval.update();
  ble_conn_check_interval.update();
  calibrate_interval.update();
  nc_led_interval.update();
  sleep_timeout.update();
  button_sleep_timeout.update();

  #ifdef JOY_DEBUG
  debug_interval.update();
  #endif
}

void update_core0_timers () {
  classic_poll_interval.update();
}

void read_dpad () {
  joy.set_dpad_state(!classic.dpadUp(), !classic.dpadRight(), !classic.dpadDown(), !classic.dpadLeft());
}

void read_buttons () {
  joy.set_button_state(JoyUtil::BUTTON_A, !classic.buttonB());
  joy.set_button_state(JoyUtil::BUTTON_B, !classic.buttonA());
  joy.set_button_state(JoyUtil::BUTTON_X, !classic.buttonY());
  joy.set_button_state(JoyUtil::BUTTON_Y, !classic.buttonX());

  joy.set_button_state(JoyUtil::BUTTON_START, !classic.buttonPlus());
  joy.set_button_state(JoyUtil::BUTTON_SELECT, !classic.buttonMinus());
  joy.set_button_state(JoyUtil::BUTTON_HOME, !classic.buttonHome());

  joy.set_button_state(JoyUtil::BUTTON_LB, !classic.buttonL());
  joy.set_button_state(JoyUtil::BUTTON_RB, !classic.buttonR());
}

float map_axis_value (byte classic_axis_value, bool invert = false) {
  if (invert) return JoyUtil::map_range(classic_axis_value, 0.0, 255.0, 1.0, -1.0);
  return JoyUtil::map_range(classic_axis_value, 0.0, 255.0, -1.0, 1.0);
}

void read_axes () {
  joy.set_axis_state(JoyUtil::AXIS_LX, map_axis_value(classic.leftJoyX()));
  joy.set_axis_state(JoyUtil::AXIS_LY, map_axis_value(classic.leftJoyY(), true));
  joy.set_axis_state(JoyUtil::AXIS_RX, map_axis_value(classic.rightJoyX()));
  joy.set_axis_state(JoyUtil::AXIS_RY, map_axis_value(classic.rightJoyY(), true));
  joy.set_axis_state(JoyUtil::AXIS_LT, classic.buttonZL() ? 1.0 : -1.0);
  joy.set_axis_state(JoyUtil::AXIS_RT, classic.buttonZR() ? 1.0 : -1.0);
}

#ifdef JOY_DEBUG
uint classic_runs = 0;
unsigned long classic_timer = 0;
uint32_t report_runs = 0;
#endif

void core0_loop (void *p) {
  for(;;) {
    update_core0_timers();
  }
}

void loop () {
  update_core1_timers();
}

void classic_poll () {
  if (!IS_CLASSIC_CONNECTED) return;

  #ifdef JOY_DEBUG
  classic_runs++;
  unsigned long start = micros();
  #endif
  classic.update();

  read_dpad();
  read_buttons();
  read_axes();
  #ifdef JOY_DEBUG
  unsigned long end = micros();
  classic_timer += end - start;
  #endif
}

void setup () {
  #ifdef JOY_DEBUG
  Serial.begin(115200);
  Serial.print("setup\n");

  debug_interval.start();
  #endif

  xTaskCreatePinnedToCore(
    core0_loop, /* Function to implement the task */
    "core0_loop", /* Name of the task */
    10000,  /* Stack size in words */
    NULL,  /* Task input parameter */
    0,  /* Priority of the task */
    &core0_handle,  /* Task handle. */
    0 /* Core where the task should run */
  );

  joy.prefs_init();
  classic.begin();

  pinMode(LED_PIN, OUTPUT);

  esp_sleep_enable_ext0_wakeup((gpio_num_t)HOME_PIN, LOW);

  i2c_conn_check_interval.start();
}

void toggle_led () {
  digitalWrite(LED_PIN, LED_STATE = !LED_STATE);
}

void deep_sleep () {
  stop_all_timers();

  #ifdef JOY_DEBUG
  Serial.print("entering sleep mode\n");
  #endif

  if (joy.is_connected()) {
    joy.raise_inputs();
    joy.report();
  }

  // delay needed otherwise buttons remain pressed briefly on disconnect (?)
  delay(120);
  esp_deep_sleep_start();
}

void i2c_conn_check () {
  if (!classic.connect()) {
    IS_CLASSIC_CONNECTED = false;
    return;
  }

  IS_CLASSIC_CONNECTED = true;
  i2c_conn_check_interval.stop();

  classic.update();

  CALIBRATION_MODE = classic.buttonX();

  if (CALIBRATION_MODE) {

    // for (byte axis = 0; axis < JoyUtil::AXIS_COUNT; axis++) {
    //   joy.axis_min[axis] = 0.0;
    //   joy.axis_max[axis] = 0.0;
    // }

    #ifdef JOY_DEBUG
    Serial.print("calibration mode\n");
    Serial.print("put all axes in neutral position then press start\n");
    #endif

    calibrate_interval.start();

  } else {

    joy.prefs_read();

    #ifdef JOY_DEBUG
    for (byte axis = 0; axis < JoyUtil::AXIS_COUNT; axis++) {
      Serial.print(String(joy.axis_names[axis]) +
        " min: " + String(joy.get_axis_min(axis)) +
        ", mid: " + String(joy.get_axis_mid(axis)) +
        ", max: " + String(joy.get_axis_max(axis)) + "\n"
      );
    }
    #endif

    joy.connect();

    classic_poll_interval.start();
    report_interval.start();
    ble_conn_check_interval.start();
    nc_led_interval.start();
    sleep_timeout.start();
  }
}

float axes_min[6] = { 0.0, 0.0, 0.0, 0.0, 0.0 };
float axes_max[6] = { 0.0, 0.0, 0.0, 0.0, 0.0 };

byte CALIBRATION_STATE = 0;
byte start_state_old = HIGH;

void report_calibrate () {
  // classic.update();

  // byte old_start = joy.get_button_state(JoyUtil::BUTTON_START);

  // read_dpad();
  // read_buttons();
  // read_axes();
  classic_poll();

  byte start_state_new = joy.get_button_state(JoyUtil::BUTTON_START) == LOW;
  bool start_pressed = start_state_old == HIGH && start_state_new == LOW;
  start_state_old = start_state_new;

  if (CALIBRATION_STATE == 1) {
    for (byte axis = 0; axis < JoyUtil::AXIS_COUNT; axis++) {
      float state = joy.get_axis_state_raw(axis);
      if (state < axes_min[axis]) {
        axes_min[axis] = state;
        joy.set_axis_min(axis, state);
      } else if (state > axes_max[axis]) {
        axes_max[axis] = state;
        joy.set_axis_max(axis, state);
      }
    }
  } else if (CALIBRATION_STATE == 0) {
    for (byte axis = 0; axis < JoyUtil::AXIS_COUNT; axis++) {
      joy.set_axis_mid(axis, joy.get_axis_state_raw(axis));
    }
  }

  if (start_pressed) {
    if (CALIBRATION_STATE == 0) {
      CALIBRATION_STATE = 1;
      #ifdef JOY_DEBUG
      Serial.print("now move all axes full range then press start");
      #endif
    } else {
      // these are fake analogs on this controller
      joy.set_axis_min(JoyUtil::AXIS_LT, -1.0);
      joy.set_axis_mid(JoyUtil::AXIS_LT, 0.0);
      joy.set_axis_max(JoyUtil::AXIS_LT, 1.0);

      joy.set_axis_min(JoyUtil::AXIS_RT, -1.0);
      joy.set_axis_mid(JoyUtil::AXIS_RT, 0.0);
      joy.set_axis_max(JoyUtil::AXIS_RT, 1.0);

      joy.prefs_write();

      #ifdef JOY_DEBUG
      for (byte axis = 0; axis < JoyUtil::AXIS_COUNT; axis++) {
        Serial.print(
          String(joy.axis_names[axis]) +
          " min: " + String(joy.get_axis_min(axis)) +
          ", mid: " + String(joy.get_axis_mid(axis)) +
          ", max: " + String(joy.get_axis_max(axis)) + "\n"
        );
      }

      Serial.print("calibration done\n");
      #endif

      calibrate_interval.stop();

      delay(500);
      ESP.restart();
    }
  }
}

void ble_conn_check () {
  if (!joy.is_connected()) { // Bluetooth not connected
    if (IS_BLE_CONNECTED) { // was previously connected (is disconnected)
      #ifdef JOY_DEBUG
      Serial.print("bluetooth disconnected\n");
      #endif
      digitalWrite(LED_PIN, HIGH);
      nc_led_interval.start();
      IS_BLE_CONNECTED = false;
    }
    return;
  } else { // Bluetooth connected
    if (!IS_BLE_CONNECTED) {
      #ifdef JOY_DEBUG
      Serial.print("bluetooth connected\n");
      #endif

      joy.raise_inputs();

      nc_led_interval.stop();
      digitalWrite(LED_PIN, LED_STATE = LOW);

      IS_BLE_CONNECTED = true;
    }
  }
}

void report () {
  #ifdef JOY_DEBUG
  report_runs++;
  #endif

  // FORCE SLEEP

  if (joy.get_button_state(JoyUtil::BUTTON_SELECT) == LOW) {
    if (button_sleep_timeout.state() != RUNNING) button_sleep_timeout.start();
  } else {
    button_sleep_timeout.stop();
  }

  if (joy.is_any_pressed()) {
    sleep_timeout.stop();
    sleep_timeout.start();
  }

  joy.report();
}

#ifdef JOY_DEBUG
void debug_common () {
  for (byte btn = 0; btn < JoyUtil::BUTTON_COUNT; btn++) {
    const std::string spacer = (btn < JoyUtil::BUTTON_COUNT - 1) ? ", " : "";
    Serial.print(
      String(joy.button_names[btn]) +
      ": " + String(joy.get_button_state(btn)) + spacer.c_str()
    );
  }
  Serial.print(" :: ");
  for (byte axis = 0; axis < JoyUtil::AXIS_COUNT; axis++) {
    const std::string spacer = (axis < JoyUtil::AXIS_COUNT - 1) ? ", " : "";
    Serial.print(
      String(joy.axis_names[axis]) +
      ": " + String(joy.get_axis_state(axis)) + "(" + String(joy.get_axis_state_raw(axis)) + ")" + spacer.c_str()
    );
  }
  Serial.print(":: DP: " + String(joy.get_dpad_state()) + "\n");
  if (classic_runs > 0) Serial.print("classic runs timer: " + String(classic_timer / classic_runs) + " :: ");
  Serial.print("report runs: " + String(report_runs) + " :: classic runs: " + String(classic_runs) + "\n");

  report_runs = 0;
  classic_runs = 0;
  classic_timer = 0;
}
#endif
