/*
 * General configuration for HoverBot.ino
 * Tested on an Arduino Mega
 */

// HARDWARE
#define LEDPIN 13

#define MOTORDIR_0 -1
#define MOTORDIR_1 1
#define ENGAGE_THRESHOLD 1500

// CONTROLLER
#define KP_BALANCE 0.8
#define KD_BALANCE -0.065
#define KD_ORIENTATION 0.01
#define KP_POSITION 0.015
#define KP_STEERING 0.01
#define TILT_LIMIT 40
#define PWM_CENTER 140

// SCHEDULING
#define BLINK_INTERVAL 200
#define CONTROLLER_INTERVAL 10
#define ACTIVATION_INTERVAL 50

// SERIAL
#define BAUDRATE_ODRIVE 115200
#define BAUDRATE_PC 115200
#define BAUDRATE_BT 115200
