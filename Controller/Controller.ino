/*
Xavier Beech
UW SARP 2022-23
Motorized valve actuation controller software for 2022 Pacific Impulse.
Must comment line 22 of [ArduinoSTL location]/ArduinoSTL/src/new_handler.cpp, or code will not compile.when using v1.8.3 or later

Encoder used is Nanotec NME2-SSI-V06-12-C
URL: https://us.nanotec.com/products/8483-nme2-ssi-v06-12-c
Motor used is Nanotec DB59l024035-A
URL: https://us.nanotec.com/products/2870-db59l024035-a

Libraries used (attached in ../lib for convenience):
 - ArduPID https://github.com/PowerBroker2/ArduPID
 - FireTimer https://github.com/PowerBroker2/FireTimer (ArduPID dependency)
 - ArduinoSTL https://github.com/mike-matera/ArduinoSTL
 - SoftPWM https://github.com/bhagman/SoftPWM

If using VSCode:

  - Arduino extension does not work (for me) with Arduino CLI even though the documentation says it does,
  I have to use legacy Arduino IDE v1.8.19

- Manual zip extraction does not work.for library installation, you must go through the library manager.

TODO:
- Manual adjustment
- Placeholders:
    - VALVE_OPEN_DEG
    - VALVE_CLOSED_DEG
    - HALL_CUTOFF
    - isActivated()
    - getRevolutions()*
- TUNE pid[fo] AND WIND UP CONSTANTS
- *Count revolutions
*/
#include <array.h>

#define FUEL                   0 // Array index constants for easy access
#define OX                     1
#define CTRL                   0
#define DIR                    1
#define ENC_CLK                2
#define ENC_DATA               3
#define ADJ_SELECT             4
#define P                      0
#define I                      1
#define D                      2
#define ENC_TOT_BIT_CT         24      // Total number of bits in encoder packet. Last bit is error bit, success=1
#define ENC_DATA_BIT_CT        17      // Data bits in encoder packet
#define ENC_MIN_TIME_US        20      // Minimum amount of time between data calls, in milliseconds
#define WIND_UP_MIN            -10.0   // Integral growth bound min const
#define WIND_UP_MAX            10.0    // Integral growth bound max const
#define ENC_TICS_PER_MOTOR_REV 0x20000 // Number of encoder tics in mechanical revolution (per datasheet)
#define GEARBOX_RATIO          15      // Revs into gearbox per 1 revolution out
#define ENC_TICS_PER_VALVE_REV (ENC_TICS_PER_MOTOR_REV) * (GEARBOX_RATIO)    // Post-gearbox encoder tics per valve revolution
#define VALVE_OPEN_DEG         90.0                                          // Encoder value for valve being fully open
#define VALVE_CLOSED_DEG       0.0                                           // Encoder value for valve being fully closed
#define ENC_TICS_PER_VALVE_DEG (int)(ENC_TICS_PER_VALVE_REV / 360)           // Post-gearbox encoder tics / degree
#define TARGET_REVS            (int)((VALVE_OPEN_DEG / 360) * GEARBOX_RATIO) // Number of rotations to get almost fully open

static const int C_FORWARD = 1;                  // Normalized forward vector. Swap to 0 if reversed
static const int C_REVERSE = abs(C_FORWARD - 1); // Normalized reverse vector. Opposite of C_FORWARD

static const array<array<byte, 10>, 2> PIN = {
  //  Fuel, Ox
    {{2, 3, 11, 12, 7}, //  CTRL (PWM), DIR, ENC_CLK (MOSI), ENC_DATA (MISO), ADJ_SELECT
     {6, 5, A1, A2, 4}}
};

array<array<double, 3>, 2> k_pid = {
    {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}
}; // pid[fo] constants, in format [kP, kI, kD]

array<unsigned long, 2> accumulator = {0.0, 0.0}; // PID integral term accumulator
array<unsigned long, 2> prev_pos = {0.0, 0.0};    // PID theta_n-1
array<double, 2> lastTime = {0.0, 0.0};           // time of th_(n-1) for use computing dt

unsigned long target = VALVE_CLOSED_DEG * ENC_TICS_PER_VALVE_DEG; // Current valve position target. Init'ed to closed
byte errorCode = 0;                                               // Global error code variable for fault tracking.
array<byte, 2> totalRevs = {0, 0};                                // Revolution counter

// Status flags
bool f_activated = false;                       // True if valve has been activated
array<bool, 2> f_withinOneRev = {false, false}; // True if totalRevs has passed TARGET_REVS

void setup() {
    Serial.begin(57600); // Init serial connection
    byte fo = FUEL;
    pinPeripheral()
}

void loop() {
    if (!f_activated) { // Wait for system to be activated
        while (!isActivated()) {};
        f_activated = true;
    }
}

// Takes input from global var, writes to pins accordingly
void update(byte fo) {
    unsigned long delta_t = getMicros() - lastTime[fo];
    unsigned long theta_n = readEncData(fo);
    accumulator[fo] += (delta_t << 1) * (theta_n + prev_pos[fo] - (target >> 1));

    double O = k_pid[fo][P] * (theta_n - target) + k_pid[fo][I] * accumulator[fo] + k_pid[fo][D] * ((theta_n - prev_pos[fo]) / delta_t);

    if (O < 0 && C_FORWARD) {
        PORTD |= ~(1 << PIN[fo][DIR]); // off
    } else {
        PORTD &= ~(1 << PIN[fo][DIR]); // on
    }

    analogWrite(PIN[fo][CTRL], abs(O) > 255 ? 255 : abs(O));
}

// Takes two readings from encoders, compares, and if they match, returns value.
// If they do not, discards data and throws non-fatal error.
// Possible issues: if the encoder moves enough to update between 20us minimum,
// it will never return valid value even if encoder is working as designed.
// Returns binary representation of encoder readings. Returns -1 if reading fails.

unsigned long readEncData(byte fo) { return 0x64; } // placeholder

// Outputs error info to serial, if fatal error stalls program
void error(bool isFatal) {
    Serial.println("Error occured!");
    Serial.println("isFatal: " + isFatal);
    Serial.println("errorCode: " + errorCode);

    while (isFatal) {};
}

// Returns true if valve has been actuated by master controller.
bool isActivated() { return true; } // Placeholder

// Returns the number of complete revolutions the motor has completed.
// Accurate to within 60deg.
byte getRevolutions() { return 0; } // Placeholder

void attachPins() {} // Use after de-safing rocket but before launch activation.

double getMicros() { return -1; } // Placeholder