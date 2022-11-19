#include <Arduino.h>
#line 1 "c:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
/*
Xavier Beech
UW SARP 2022-23
Motorized valve actuation controller software for 2022 Pacific Impulse.

Encoder used is Nanotec NME2-SSI-V06-12-C
URL: https://us.nanotec.com/products/8483-nme2-ssi-v06-12-c
Motor used is Nanotec DB59l024035-A
URL: https://us.nanotec.com/products/2870-db59l024035-a

Libraries used (attached in ../lib for convenience):
 - ArduPID https://github.com/PowerBroker2/ArduPID
 - FireTimer https://github.com/PowerBroker2/FireTimer (ArduPID dependency)
 - ArduinoSTL https://github.com/mike-matera/ArduinoSTL

If using VSCode:

- If for some reason you need to reinitialize/regen arduino.json, make sure to add
      "buildPreferences": [
              [
                  "build.extra_flags",
                  "-D__AVR_ATmega328P__"
              ]
          ]
  at the bottom so that the compiler knowns which chip it is working with. If hardware does not use a
  328P chip (Uno, Nano, etc.) replace the tag with the correct value.

  - Arduino extension does not work (for me) with Arduino CLI even though the documentation says it does,
  I have to use legacy Arduino IDE v1.8.19

- The only consistent method I've found for getting the extension to find the libraries and include them in
  c_cpp_properties.json' includePath is to install them via the library manager, manual zip extraction does not work.

IMPORTANT!! Before deployment, verify F_CPU is correct for the board you are using. If it is not, all serial communication will
break.

TODO:
- Homing routine (may just end up being manual adjustment)
- Placeholders:
    - VALVE_OPEN_DEG
    - VALVE_CLOSED_DEG
    - HALL_CUTOFF
    - isActivated()
    - getRevolutions()*
- TUNE PID AND WIND UP CONSTANTS
- *Count revolutions w Halls
*/

#include <ArduPID.h>
#include <ArduinoSTL.h>
#include <array>

#define PWM_PIN                3                  // Motor control pin
#define DIR_PIN                5                  // Directional control pin
#define CLK_PIN                5                  // Encoder clock PWM pin
#define ENC_DATA_PIN           6                  // Encoder data input pin
#define H1_PIN                 A1                 // Hall effect sensor pins.
#define H2_PIN                 A2                 // Generally to be used digitally, but
#define H3_PIN                 A3                 // mapped to analog pins for noise-filtering
#define C_FORWARD              1                  // Normalized forward vector. Swap to 0 if reversed
#define C_REVERSE              abs(C_FORWARD - 1) // Normalized reverse vector. Always opposite of C_FORWARD
#define ENC_TOT_BIT_CT         24                 // Total number of bits in encoder packet. Last bit is error bit, success=1
#define ENC_DATA_BIT_CT        17                 // Data bits in encoder packet
#define ENC_MIN_TIME_US        20                 // Minimum amount of time between data calls, in milliseconds
#define WIND_UP_MIN            -10.0              // Integral growth bound min const
#define WIND_UP_MAX            10.0               // Integral growth bound max const
#define ENC_TICS_PER_MOTOR_REV 0x20000            // Number of encoder tics in mechanical revolution (per datasheet)
#define GEARBOX_RATIO          15                 // Revs into gearbox per 1 revolution out
#define ENC_TICS_PER_VALVE_REV ENC_TICS_PER_MOTOR_REV *GEARBOX_RATIO         // Post-gearbox encoder tics per valve revolution
#define VALVE_OPEN_DEG         90.0                                          // Encoder value for valve being fully open
#define VALVE_CLOSED_DEG       0.0                                           // Encoder value for valve being fully closed
#define ENC_TICS_PER_VALVE_DEG (int)(ENC_TICS_PER_VALVE_REV / 360)           // Post-gearbox encoder tics / degree
#define TARGET_REVS            (int)((VALVE_OPEN_DEG / 360) * GEARBOX_RATIO) // Number of rotations to get almost fully open
#define HALL_CUTOFF            1023                                          // Voltage level cutoff for a positive hall effect status
#define MAG_TIME_TO_FORM_MS    2 // Time in ms for magnetic fields to form large enough to be registered on Hall sensors
#define N_COMBOS               6 // Number of posssible Hall combinations

// Combinations of hall sensors based on motor angle, at 60 deg increments
static const std::array<std::array<bool, 3>, N_COMBOS> HALL_COMBOS = {
  //    {H1, H2, H3}
    {1, 0, 1}, //  0
    {0, 0, 1}, //  60
    {0, 1, 1}, //  120
    {0, 1, 0}, //  180
    {1, 1, 0}, //  240
    {1, 0, 0}  //  300, loop around to 0/360
};

// Decimal approximation of how many revolutions each entry in HALL_COMBOS is from 0.
static const std::array<double, N_COMBOS> HALL_COMBO_DEGS = {0, 60 / 360, 120 / 360, 180 / 360, 240 / 360, 300 / 360};

double output = 0;                                         // Signed PID output from -255 to 255.
std::array<double, 3> k_pid = {0.0, 0.0, 0.0};             // PID constants, in format [kP, kI, kD]
double target = VALVE_CLOSED_DEG * ENC_TICS_PER_VALVE_DEG; // Current valve position target. Init'ed to closed
double pos = 0.0;                                          // Current valve position
int errorCode = 0;                                         // Global error code variable for fault tracking.
byte totalRevs = 0;                                        // Revolution counter
std::array<bool, 3> hallStatus = {0, 0, 0};                // Initial hall effect status, 0deg calibration
byte initHallReading;                                      // Index in HALL_COMBOS of the initial Hall sensor state

// Status flags
bool f_activated = false;    // True if valve has been activated
bool f_withinOneRev = false; // True if totalRevs has passed TARGET_REVS
bool f_motorCharged = false; // True if motor has been charged long enough for Hall sensor readings to be valid.

ArduPID pid; // PID instance

#line 108 "c:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
void setup();
#line 130 "c:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
void loop();
#line 161 "c:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
void updateEngineSpeed();
#line 169 "c:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
bool updateValvePos();
#line 183 "c:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
unsigned long readEncData();
#line 218 "c:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
void error(bool isFatal);
#line 228 "c:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
byte getHallSensorPosition(std::array<bool, 3> given);
#line 246 "c:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
bool isActivated();
#line 250 "c:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
byte getRevolutions();
#line 108 "c:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
void setup() {
    Serial.begin(57600); // Init serial connection

    // Init pin values
    pinMode(PWM_PIN, OUTPUT); // Motor magnitude control
    pinMode(DIR_PIN, OUTPUT); // Motor direction control
    pinMode(CLK_PIN, OUTPUT); // Encoder serial clock

    pinMode(ENC_DATA_PIN, INPUT); // Encoder data
    pinMode(H1_PIN, INPUT);       // Hall effect sensors
    pinMode(H2_PIN, INPUT);
    pinMode(H3_PIN, INPUT);

    analogWrite(PWM_PIN, 0); // Set motor to 0, intiailize correct orientation
    digitalWrite(DIR_PIN, C_FORWARD);

    updateValvePos();                                                // Initialize valve position
    pid.begin(&pos, &output, &target, k_pid[0], k_pid[1], k_pid[2]); // Initialize PID
    pid.setOutputLimits(-255, 255);                                  // Motor can be actuated from 0-255 in either direction
    pid.setWindUpLimits(WIND_UP_MIN, WIND_UP_MAX);                   // Growth bounds to prevent integral wind-up
}

void loop() {
    if (!f_activated) { // Wait for system to be activated
        while (!isActivated()) {};
        f_activated = true;
    }

    f_withinOneRev = totalRevs > TARGET_REVS;

    if (!f_withinOneRev) { // If system has been activated but hasn't gotten to within 180* of final open position
        output = C_FORWARD * 255;
        updateEngineSpeed();
        if (f_motorCharged) {
            totalRevs = getRevolutions();
        } else {
            delay(MAG_TIME_TO_FORM_MS);
            initHallReading = getHallSensorPosition(readHallSensors());
            f_motorCharged = true;
        }
        return;
    }

    if (updateValvePos()) { // If getting bad readings from enc, stop pid to prevent damage to system
        pid.stop();
    } else { // When recovered, continue. Depending on testing, may sub for fatal error.
        pid.start();
    }
    pid.compute();       // Update motor target
    updateEngineSpeed(); // Set motor to target
}

// Takes input from global var, writes to pins accordingly
void updateEngineSpeed() {
    digitalWrite(DIR_PIN, output > 0 ? C_FORWARD : C_REVERSE);
    analogWrite(PWM_PIN, abs(output));
}

// Takes two readings from encoders, compares, and if they match, updates global var
// Possible issues: if the encoder moves enough to update between 20us minimum,
// it will never return valid value even if encoder is working as designed.
bool updateValvePos() {
    unsigned long sample1 = readEncData();
    unsigned long sample2 = readEncData();

    if ((sample1 != sample2)) {
        errorCode = 1;
        error(false);
        return true;
    }
    pos = sample1;
    return false;
}

// Returns binary representation of encoder readings. Returns -1 if reading fails.
unsigned long readEncData() {
    noInterrupts();             // Deactivate interrupts for more accurate timing
    digitalWrite(CLK_PIN, LOW); // First bit is latch, always 1.
    _delay_us(1);
    digitalWrite(CLK_PIN, HIGH);
    _delay_us(1);
    if (!digitalRead(ENC_DATA_PIN)) { // If latch reads successfully, continue to data bits
        errorCode = 2;
        error(false);
        return -1;
    }

    unsigned long data = 0;
    for (int i = 0; i < ENC_TOT_BIT_CT - 1; i++) {
        data <<= 1;

        PORTD &= ~(1 << CLK_PIN); // clock pin goes low
        _delay_us(1);             // Wait for 1us
        PORTD |= (1 << CLK_PIN);  // clock pin goes high
        _delay_us(1);             // Wait for 1us
        data |= digitalRead(ENC_DATA_PIN);
    }

    if (!(data & ~(~0U << 1))) { // If last error bit is 0, internal encoder error occured
        errorCode = 3;
        error(true); // Throw fatal error
    }

    _delay_us(20);
    interrupts(); // Reactivate interrupts

    return data >> (ENC_TOT_BIT_CT - ENC_DATA_BIT_CT); // Return the first 17 bits of the data
}

// Outputs error info to serial, if fatal error stalls program
void error(bool isFatal) {
    Serial.println("Error occured!");
    Serial.println("isFatal: " + isFatal);
    Serial.println("errorCode: " + errorCode);

    while (isFatal) {};
}

// Returns the index of the entry in HALL_COMBOS which is the same
// as the given array. Returns -1 if no match found.
byte getHallSensorPosition(std::array<bool, 3> given) {
    for (int i = 0; i < HALL_COMBOS.size(); i++) {
        if ((given[0] == HALL_COMBOS[i][0]) && (given[1] == HALL_COMBOS[i][1]) && (given[2] == HALL_COMBOS[i][2])) {
            return i;
        }
    }
    return -1;
}

// Updates hallStats and returns Hall sensor readings
std::array<bool, 3> readHallSensors() {
    hallStatus[0] = (analogRead(H1_PIN) >= HALL_CUTOFF);
    hallStatus[1] = (analogRead(H2_PIN) >= HALL_CUTOFF);
    hallStatus[2] = (analogRead(H3_PIN) >= HALL_CUTOFF);
    return hallStatus;
}

// Returns true if valve has been actuated by master controller.
bool isActivated() { return true; } // Placeholder

// Returns the number of complete revolutions the motor has completed.
// Accurate to within 60deg.
byte getRevolutions() { return 0; } // Placeholder
