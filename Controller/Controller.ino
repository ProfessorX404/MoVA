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

- Place following lines in arduino.json to allow it to build for AVR boards. Also, if using the Arduino nano
  in the Avionics cabinet in KWT, you must use the "ATmega328P (Old bootloader)" option in the board manager..
      "buildPreferences": [
              [
                  "build.extra_flags",
                  "-D__AVR_ATmega328P__"
              ]
          ]

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
- *Count revolutions w Halls
*/

#include <ArduPID.h>
#include <ArduinoSTL.h>
#include <SoftPWM.h>
#include <array>

#define FUEL                   0
#define OX                     1
#define CTRL                   0
#define DIR                    1
#define ENC_CLK                2
#define ENC_DATA               3
#define H1                     4
#define H2                     5
#define H3                     6
#define ADJ_L                  7
#define ADJ_R                  8
#define ADJ_ACT                9
#define ENC_TOT_BIT_CT         24      // Total number of bits in encoder packet. Last bit is error bit, success=1
#define ENC_DATA_BIT_CT        17      // Data bits in encoder packet
#define ENC_MIN_TIME_US        20      // Minimum amount of time between data calls, in milliseconds
#define WIND_UP_MIN            -10.0   // Integral growth bound min const
#define WIND_UP_MAX            10.0    // Integral growth bound max const
#define ENC_TICS_PER_MOTOR_REV 0x20000 // Number of encoder tics in mechanical revolution (per datasheet)
#define GEARBOX_RATIO          15      // Revs into gearbox per 1 revolution out
#define ENC_TICS_PER_VALVE_REV (ENC_TICS_PER_MOTOR_REV) * (GEARBOX_RATIO)      // Post-gearbox encoder tics per valve revolution
#define F_VALVE_OPEN_DEG       90.0                                            // Encoder value for valve being fully open
#define F_VALVE_CLOSED_DEG     0.0                                             // Encoder value for valve being fully closed
#define O_VALVE_OPEN_DEG       90.0                                            // Encoder value for valve being fully open
#define O_VALVE_CLOSED_DEG     0.0                                             // Encoder value for valve being fully closed
#define ENC_TICS_PER_VALVE_DEG (int)(ENC_TICS_PER_VALVE_REV / 360)             // Post-gearbox encoder tics / degree
#define F_TARGET_REVS          (int)((F_VALVE_OPEN_DEG / 360) * GEARBOX_RATIO) // Number of rotations to get almost fully open
#define O_TARGET_REVS          (int)((F_VALVE_OPEN_DEG / 360) * GEARBOX_RATIO) // Number of rotations to get almost fully open
#define HALL_CUTOFF            1023                                            // Voltage level cutoff for a positive hall effect status
#define MAG_TIME_TO_FORM_MS    2 // Time in ms for magnetic fields to form large enough to be registered on Hall sensors
#define N_COMBOS               6 // Number of posssible Hall combinations

static const int C_FORWARD = 1;                  // Normalized forward vector. Swap to 0 if reversed
static const int C_REVERSE = abs(C_FORWARD - 1); // Normalized reverse vector. Always opposite of C_FORWARD
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
static const std::array<std::array<byte, 10>, 2> PIN = {
  //  fo, Ox
    {10, 5, 1,  0,  A4, A5, A6, 9, 2, A0}, //  CTRL, DIr, ENC_CLK, ENC_DATA, H1, H2, H3, ADJ_L, ADJ_R, ADJ_ACT
    {8,  4, 12, 11, A3, A2, A1, 9, 2, A7}
};

std::array<double, 2> output = {0, 0}; // Signed pid[fo] output from -255 to 255.
std::array<std::array<double, 3>, 2> k_pid = {
    {0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0}
}; // pid[fo] constants, in format [kP, kI, kD]
std::array<double, 2> target = {{F_VALVE_CLOSED_DEG * ENC_TICS_PER_VALVE_DEG},
                                {O_VALVE_CLOSED_DEG * ENC_TICS_PER_VALVE_DEG}}; // Current valve position target. Init'ed to closed
std::array<double, 2> pos = {0.0, 0.0};                                         // Current valve position
byte errorCode = 0;                                                             // Global error code variable for fault tracking.
std::array<byte, 2> totalRevs = {0, 0};                                         // Revolution counter
std::array<std::array<bool, 3>, 2> hallStatus = {
    {0, 0, 0},
    {0, 0, 0}
};           // Initial hall effect status, 0deg calibration
std::array<byte, 2> initHallReading; // Index in HALL_COMBOS of the initial Hall sensor state

// Status flags
bool f_activated = false;                            // True if valve has been activated
std::array<bool, 2> f_withinOneRev = {false, false}; // True if totalRevs has passed TARGET_REVS
std::array<bool, 2> f_motorCharged = {false, false}; // True if motor has been charged long enough for Hall sensor readings to be valid.

std::array<ArduPID, 2> pid; // pid instances

void setup() {
    Serial.begin(57600); // Init serial connection
    byte fo = FUEL;
    initPins(fo);
    initPins(OX);

    analogWrite(PIN[fo][CTRL], 0); // Set motor to 0, intiailize correct orientation
    digitalWrite(PIN[fo][DIR], C_FORWARD);

    updateValvePos(fo);                                                                          // Initialize valve position
    pid[fo].begin(&pos[fo], &output[fo], &target[fo], k_pid[fo][0], k_pid[fo][1], k_pid[fo][2]); // Initialize pid[fo]
    pid[fo].setOutputLimits(-255, 255);                // Motor can be actuated from 0-255 in either direction
    pid[fo].setWindUpLimits(WIND_UP_MIN, WIND_UP_MAX); // Growth bounds to prevent integral wind-up
}

void loop() {
    if (!f_activated) { // Wait for system to be activated
        while (!isActivated()) {};
        f_activated = true;
    }
    byte fo = FUEL;
    f_withinOneRev[fo] = totalRevs[fo] > (fo == FUEL ? F_TARGET_REVS : O_TARGET_REVS);

    if (!f_withinOneRev[fo]) { // If system has been activated but hasn't gotten to within 180* of final open position
        output[fo] = C_FORWARD * 255;
        updateEngineSpeed(fo);
        if (f_motorCharged[fo]) {
            totalRevs[fo] = getRevolutions();
        } else {
            delay(MAG_TIME_TO_FORM_MS);
            updateHallSensors(fo);
            initHallReading[fo] = getHallSensorPosition(hallStatus[fo]);
            f_motorCharged[fo] = true;
        }
        return;
    }

    if (updateValvePos(fo)) { // If getting bad readings from enc, stop pid[fo] to prevent damage to system
        pid[fo].stop();
    } else { // When recovered, continue. Depending on testing, may sub for fatal error.
        pid[fo].start();
    }
    pid[fo].compute();     // Update motor target
    updateEngineSpeed(fo); // Set motor to target
}

// Takes input from global var, writes to pins accordingly
void updateEngineSpeed(byte fo) {
    digitalWrite(PIN[fo][DIR], output[fo] > 0 ? C_FORWARD : C_REVERSE);
    analogWrite(PIN[fo][CTRL], abs(output[fo]));
}

// Takes two readings from encoders, compares, and if they match, updates global var
// Possible issues: if the encoder moves enough to update between 20us minimum,
// it will never return valid value even if encoder is working as designed.
bool updateValvePos(byte fo) {
    unsigned long sample1 = readEncData(fo);
    unsigned long sample2 = readEncData(fo);

    if ((sample1 != sample2)) {
        errorCode = 1;
        error(false);
        return true;
    }
    pos[fo] = sample1;
    return false;
}

// Returns binary representation of encoder readings. Returns -1 if reading fails.
unsigned long readEncData(byte fo) {
    noInterrupts();                      // Deactivate interrupts for more accurate timing
    digitalWrite(PIN[fo][ENC_CLK], LOW); // First bit is latch, always 1.
    _delay_us(1);
    digitalWrite(PIN[fo][ENC_CLK], HIGH);
    _delay_us(1);
    if (!digitalRead(PIN[fo][ENC_DATA])) { // If latch reads successfully, continue to data bits
        errorCode = 2;
        error(false);
        return -1;
    }

    unsigned long data = 0;
    for (int i = 0; i < ENC_TOT_BIT_CT - 1; i++) {
        data <<= 1;

        PORTD &= ~(1 << PIN[fo][ENC_CLK]); // clock pin goes low
        _delay_us(1);                      // Wait for 1us
        PORTD |= (1 << PIN[fo][ENC_CLK]);  // clock pin goes high
        _delay_us(1);                      // Wait for 1us
        data |= digitalRead(PIN[fo][ENC_DATA]);
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

// Updates hallStatus
void updateHallSensors(byte fo) {
    hallStatus[fo][0] = (analogRead(PIN[fo][H1]) >= HALL_CUTOFF);
    hallStatus[fo][1] = (analogRead(PIN[fo][H2]) >= HALL_CUTOFF);
    hallStatus[fo][2] = (analogRead(PIN[fo][H3]) >= HALL_CUTOFF);
}

// Returns true if valve has been actuated by master controller.
bool isActivated() { return true; } // Placeholder

// Returns the number of complete revolutions the motor has completed.
// Accurate to within 60deg.
byte getRevolutions() { return 0; } // Placeholder

void initPins(byte fo) {
    pinMode(PIN[fo][CTRL], output[fo]);    // Motor magnitude control
    pinMode(PIN[fo][DIR], output[fo]);     // Motor direction control
    pinMode(PIN[fo][ENC_CLK], output[fo]); // Encoder serial clock

    pinMode(PIN[fo][ENC_DATA], INPUT); // Encoder data
    pinMode(PIN[fo][H1], INPUT);       // Hall effect sensors
    pinMode(PIN[fo][H2], INPUT);
    pinMode(PIN[fo][H3], INPUT);
}