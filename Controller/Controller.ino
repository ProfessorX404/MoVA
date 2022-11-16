/*
Xavier Beech
UW SARP 2022-23

Encoder used is Nanotec NME2-SSI-V06-12-C
URL: https://us.nanotec.com/products/8483-nme2-ssi-v06-12-c
Motor used is Nanotec DB59l024035-A
URL: https://us.nanotec.com/products/2870-db59l024035-a

TODO:
- Homing routine
- Placeholders:
    - VALVE_OPEN_DEG
    = VALVE_CLOSED_DEG
    - isActivated()
    - getRevolutions()
- TUNE PID AND WIND UP CONSTANTS
- Count revolutions w Halls
*/

#include <ArduPID.h>

#define PWM_PIN 3                      // Motor control pin
#define DIR_PIN 5                      // Directional control pin
#define CLK_PIN 5                      // Encoder clock PWM pin
#define ENC_DATA_PIN 6                 // Encoder data input pin
#define H1_PIN A1                      // Hall effect sensor pins.
#define H2_PIN A2                      // Generally be used digitally, but
#define H3_PIN A3                      // mapped to analog pins for resolution.
#define C_FORWARD 1                    // Normalized forward vector. Swap to 0 if reversed.
#define C_REVERSE abs(C_FORWARD - 1)   // Normalized reverse vector. Always opposite of C_FORWARD.
#define ENC_TOT_BIT_CT 24              // Total number of bits in encoder packet. Last bit is error bit, success=1.
#define ENC_DATA_BIT_CT 17             // Data bits in encoder packet.
#define ENC_MIN_TIME_US 20             // Minimum amount of time between data calls, in milliseconds
#define CPU_MHZ 0x10                   // HARDWARE DEPENDENT!!! For accurate data reading timings. Eq. to Clocks/us
#define WIND_UP_MIN -10.0              // Integral growth bound min const
#define WIND_UP_MAX 10.0               // Integral growth bound max const
#define ENC_TICS_PER_MOTOR_REV 0x20000 // Number of encoder tics in mechanical revolution (per datasheet)
#define GEARBOX_RATIO 15               // Revs into gearbox per 1 revolution out
#define ENC_TICS_PER_VALVE_REV ENC_TICS_PER_MOTOR_REV * GEARBOX_RATIO // Post-gearbox encoder tics per valve revolution
#define VALVE_OPEN_DEG 90.0                                           // Encoder value for valve being fully open.
#define VALVE_CLOSED_DEG 0.0                                          // Encoder value for valve being fully closed
#define ENC_TICS_PER_VALVE_DEG (int)(ENC_TICS_PER_VALVE_REV / 360)    // Post-gearbox encoder tics / degree
#define TARGET_REVS (int)((VALVE_OPEN_DEG / 360) * GEARBOX_RATIO)     // Number of rotations to get almost fully open
#define HALL_CUTOFF 0xff      // Voltage level cutoff for a positive hall effect status
#define MAG_TIME_TO_FORM_MS 2 // Time in ms for magnetic fields to form large enough to be registered on Hall sensors

#define DELAY_US(n) __builtin_avr_delay_cycles(n * CPU_MHZ)
// Uses built in routine to skip clock cycles for timing purposes.

const bool HALL_COMBOS[6][3] = // Combinations of hall sensors based on motor angle, at 45deg increments
    {
  //    {H1, H2, H3}
        {1, 0, 1}, //  0->45
        {0, 0, 1}, //  45->90
        {0, 1, 1}, //  90->135
        {0, 1, 0}, //  135->180
        {1, 1, 0}, //  180->225
        {1, 0, 0}  //  225->270
};

const int HALL_COMBO_DEGS[7] = {0, .125, .25, .375, .5, .625, .75};

double output = 0;                                         // Signed PID output from -255 to 255.
double k_pid[3] = {0.0, 0.0, 0.0};                         // PID constants, in format [kP, kI, kD]
double target = VALVE_CLOSED_DEG * ENC_TICS_PER_VALVE_DEG; // Current valve position target. Init'ed to closed
double pos = 0.0;                                          // Current valve position
int errorCode = 0;                                         // Global error code variable for fault tracking.
bool activated = false;                                    // Flag for actuating valve.
bool withinOneRev = false;                                 // Flag for activating PID
bool motorCharged = false;                                 // Flag for current in motor
byte totalRevs = 0;                                        // Revolution counter
byte hallStatus[3] = {0, 0, 0};                            // Initial hall effect status, 0deg calibration
int degrees

    ArduPID pid; // PID instance

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
    pid.setOutputLimits(-255, 255);                // Motor can be actuated from 0-255 in either direction
    pid.setWindUpLimits(WIND_UP_MIN, WIND_UP_MAX); // Growth bounds to prevent integral wind-up
}

void loop() {
    if (!activated) { // Wait for system to be activated
        while (!isActivated()) {};
    }
    activated = true;
    withinOneRev = totalRevs > TARGET_REVS;

    if (!withinOneRev) { // If system has been activated but hasn't gotten to within 180* of final open position
        output = C_FORWARD * 255;
        updateEngineSpeed();
        if (motorCharged) {
            totalRevs = getRevolutions();
        } else {
            delay(MAG_TIME_TO_FORM_US);
            motorCharged = true;
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

// Returns binary representation of encoder readings
unsigned long readEncData() {
    noInterrupts();             // Deactivate interrupts for more accurate timing
    digitalWrite(CLK_PIN, LOW); // First bit is latch, always 1.
    DELAY_US(1);
    digitalWrite(CLK_PIN, HIGH);
    DELAY_US(1);
    if (!digitalRead(ENC_DATA_PIN)) { // If latch reads successfully, continue to data bits
        errorCode = 2;
        error(false);
        return -1.0;
    }

    unsigned long data = 0;
    for (int i = 0; i < ENC_TOT_BIT_CT - 1; i++) {
        data <<= 1;

        PORTD &= ~(1 << CLK_PIN); // clock pin goes low
        DELAY_US(1);              // Wait for 1us
        PORTD |= (1 << CLK_PIN);  // clock pin goes high
        DELAY_US(1);              // Wait for 1us
        data |= digitalRead(ENC_DATA_PIN);
    }

    if (!(data & ~(~0U << 1))) { // If last error bit is 0, internal encoder error occured
        errorCode = 3;
        error(true); // Throw fatal error
    }

    DELAY_US(20);
    interrupts(); // Reactivate interrupts

    return data >> ENC_TOT_BIT_CT - ENC_DATA_BIT_CT; // Return the first 17 bits of the data
}

// Outputs crash info to serial, if fatal hard crashes
void error(bool isFatal) {
    Serial.println("Error occured!");
    Serial.println("isFatal: " + isFatal);
    Serial.println("errorCode: " + errorCode);

    while (isFatal) {};
}

// Updates Hall sensor readings
void updateHallSensors() {
    hallStatus[0] = analogRead(H1_PIN >= HALL_CUTOFF);
    hallStatus[1] = analogRead(H2_PIN >= HALL_CUTOFF);
    hallStatus[2] = analogRead(H3_PIN >= HALL_CUTOFF);
}

bool isActivated() { return true; } // Placeholder

byte getRevolutions() {
    updateHallSensors();
    for (int i = 0; i < (*(&HALL_COMBOS + 1) - HALL_COMBOS); i++) {
        if ()
    }
}

void preciseDelay(int us) {
    noInterrupts();
    DELAY_US(us);
    interrupts();
}