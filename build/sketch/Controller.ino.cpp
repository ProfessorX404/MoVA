#include <Arduino.h>
#line 1 "C:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
#include <ArduPID.h>
#include <vector.h>
// Encoder used is Nanotec NME2-SSI-V06-12-C
// URL: https://us.nanotec.com/products/8483-nme2-ssi-v06-12-c
// Motor used is Nanotec DB59l024035-A
// URL: https://us.nanotec.com/products/2870-db59l024035-a
#define PWM_PIN 3                      // Motor control pin
#define DIR_PIN 5                      // Directional control pin
#define CLK_PIN 5                      // Encoder clock PWM pin
#define ENC_DATA_PIN 6                 // Encoder data input pin
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
#define ENC_TICS_PER_VALVE_DEG ENC_TICS_PER_VALVE_REV / 360           // Post-gearbox encoder tics / degree

#define DELAY_US(n) __builtin_avr_delay_cycles(n * CPU_MHZ)
// Uses built in routine to skip clock cycles for timing purposes.

double output = 0;                 // Signed PID output from -255 to 255.
double k_pid[3] = {0.0, 0.0, 0.0}; // PID constants, in format [kP, kI, kD]
double target = VALVE_CLOSED;      // Current valve position target. Init'ed to closed
double pos = 0.0;                  // Current valve position
int errorCode = 0;                 // Global error code variable for fault tracking.

/* TODO:
- Homing routine
- Change placeholder values to VALVE_OPEN and VALVE_CLOSED
- TUNE PID AND WIND UP CONSTANTS
- Count revolutions
*/

ArduPID pid; // PID instance

void setup() {
    Serial.begin(57600);

    pinMode(PWM_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(CLK_PIN, OUTPUT);
    pinMode(ENC_DATA_PIN, INPUT);
    analogWrite(PWM_PIN, 0);
    digitalWrite(DIR_PIN, C_FORWARD);

    updateValvePos();
    pid.begin(&pos, &output, &target, k_pid[0], k_pid[1], k_pid[2]);
    pid.setOutputLimits(-255, 255);                // Motor can be actuated from 0-255 in either direction
    pid.setWindUpLimits(WIND_UP_MIN, WIND_UP_MAX); // Growth bounds to prevent integral wind-up

    pid.start();
}

void loop() {

    if (updateValvePos()) {
        pid.stop();
    } else {
        pid.start();
    }
    pid.compute();
    updateEngineSpeed();
}

void updateEngineSpeed() {
    digitalWrite(DIR_PIN, output > 0 ? C_FORWARD : C_REVERSE);
    analogWrite(PWM_PIN, abs(output));
}

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

unsigned long readEncData() {
    noInterrupts();
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
    for (int i = 0; i < ENC_TOT_BIT_CT - 1; i++) { // Read in all 17 data bits
        data <<= 1;
        digitalWrite(CLK_PIN, LOW);
        DELAY_US(1);
        digitalWrite(CLK_PIN, HIGH);
        DELAY_US(1);

        data |= digitalRead(ENC_DATA_PIN);
    }

    if (!(data & ~(~0U << 1))) { // If last error bit is 0, internal encoder error occured
        errorCode = 3;
        error(true);
    }

    DELAY_US(20);
    interrupts();

    return data >> ENC_TOT_BIT_CT - ENC_DATA_BIT_CT;
}

void error(bool isFatal) {
    Serial.println("Error occured!");
    Serial.println("isFatal: " + isFatal);
    Serial.println("errorCode: " + errorCode);

    while (isFatal) {};
}
