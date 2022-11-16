#include <Arduino.h>
#line 1 "C:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
#include <ArduPID.h>
#include <string.h>
// Encoder used is Nanotec NME2-SSI-V06-12-C
#define PWM_PIN 3                    // Motor control pin
#define DIR_PIN 5                    // Directional control pin
#define CLK_PIN 5                    // Encoder clock PWM pin
#define ENC_DATA_PIN 6               // Encoder data input pin
#define C_FORWARD 1                  // Normalized forward vector. Swap to 0 if reversed.
#define C_REVERSE abs(C_FORWARD - 1) // Normalized reverse vector. Always opposite of C_FORWARD.
#define ENC_TOT_BIT_CT 24            // Total number of bits in encoder packet. Last bit is error bit, success=1.
#define ENC_DATA_BIT_CT 17           // Data bits in encoder packet.
#define ENC_MIN_TIME_US 20           // Minimum amount of time between data calls, in milliseconds
#define CPU_MHZ 0x10                 // HARDWARE DEPENDENT!!! For accurate data reading timings. Eq. to Clocks/us
#define VALVE_OPEN 90.0              // Encoder value for valve being fully open.
#define VALVE_CLOSED 0.0             // Encoder value for valve being fully closed
#define WIND_UP_MIN -10.0            // Integral growth bound min const
#define WIND_UP_MAX 10.0             // Integral growth bound max const
#define ENC_TICS_PER_REV 0x20000     // Number of encoder tics in mechanical revolution (per datasheet)

#define DELAY_US(n) __builtin_avr_delay_cycles(n *CPU_MHZ)
// Uses built in routine to skip clock cycles for timing purposes.

double output = 0;                 // Signed PID output from -255 to 255.
double k_pid[3] = {0.0, 0.0, 0.0}; // PID constants, in format [kP, kI, kD]
double target = VALVE_CLOSED;      // Current valve position target. Init'ed to closed
double pos = 0.0;                  // Current valve position
int errorCode = 0;                 // Global error code variable for fault tracking.

/* TODO:
-Homing routine
-Change placeholder values to VALVE_OPEN and VALVE_CLOSED
- TUNE PID AND WIND UP CONSTANTS
- Work out how to count revolutions
*/
ArduPID pid; // PID instance

#line 37 "C:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
void setup();
#line 56 "C:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
void loop();
#line 108 "C:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
void updateEngineSpeed();
#line 125 "C:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
bool updateValvePos();
#line 138 "C:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
unsigned long readEncData();
#line 170 "C:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
void error(bool isFatal);
#line 37 "C:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
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
    pid.setSampleTime(ENC_MIN_TIME_MS);
    pid.setOutputLimits(-255, 255);                // Motor can be actuated from 0-255 in either direction
    pid.setWindUpLimits(WIND_UP_MIN, WIND_UP_MAX); // Growth bounds for the integral term to prevent integral wind-up

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

    /*
    if (Serial.available() > 0) {
        String input = Serial.readString();
        input.trim();
        if (input.substring(0, 1) == "!") {
            switch ((char)input.substring(1, 2)) {
                case 'P':
                    p = input.substring(2).toFloat();
                    Serial.print("Set P to: ");
                    Serial.println(p);
                    break;
                case 'I':
                    i = input.substring(2).toFloat();
                    Serial.print("Set I to: ");
                    Serial.println(I);
                    break;
                case 'D':
                    d = input.substring(2).toFloat();
                    Serial.print("Set D to: ");
                    Serial.println(d);
                    break;
            }
            pid.setCoefficients(p, i, d);
        } else if (input.substring(0, 1) == "%") {
            float incoming = parseCommand(input.substring(1).toFloat());
            byte newDir = (incoming >= 0) ? C_FORWARD : C_REVERSE;
            byte newVal = (byte)abs((255 * incoming));
            Serial.print(">Set duty cycle to " + (String)(incoming * 100.0) + "% or ");
            Serial.println((newDir == C_FORWARD ? "" : "-") + (String)newVal + "/255");
            updateEngineSpeed(newVal, newDir);
        } else {
            int incoming = input.toInt();
            byte newDir = (incoming >= 0) ? C_FORWARD : C_REVERSE;
            byte newVal = (byte)abs(incoming);
            Serial.println(">Set duty cycle to " + (String)(newDir == C_FORWARD ? "" : "-") + (String)(newVal) +
                           "/255");
            updateEngineSpeed(newVal, newDir);
        }
    }
    */
}

void updateEngineSpeed() {
    digitalWrite(DIR_PIN, output > 0 ? C_FORWARD : C_REVERSE);
    analogWrite(PWM_PIN, abs(output));
}
/*
float parseCommand(float input) {
    if (abs(input) > 100) {
        while (abs(input) > 1) {
            input = input / 10;
        }
    } else {
        input = input / 100;
    }
    return input;
}
*/

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
    delayMicroseconds(1);
    digitalWrite(CLK_PIN, HIGH);
    delayMicroseconds(1);
    if (!digitalRead(ENC_DATA_PIN)) {
        errorCode = 2;
        error(false);
        return -1.0;
    }

    unsigned long data = 0; // If latch reads successfully, continue to data bits
    for (int i = 0; i < ENC_TOT_BIT_CT; i++) {
        data <<= 1;
        digitalWrite(CLK_PIN, LOW);
        delayMicroseconds(1);
        digitalWrite(CLK_PIN, HIGH);
        delayMicroseconds(1);

        data |= digitalRead(ENC_DATA_PIN);
    }

    if (!(data & ~(~0U << 1))) { // If last error bit is 0, internal encoder error occured
        errorCode = 3;
        error(true);
    }
    delayMicroseconds(ENC_MIN_TIME_MS * 10);
    interrupts();
    return data >> ENC_TOT_BIT_CT - ENC_DATA_BIT_CT;
}

void error(bool isFatal) {
    Serial.println("Error occured!");
    Serial.println("isFatal: " + isFatal);
    Serial.println("errorCode: " + errorCode);

    while (isFatal) {};
}
