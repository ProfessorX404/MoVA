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

OTMX Values:
CCx | WO[n]
CC0 | WO[0], WO[4]
CC1 | WO[1], WO[5]
CC2 | WO[2], WO[6]
CC3 | WO[3], WO[7]
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
#define PWM_FREQ_COEF          1262                                          // 48MHz / (1262 + 1) = 38kHz
#define TCC_FUEL               TCC1
#define TCC_OX                 TCC0
#define REGISTER_MOSI          11
#define REGISTER_LATCH         8

static const int C_FORWARD = 1;                  // Normalized forward vector. Swap to 0 if reversed
static const int C_REVERSE = abs(C_FORWARD - 1); // Normalized reverse vector. Opposite of C_FORWARD

static const array<array<byte, 10>, 2> PIN = {
  //  Fuel, Ox
    {{A2, 3, 13, 12, 10}, //  CTRL (PWM), DIR, ENC_CLK (SCK), ENC_DATA (MISO), ADJ_SELECT
     {A3, 5, 5, 6, 4}}
};

array<array<double, 3>, 2> k_pid = {
    {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}
}; // pid[fo] constants, in format [kP, kI, kD]

array<unsigned long, 2> accumulator = {0, 0}; // PID integral term accumulator
array<unsigned long, 2> prev_pos = {0, 0};    // PID theta_n-1
array<double, 2> lastTime = {0.0, 0.0};       // time of th_(n-1) for use computing dt

unsigned long target = VALVE_CLOSED_DEG * ENC_TICS_PER_VALVE_DEG; // Current valve position target. Init'ed to closed
byte errorCode = 0;                                               // Global error code variable for fault tracking.
array<byte, 2> totalRevs = {0, 0};                                // Revolution counter

// Status flags
bool f_activated = false;                       // True if valve has been activated
array<bool, 2> f_withinOneRev = {false, false}; // True if totalRevs has passed TARGET_REVS

void setup() {
    Serial.begin(115200); // Init serial connection
    byte fo = FUEL;
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
    O = (O > PWM_FREQ_COEF) ? PWM_FREQ_COEF : O;
    if (O < 0 && C_FORWARD) {

    } else {
    }

    if (fo == FUEL) {
        TCC_FUEL->CCB[0].reg = O;
        while (TCC_FUEL->SYNCBUSY.bit.CCB0)
            ;
    } else {
        TCC_OX->CCB[0].reg = O;
        while (TCC_OX->SYNCBUSY.bit.CCB0)
            ;
    }
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

void attachPins() {
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |       // Enable GCLK0 as a clock source
                        GCLK_CLKCTRL_GEN_GCLK0 |   // Select GCLK0 at 48MHz
                        GCLK_CLKCTRL_ID_TCC0_TCC1; // Feed GCLK0 to TCC0 and TCC1
    while (GCLK->STATUS.bit.SYNCBUSY)
        ; // Wait for synchronization

    // Enable the port multiplexer for PWM pins
    PORT->Group[g_APinDescription[PIN[FUEL][CTRL]].ulPort].PINCFG[g_APinDescription[PIN[FUEL][CTRL]].ulPin].bit.PMUXEN = 1;

    PORT->Group[g_APinDescription[PIN[OX][CTRL]].ulPort].PINCFG[g_APinDescription[PIN[OX][CTRL]].ulPin].bit.PMUXEN = 1;

    // Enable the port multiplexer for Encoder coms and status register pins
    PORT->Group[g_APinDescription[PIN[FUEL][ENC_CLK]].ulPort].PINCFG[g_APinDescription[PIN[FUEL][ENC_CLK]].ulPin].bit.PMUXEN = 1;
    PORT->Group[g_APinDescription[PIN[FUEL][ENC_DATA]].ulPort].PINCFG[g_APinDescription[PIN[FUEL][ENC_DATA]].ulPin].bit.PMUXEN = 1;
    PORT->Group[g_APinDescription[REGISTER_MOSI].ulPort].PINCFG[g_APinDescription[REGISTER_MOSI].ulPin].bit.PMUXEN = 1;
    PORT->Group[g_APinDescription[REGISTER_LATCH].ulPort].PINCFG[g_APinDescription[REGISTER_LATCH].ulPin].bit.PMUXEN = 1;
    // Status register MOSI and CS technically part of the FUEL encoder SERCOM

    PORT->Group[g_APinDescription[PIN[OX][ENC_CLK]].ulPort].PINCFG[g_APinDescription[PIN[OX][ENC_CLK]].ulPin].bit.PMUXEN = 1;
    PORT->Group[g_APinDescription[PIN[OX][ENC_DATA]].ulPort].PINCFG[g_APinDescription[PIN[OX][ENC_DATA]].ulPin].bit.PMUXEN = 1;

    // Attach PWM pins to timer peripherals
    PORT->Group[g_APinDescription[PIN[FUEL][CTRL]].ulPort].PMUX[g_APinDescription[PIN[FUEL][CTRL]].ulPin >> 1].reg =
        ((g_APinDescription[PIN[FUEL][CTRL]].ulPin % 2) == 0) ? PORT_PMUX_PMUXE_E : PORT_PMUX_PMUXO_E;
    PORT->Group[g_APinDescription[PIN[OX][CTRL]].ulPort].PMUX[g_APinDescription[PIN[OX][CTRL]].ulPin >> 1].reg =
        ((g_APinDescription[PIN[OX][CTRL]].ulPin % 2) == 0) ? PORT_PMUX_PMUXE_E : PORT_PMUX_PMUXO_E;

    // Attach Encoder coms and status register pins to SERCOM peripherals
    PORT->Group[g_APinDescription[PIN[FUEL][ENC_CLK]].ulPort].PMUX[g_APinDescription[PIN[FUEL][ENC_CLK]].ulPin >> 1].reg =
        ((g_APinDescription[PIN[FUEL][ENC_CLK]].ulPin % 2) == 0) ? PORT_PMUX_PMUXE_C : PORT_PMUX_PMUXO_C;
    PORT->Group[g_APinDescription[PIN[FUEL][ENC_DATA]].ulPort].PMUX[g_APinDescription[PIN[FUEL][ENC_DATA]].ulPin >> 1].reg =
        ((g_APinDescription[PIN[FUEL][ENC_DATA]].ulPin % 2) == 0) ? PORT_PMUX_PMUXE_C : PORT_PMUX_PMUXO_C;
    PORT->Group[g_APinDescription[REGISTER_MOSI].ulPort].PMUX[g_APinDescription[REGISTER_MOSI].ulPin >> 1].reg =
        ((g_APinDescription[REGISTER_MOSI].ulPin % 2) == 0) ? PORT_PMUX_PMUXE_C : PORT_PMUX_PMUXO_C;
    PORT->Group[g_APinDescription[REGISTER_LATCH].ulPort].PMUX[g_APinDescription[REGISTER_LATCH].ulPin >> 1].reg =
        ((g_APinDescription[REGISTER_LATCH].ulPin % 2) == 0) ? PORT_PMUX_PMUXE_C : PORT_PMUX_PMUXO_C;

    PORT->Group[g_APinDescription[PIN[OX][ENC_CLK]].ulPort].PMUX[g_APinDescription[PIN[OX][ENC_CLK]].ulPin >> 1].reg =
        ((g_APinDescription[PIN[OX][ENC_CLK]].ulPin % 2) == 0) ? PORT_PMUX_PMUXE_D : PORT_PMUX_PMUXO_D;
    PORT->Group[g_APinDescription[PIN[OX][ENC_DATA]].ulPort].PMUX[g_APinDescription[PIN[OX][ENC_DATA]].ulPin >> 1].reg =
        ((g_APinDescription[PIN[OX][ENC_DATA]].ulPin % 2) == 0) ? PORT_PMUX_PMUXE_D : PORT_PMUX_PMUXO_D;

    // Configure PWM timers

    // Normal (single slope) PWM operation: timer countinuouslys count up to PER register value and then is reset to 0
    TCC0->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM; // Setup single slope PWM on TCC0
    while (TCC0->SYNCBUSY.bit.WAVE)
        ;                                    // Wait for synchronization
    TCC1->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM; // Setup single slope PWM on TCC1
    while (TCC1->SYNCBUSY.bit.WAVE)
        ; // Wait for synchronization

    TCC0->PER.reg = PWM_FREQ_COEF; // Set the frequency of the PWM on TCC0 to 38kHz: 48MHz / (1262 + 1) = 38kHz
    while (TCC0->SYNCBUSY.bit.PER)
        ;                          // Wait for synchronization
    TCC1->PER.reg = PWM_FREQ_COEF; // Set the frequency of the PWM on TCC1 to 38kHz: 48MHz / (1262 + 1) = 38kHz
    while (TCC1->SYNCBUSY.bit.PER)
        ; // Wait for synchronization

    TCC0->CC[0].reg = .75 * PWM_FREQ_COEF; // TCC0 CC0 - 50% duty cycle on D7
    while (TCC0->SYNCBUSY.bit.CC0)
        ;                                  // Wait for synchronization
    TCC1->CC[0].reg = .75 * PWM_FREQ_COEF; // TCC0 CC0 - 50% duty cycle on D6
    while (TCC1->SYNCBUSY.bit.CC0)
        ; // Wait for synchronization

    TCC0->CTRLA.bit.ENABLE = 1; // Enable the TCC1 counter
    while (TCC0->SYNCBUSY.bit.ENABLE)
        ;
    TCC1->CTRLA.bit.ENABLE = 1; // Enable the TCC1 counter
    while (TCC1->SYNCBUSY.bit.ENABLE)
        ; // Wait for synchronization

} // Use after de-safing rocket but before launch activation.

double getMicros() { return -1; } // Placeholder