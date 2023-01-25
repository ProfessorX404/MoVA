/*
Xavier Beech
UW SARP 2022-23
Motorized valve actuation controller software for 2022 Pacific Impulse.
Must comment line 22 of [ArduinoSTL location]/ArduinoSTL/src/new_handler.cpp, or code will not compile.when using v1.8.3
or later

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

#define FUEL                   0u // Array index constants for easy access
#define OX                     1u
#define CTRL                   0u
#define DIR                    1u
#define ENC_CLK                2u
#define ENC_DATA               3u
#define ADJ_SELECT             4u
#define P                      0u
#define I                      1u
#define D                      2u
#define ENC_TOT_BIT_CT         24u      // Total number of bits in encoder packet. Last bit is error bit, success=1
#define ENC_DATA_BIT_CT        17u      // Data bits in encoder packet
#define ENC_MIN_TIME_US        20u      // Minimum amount of time between data calls, in milliseconds
#define WIND_UP_MIN            -10.0    // Integral growth bound min const
#define WIND_UP_MAX            10.0     // Integral growth bound max const
#define ENC_TICS_PER_MOTOR_REV 0x20000u // Number of encoder tics in mechanical revolution (per datasheet)
#define GEARBOX_RATIO          15u      // Revs into gearbox per 1 revolution out
#define ENC_TICS_PER_VALVE_REV (ENC_TICS_PER_MOTOR_REV) * (GEARBOX_RATIO) // Post-gearbox encoder tics per valve revolution
#define VALVE_OPEN_DEG         90.0                                       // Encoder value for valve being fully open
#define VALVE_CLOSED_DEG       0.0                                        // Encoder value for valve being fully closed
#define ENC_TICS_PER_VALVE_DEG (int)(ENC_TICS_PER_VALVE_REV / 360)        // Post-gearbox encoder tics / degree
#define TARGET_REVS            (int)((VALVE_OPEN_DEG / 360) * GEARBOX_RATIO) // Number of rotations to get almost fully open
#define PWM_FREQ_COEF          480u                                          // 48MHz / (2 + 480) = .05MHz = 50KHz
#define REGISTER_MOSI          11u
#define REGISTER_LATCH         8u
#define syncClock()                                                                                                         \
    while (GCLK->STATUS.bit.SYNCBUSY)                                                                                       \
        ;               // Wait for clock synchronization
#define TC_FUEL     TC4 // Timers for each PID loop
#define TC_OX       TC5
#define SERCOM_BAUD 0

static const signed short int C_FORWARD = 1;                  // Normalized forward vector. Swap to 0 if reversed
static const signed short int C_REVERSE = abs(C_FORWARD - 1); // Normalized reverse vector. Opposite of C_FORWARD

static const array<array<byte, 10>, 2> PIN = {
  //  Fuel, Ox
    {{A2, x, 13, 12, x}, //  CTRL (PWM), DIR, ENC_CLK (SCK), ENC_DATA (MISO), ADJ_SELECT
     {A3, x, 5, 6, x}}
};

array<array<double, 3>, 2> k_pid = {
    {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}
}; // pid[fo] constants, in format [kP, kI, kD]

array<uint32_t, 2> accumulator = {0, 0}; // PID integral term accumulator
array<uint16_t, 2> prev_pos = {0, 0};    // PID theta_n-1

uint16_t target = VALVE_CLOSED_DEG * ENC_TICS_PER_VALVE_DEG; // Current valve position target. Init'ed to closed
byte errorCode = 0;                                          // Global error code variable for fault tracking.
array<byte, 2> totalRevs = {0, 0};                           // Revolution counter

// Status flags
bool f_activated = false;                       // True if valve has been activated
array<bool, 2> f_withinOneRev = {false, false}; // True if totalRevs has passed TARGET_REVS

void setup() {
    configureClocks();
    Serial.begin(115200); // Init serial connection
    byte fo = FUEL;
}

void loop() {
    if (!f_activated) { // Wait for system to be activated
        while (!isActivated()) {};
        f_activated = true;
    }
    // start TC4 and TC5 before first PID loop
}

// Takes input from global var, writes to pins accordingly
void update(byte fo) {
    uint16_t delta_t = getDeltaT(fo);
    uint16_t theta_n = readEncData(fo);
    accumulator[fo] += (delta_t << 1) * (theta_n + prev_pos[fo] - (target >> 1));

    double O = k_pid[fo][P] * (theta_n - target) + k_pid[fo][I] * accumulator[fo] +
               k_pid[fo][D] * ((theta_n - prev_pos[fo]) / delta_t);
    O = (O > PWM_FREQ_COEF) ? PWM_FREQ_COEF : O;

    if (O < 0 && C_FORWARD) {

    } else {
    }

    TCC0->CCB[!FUEL].reg = O;
    while (fo ? TCC0->SYNCBUSY.bit.CCB1 : TCC0->SYNCBUSY.bit.CCB1)
        ;
}

uint16_t getDeltaT(byte fo) {
    Tc *TC_x = (fo == FUEL ? TC_FUEL : TC_OX);
    TC4->COUNT16.READREQ.bit.RREQ = 0x1u;
    while (TC_x->COUNT16.STATUS.bit.SYNCBUSY)
        ;
    uint16_t dt = TC_x->COUNT16.COUNT.reg;
    TC_x->COUNT16.CTRLBSET.reg = TC_CTRLBSET_CMD_RETRIGGER;
    while (TC_x->COUNT16.STATUS.bit.SYNCBUSY)
        ;
    return dt;
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

// Configures internal peripherals and attaches to physical Arduino pins.
// A peripheral is an internal microcontroller node that has functions
// independent of the main controller thread. For example, TCs (Timer/Counters)
// maintain a clock rate, but do not have to be toggled on and off (eg. by interrupts)
// once they have been started. SERCOM (or SERial COMmunication ports) allow for communication
// through various serial interfaces to external devices, but once they have been given data
// to transmit or receive they handle all pin modulation, clock timing, etc. independently.
// All one has to do is configure the SERCOM (or whatever peripheral)
// and read/write the associated data register and it handles everything else.
// Different pins have different possible peripheral connections,
// and which pins connect where can be found in Nano_33iot_variant.xlsx,
// which was compiled from the variant.h and variant.cpp files found
// in the Arduino Core, which is in turn created using the specific Arduino pin
// schematic and the SAMD21 datasheet, Table 7.1. You will see some peripherals
// throughout the program require read- and write-synchronization. This is because they run
// at different speeds to the CPU, and so their registers will not necessarily be fully updated
// by the time the CPU has completed the operation and is ready to move on. This can cause undocumented
// behavior if the CPU then turns around and tries to re-access the data for whatever reason.
void attachPins() {

    // PORT is the name of the multiplexer controller. It is what controls which microcontroller (MC) pins
    // connect to which internal peripherals. Though it is not in itself a peripheral, it still requires
    // configuration. The port has several PORT Groups, and the ones we care about are Groups 0 and 1,
    // which map to the PA pins (first 32 pins) and PB pins (second group of 32), respectively.
    // Each group has it's own 0x80 bit register, which contains all the IO configuration,
    // pin output set and input read controls, peripheral multiplexing for its associated pins.
    // The SAMD-defined PA/PB/etc pins should not be confused with the numbered GPIO pins on the Arduino.

    // Also included in the variant tables mentioned previously is the list of which Arduino pins connect
    // to which MC pins. For example, the Arduinos digital GPIO pin 7 (D7) connects to pin PA06
    // on the MC. To reference, eg, the PMUXEN bit for D7 we would use:PORT->Group[0].PINCFG[6].bit.PMUXEN
    // Piece by piece, this first takes the PORT macro, which is a pointer to PORT's location in memory,
    // applies on offset based on which Group you are looking for (0x0, as it is a PA pin), then an additional offset
    // to the PINCFG region of the register, which contains a register for each IO line (0x40+n*0x01, n=[0:31]).
    // It then applies a final offset to the bit in question (0x0, for PMUXEN), and in this case writes the bit located
    // at that address. This changes the pin function from GPIO to being controller by the connected peripheral.
    // More details can be found in Chapter 23 of the SAMD datasheet, specifically section 23.8.13, the register
    // description for pin configuration.

    // Enable the port multiplexer for PWM pins
    PORT->Group[g_APinDescription[PIN[FUEL][CTRL]].ulPort].PINCFG[g_APinDescription[PIN[FUEL][CTRL]].ulPin].bit.PMUXEN = 1;
    PORT->Group[g_APinDescription[PIN[OX][CTRL]].ulPort].PINCFG[g_APinDescription[PIN[OX][CTRL]].ulPin].bit.PMUXEN = 1;

    // Enable the port multiplexer for Encoder coms and status register pins
    PORT->Group[g_APinDescription[PIN[FUEL][ENC_CLK]].ulPort]
        .PINCFG[g_APinDescription[PIN[FUEL][ENC_CLK]].ulPin]
        .bit.PMUXEN = 1;
    PORT->Group[g_APinDescription[PIN[FUEL][ENC_DATA]].ulPort]
        .PINCFG[g_APinDescription[PIN[FUEL][ENC_DATA]].ulPin]
        .bit.PMUXEN = 1;
    PORT->Group[g_APinDescription[REGISTER_MOSI].ulPort].PINCFG[g_APinDescription[REGISTER_MOSI].ulPin].bit.PMUXEN = 1;
    PORT->Group[g_APinDescription[REGISTER_LATCH].ulPort].PINCFG[g_APinDescription[REGISTER_LATCH].ulPin].bit.PMUXEN = 1;
    // Status LED shift register MOSI and CS technically part of the FUEL encoder SERCOM. As it is not neccesary to
    // write data to the encoders, and we are out of SERCOMs to communicate with the register, it has been connected to
    // the output pins (MOSI and CS, as well as SCK in parallel) for SERCOM1, which is associated with the FUEL valve
    // encoder. This means that the shift register can be set during any read of the encoder. Normally this would be
    // suboptimal as it would constrain the writing of the shift register only to times the encoder is able to be read
    // from, but seeing as the LEDs are only a human information interface, the timing is very non-critical and as long
    // as we maintain the >20us turnaround time between encoder reads, it will not present any issues.

    // SERCOM0 SCK and MISO pins for OX
    PORT->Group[g_APinDescription[PIN[OX][ENC_CLK]].ulPort].PINCFG[g_APinDescription[PIN[OX][ENC_CLK]].ulPin].bit.PMUXEN = 1;
    PORT->Group[g_APinDescription[PIN[OX][ENC_DATA]].ulPort].PINCFG[g_APinDescription[PIN[OX][ENC_DATA]].ulPin].bit.PMUXEN =
        1;

    // Now that we are able to connect the pins to peripherals, now we actually need to tell the MC which peripherals to
    // attach them to. As seen in the variant tables, each pin has as many as 8 peripherals to choose from (3bits),
    // labelled A-H. In each Group, there are actually two registers controlling connections. One for the odd numbered
    // pins (PA/PB01, 03, 05, etc.) and one for the even numbered pins. It is unclear why they choose to do it this way,
    // but what that means is it just adds an extra step when configuring this step. This time, instead of accessing the
    // PINCFG register, we are accessing the PMUX register for the pin. There are 16 of these registers (1 for
    // PA01+PA02, next for PA03+PA04, etc), each 1 byte long. The first 4-bit nibble in the register holds the ID for
    // the peripheral of the even pin, and the 2nd nibble holds the ID for the odd pin. What this means is, when
    // accessing registers we have 2 new considerations: first, the nth IO line actually correponds to the (n >> 1)th
    // PMUX register. Secondly, the even and odd masks are different even when selecting the same peripheral ID, since
    // we are writing to a single register. The even mask is going to be shifted up 4 bits, as the first four are the
    // location of the ID for the odd pin. We also need to be careful not the overwrite any information that has already
    // been written to the least-significant ID by writing to the register. Instead, we must OR the register with the
    // mask to maintain the LS value. This is good practice when writing to any register that contains multiple parameters.
    // If we wanted to replace an ID (without knowing the initial state or wanting to clear the other ID), we would have to
    // do: x.reg &= [15 (00001111) for replacing first four | 240 (11110000) for replacing last 4]; x.reg |= [bitmask]; The
    // ID mask names are formatted as follows: PORT_PMUX_PMUX[O (odd)| E (even)]_[A:H (peripheral ID)]. More info can be
    // found in 23.8.12

    // Attach PWM pins to timer peripherals
    PORT->Group[g_APinDescription[PIN[FUEL][CTRL]].ulPort].PMUX[g_APinDescription[PIN[FUEL][CTRL]].ulPin >> 1].reg |=
        ((g_APinDescription[PIN[FUEL][CTRL]].ulPin % 2) == 0) ? PORT_PMUX_PMUXE_E : PORT_PMUX_PMUXO_E;
    PORT->Group[g_APinDescription[PIN[OX][CTRL]].ulPort].PMUX[g_APinDescription[PIN[OX][CTRL]].ulPin >> 1].reg |=
        ((g_APinDescription[PIN[OX][CTRL]].ulPin % 2) == 0) ? PORT_PMUX_PMUXE_E : PORT_PMUX_PMUXO_E;

    // Attach Encoder coms and status register pins to SERCOM peripherals
    PORT->Group[g_APinDescription[PIN[FUEL][ENC_CLK]].ulPort].PMUX[g_APinDescription[PIN[FUEL][ENC_CLK]].ulPin >> 1].reg |=
        ((g_APinDescription[PIN[FUEL][ENC_CLK]].ulPin % 2) == 0) ? PORT_PMUX_PMUXE_C : PORT_PMUX_PMUXO_C;
    PORT->Group[g_APinDescription[PIN[FUEL][ENC_DATA]].ulPort].PMUX[g_APinDescription[PIN[FUEL][ENC_DATA]].ulPin >> 1].reg |=
        ((g_APinDescription[PIN[FUEL][ENC_DATA]].ulPin % 2) == 0) ? PORT_PMUX_PMUXE_C : PORT_PMUX_PMUXO_C;
    PORT->Group[g_APinDescription[REGISTER_MOSI].ulPort].PMUX[g_APinDescription[REGISTER_MOSI].ulPin >> 1].reg |=
        ((g_APinDescription[REGISTER_MOSI].ulPin % 2) == 0) ? PORT_PMUX_PMUXE_C : PORT_PMUX_PMUXO_C;
    PORT->Group[g_APinDescription[REGISTER_LATCH].ulPort].PMUX[g_APinDescription[REGISTER_LATCH].ulPin >> 1].reg |=
        ((g_APinDescription[REGISTER_LATCH].ulPin % 2) == 0) ? PORT_PMUX_PMUXE_C : PORT_PMUX_PMUXO_C;

    PORT->Group[g_APinDescription[PIN[OX][ENC_CLK]].ulPort].PMUX[g_APinDescription[PIN[OX][ENC_CLK]].ulPin >> 1].reg |=
        ((g_APinDescription[PIN[OX][ENC_CLK]].ulPin % 2) == 0) ? PORT_PMUX_PMUXE_D : PORT_PMUX_PMUXO_D;
    PORT->Group[g_APinDescription[PIN[OX][ENC_DATA]].ulPort].PMUX[g_APinDescription[PIN[OX][ENC_DATA]].ulPin >> 1].reg |=
        ((g_APinDescription[PIN[OX][ENC_DATA]].ulPin % 2) == 0) ? PORT_PMUX_PMUXE_D : PORT_PMUX_PMUXO_D;

    // See configureClocks() for information regarding GCLK configuration and linking to periphs.

    // TCCs, or Timer/Counters for Control applications, are a Timer/Counter peripheral with added logical functionality
    // in order to function as controllers for other peripherals or external devices. Their main advantage over standard
    // TCs, at least for our application, is their greatly improved waveform generation and double-buffered setup which
    // allows for more nuanced PWM control. This is not currently neccessary for our application, but it very well could
    // be in future iterations. There are 4 TCC instances, TCC0 through TCC3, each with slightly different specs. Each TCC is
    // linked to a GCLK through a prescaler.  In our case with waveform generation, this event is toggling the waveform
    // output to create a PWM wave. Each TCC has up to four Compare/Capture channels, CC(B)[0:3]. CCBx is double buffered
    // through CCx, which means that writing to CCBx takes several clock cycles longer to take effect but is synchronized as
    // to prevent glitches during waveform period transitions.. Each CC channel is linked to 2 waveform generators
    // WO[0:8].This fact is not made very clear in the datasheet, so I provided the default connection configuration in the
    // program header. More information about waveform generation, PWM frequency vs resolution, and pulse width can be found
    // in Section 31.6 of SAMD datasheet.

    // Configure TCC0
    // Setup basic dual slope PWM on TCC0 (See 31.6.2.5.6 in SAMD datasheet). Note that we are writing, not ORing, the
    // registers as we need them to only contain the value we give it and do not care about any previous settings.
    TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;
    while (TCC0->SYNCBUSY.bit.WAVE)
        ; // Sync

    TCC0->PER.reg = PWM_FREQ_COEF; // Set PWM frequency to 50KHz (f=GCLK/(2+PER))
    while (TCC0->SYNCBUSY.bit.PER)
        ; // Sync

    TCC0->CTRLA.bit.ENABLE = 1u; // Enable the TCC0 counter
    while (TCC0->SYNCBUSY.bit.ENABLE)
        ; // Sync

    // Much of the TCC information applies with TCs as well. The biggest difference is that TCs lack a double buffered input,
    // and instead only have CCx. This is fine as we are only using TC4 and TC5 as timers for the PID controllers, and do not
    // need any waveform generation from them. The counter increments from 0 or decrements from the max value depending on
    // configuration, and COUNT register is compared to the value in the Compare/Capture register (CCx). Every time the count
    // hits CCx, it resets to 0/max and triggers an event. See Chapter 30 for more details. TC4 and TC5 are configured
    // identically, as we are using them to run the FUEL and OX PID controllers.
    TC4->COUNT16.CTRLA.reg =
        TC_CTRLA_MODE_COUNT16 |   // Select 16 bit mode for TC4
        TC_CTRLA_PRESCALER_DIV8 | // Divides frequency by 8, to get f_TC of 1MHz, or period 1us
        TC_CTRLA_PRESCSYNC_PRESC; // Clock reset is connected to prescaler clock not GCLK. Keeps 1us period.

    while (TC4->COUNT16.STATUS.bit.SYNCBUSY)
        ; // sync
    TC4->COUNT16.READREQ.reg =
        TC_READREQ_RCONT | TC_READREQ_RREQ | TC_READREQ_ADDR(0x10); // Sets continuous read synchronization of count register
    while (TC4->COUNT16.STATUS.bit.SYNCBUSY)
        ; // sync

    TC5->COUNT16.CTRLA.reg =
        TC_CTRLA_MODE_COUNT16 |   // Select 16 bit mode for TC5
        TC_CTRLA_PRESCALER_DIV8 | // Divides frequency by 8, to get f_TC of 1MHz, or period 1us
        TC_CTRLA_PRESCSYNC_PRESC; // Clock reset is connected to prescaler clock not GCLK. Keeps 1us period.

    while (TC5->COUNT16.STATUS.bit.SYNCBUSY)
        ; // sync
    TC5->COUNT16.READREQ.reg =
        TC_READREQ_RCONT | TC_READREQ_RREQ | TC_READREQ_ADDR(0x10); // Sets continuous read synchronization of count register
    while (TC5->COUNT16.STATUS.bit.SYNCBUSY)
        ; // sync

    // TODO: Document SERCOM configuration
    SERCOM0->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_DIPO(0x3) |      // MISO is Pad 3
                             SERCOM_SPI_CTRLA_DOPO(0x0) |      // MOSI pad 0, SCK Pad 1, CS Pad 2
                             SERCOM_SPI_CTRLA_MODE_SPI_MASTER; // Sets device as host
    SERCOM0->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_RXEN |           // Enable reciever
                             SERCOM_SPI_CTRLB_MSSEN;           // Enable hardware chip select
    SERCOM0->SPI.BAUD.reg = SERCOM_SPI_BAUD_BAUD(SERCOM_BAUD);

    SERCOM1->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_DIPO(0x3) |      // MISO is Pad 3
                             SERCOM_SPI_CTRLA_DOPO(0x0) |      // MOSI pad 0, SCK Pad 1, CS Pad 2
                             SERCOM_SPI_CTRLA_MODE_SPI_MASTER; // Sets device as host
    SERCOM1->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_RXEN |           // Enable reciever
                             SERCOM_SPI_CTRLB_MSSEN;           // Enable hardware chip select
    SERCOM1->SPI.BAUD.reg = SERCOM_SPI_BAUD_BAUD(SERCOM_BAUD);
}

void configureClocks() {

    // Link timer periphs to GCLK for PWM
    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(4u) |     // Edit GCLK4
                        GCLK_GENCTRL_IDC |        // Improve 50/50 PWM
                        GCLK_GENCTRL_GENEN |      // Enable generic clock
                        GCLK_GENCTRL_SRC_DFLL48M; // Link to 48MHz source
    syncClock();

    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_GEN_GCLK4 |   // Select GCLK4 at 48MHz to edit |
                        GCLK_CLKCTRL_CLKEN |       // Enable GCLK4 as a clock source
                        GCLK_CLKCTRL_ID_TCC0_TCC1; // Feed GCLK4 to TCC0 and TCC1
    syncClock();

    // Link timer periphs to getDeltaT
    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(5u) |   // Edit GCLK5
                        GCLK_GENCTRL_IDC |      // Improve 50/50 PWM
                        GCLK_GENCTRL_GENEN |    // Enable generic clock
                        GCLK_GENCTRL_SRC_OSC8M; // Link to 8MHz source
    syncClock();

    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_GEN_GCLK5 | // Select GCLK5 at 48MHz to edit |
                        GCLK_CLKCTRL_CLKEN |     // Enable GCLK5 as a clock source
                        GCLK_CLKCTRL_ID_TC4_TC5; // Feed GCLK4 to TC4 and TC5
    syncClock();

    // Link timer periphs to SERCOM0
    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(6u) |   // Edit GCLK6
                        GCLK_GENCTRL_IDC |      // Improve 50/50 PWM
                        GCLK_GENCTRL_GENEN |    // Enable generic clock
                        GCLK_GENCTRL_SRC_OSC8M; // Link to 8MHz source
    syncClock();

    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_GEN_GCLK6 |      // Select GCLK6 at 48MHz to edit |
                        GCLK_CLKCTRL_CLKEN |          // Enable GCLK6 as a clock source
                        GCLK_CLKCTRL_ID_SERCOM0_CORE; // Feed GCLK4 to SERCOM0 Baud Generator
    syncClock();

    // Link timer periph to SERCOM1
    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(7u) |   // Edit GCLK6
                        GCLK_GENCTRL_IDC |      // Improve 50/50 PWM
                        GCLK_GENCTRL_GENEN |    // Enable generic clock
                        GCLK_GENCTRL_SRC_OSC8M; // Link to 8MHz source
    syncClock();

    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_GEN_GCLK7 |      // Select GCLK6 at 48MHz to edit |
                        GCLK_CLKCTRL_CLKEN |          // Enable GCLK6 as a clock source
                        GCLK_CLKCTRL_ID_SERCOM1_CORE; // Feed GCLK4 to SERCOM0 Baud Generator
    syncClock();
}