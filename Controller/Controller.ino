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
#define EXT                    2u
#define CTRL                   0u
#define DIR                    1u
#define STOP                   2u
#define ENC_CLK                3u
#define ENC_DATA               4u
#define SER_OUT                5u
#define CS                     6u
#define BUTTON_ONE             0u
#define BUTTON_TWO             1u
#define SEL_SWITCH             2u
#define TX                     3u
#define RX                     4u
#define SER_EN                 3u // External serial enable sensor pin. SERCOM if low, Digital-In if high.
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
#define syncClock()                                                                                                         \
    while (GCLK->STATUS.bit.SYNCBUSY)                                                                                       \
        ;               // Wait for clock synchronization
#define TC_FUEL     TC4 // Timer pointers for each PID loop
#define TC_OX       TC5
#define SERCOM_BAUD 3u

static const signed short int C_FORWARD = 1;                  // Normalized forward vector. Swap to 0 if reversed
static const signed short int C_REVERSE = abs(C_FORWARD - 1); // Normalized reverse vector. Opposite of C_FORWARD

static const array<array<byte, 7>, 3> PIN = {
  //  Fuel, Ox, External; 255 = no pin needed for that role
    {{A2, 10, A1, 13, 12, 11, 8}, //  CTRL (PWM), DIR, STOP, ENC_CLK (SCK), ENC_DATA (MISO), SER_OUT (MOSI), CS
     {A3, 9, A0, 5, 4, 6, 7}, // Pull down, down, down, null, down/null, down/null, up/null
     {A6, A7, 2, 0, 1, 255, 255}}  //  BUTTON_ONE, BUTTON_TWO, SEL_SWITCH, TX, RX, null, null
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

    // g_APinDescription is a constant array of PinDescription structs which contain the metadata for each Arduino pin (eg.
    // DI/O pin #7, A4, whatever), defined inside variant.cpp. If you do the same on the PinDescription,
    // you can see that ulPort is the first member of the struct, of type EPortType. EPortType is an enum
    // which contains 3 values (plus a null value), which map to PortA, PortB, and PortC (which I believe in this
    // microcontroller is not used). Each of the first entries in the array are mapped to either PORTA or PORTB, which again
    // are just predefined ints from EPortType. A similar process is used for ulPin, which when combined with ulPort gives
    // the MC all the data it needs to figure out which pin is being referenced (Pin group + pin number in that group). For
    // example, the following is a breakdown of a reference to the fuel valve control pin, which outputs a PWM signal to the
    // driver board to control the power output of the valve motor. The Group and PMUX objects are arrays of integer memory
    // location offsets to access the correct portion of memory (Group[n] and PMUX[n] can be thought of to be basically
    // equivalent to returning [Group|PMUX register start location] + n*[Group|PMUX register width]). However, this requires
    // the ID (Group and pin #) of the microcontroller pin, while only having the Arduino GPIO number available (in this
    // case, PIN[FUEL][CTRL]=A2, which is in itself a macro that expands to 16u, the index used for the PinDescription
    // struct associated with the 2nd analog pin in g_APinDescription).  To assess the Group #, read the ulPort value inside
    // the PinDescription struct at index A2=16u in the g_APinDescription array. This whole process simplies like so:
    //
    // Group[g_APinDescription[PIN[FUEL][CTRL]].ulPort]
    // = Group[g_APinDescription[PIN[0][0]].ulPort]
    // = Group[g_APinDescription[A2].ulPort]
    // = Group[g_APinDescription[16u].ulPort
    // = Group[{ PORTA, 11, PIO_ANALOG,...}.ulPort]
    // = Group[PORTA]
    // =Group[0]
    //
    // Since the Group 0 control register is the first in the series, the offset turns out to be 0x0. A similar process can
    // be used to assess the pin # using PMUX (or other pin-number-dependent register) instead of Group and ulPin instead of
    // ulPort.

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
    PORT->Group[g_APinDescription[PIN[FUEL][SER_OUT]].ulPort]
        .PINCFG[g_APinDescription[PIN[FUEL][SER_OUT]].ulPin]
        .bit.PMUXEN = 1;
    PORT->Group[g_APinDescription[PIN[FUEL][CS]].ulPort].PINCFG[g_APinDescription[PIN[FUEL][CS]].ulPin].bit.PMUXEN = 1;
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
    PORT->Group[g_APinDescription[PIN[FUEL][SER_OUT]].ulPort].PMUX[g_APinDescription[PIN[FUEL][SER_OUT]].ulPin >> 1].reg |=
        ((g_APinDescription[PIN[FUEL][SER_OUT]].ulPin % 2) == 0) ? PORT_PMUX_PMUXE_C : PORT_PMUX_PMUXO_C;
    PORT->Group[g_APinDescription[PIN[FUEL][CS]].ulPort].PMUX[g_APinDescription[PIN[FUEL][CS]].ulPin >> 1].reg |=
        ((g_APinDescription[PIN[FUEL][CS]].ulPin % 2) == 0) ? PORT_PMUX_PMUXE_C : PORT_PMUX_PMUXO_C;

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
    // THE PWM pins on the Arduino were chosen such that both pins connect to the same TCC (in this case TCC0) on separate
    // waveform outputs, which means we only have to spin up and configure a single peripheral, and just change the
    // corresponding CCBx value to change the outputs. Note that every TCC does not neccessarily share the same configuration
    // options or parameters. Should it be neccesary to change pins and spin up a different TCC, while it may be possible to
    // just copy-paste the original config routine, that should not be assumed.
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
    // identically, as we are using them to run the FUEL and OX PID controllers respectively.

    //
    TC4->COUNT16.CTRLA.reg =
        TC_CTRLA_MODE_COUNT16 |   // Select 16 bit mode for TC4
        TC_CTRLA_PRESCALER_DIV8 | // Divides frequency by 8, to get f_TC of 1MHz, or period 1us
        TC_CTRLA_PRESCSYNC_PRESC; // Clock reset is connected to prescaler clock not GCLK. Keeps 1us period.

    while (TC4->COUNT16.STATUS.bit.SYNCBUSY)
        ; // sync

    // Sets continuous read synchronization of count register. This means it is not neccesary to set the RREQ flag and
    // wait to synchronize every time we want to read the COUNT value.See section 30.10.2 in MC datasheet.
    TC4->COUNT16.READREQ.reg = TC_READREQ_RCONT | // Sets RCONT flag, disabling automatic clearing of RREQ flag after sync
                               TC_READREQ_RREQ |  // Read sync request flag, synchronizes COUNT register for reading
                               TC_READREQ_ADDR(0x10); // COUNT register address

    // Same as TC4
    TC5->COUNT16.CTRLA.reg =
        TC_CTRLA_MODE_COUNT16 |   // Select 16-bit mode for TC5.
        TC_CTRLA_PRESCALER_DIV8 | // Divides GCLK frequency by 8, to get f_TC of 8Mhz/8=1MHz, or period 1us
        TC_CTRLA_PRESCSYNC_PRESC; // Clock reset is connected to prescaler clock not GCLK. Keeps 1us period.
    while (TC5->COUNT16.STATUS.bit.SYNCBUSY)
        ; // sync
    // Same as TC4
    TC5->COUNT16.READREQ.reg = TC_READREQ_RCONT | // Sets RCONT flag, disabling automatic clearing of RREQ flag after sync
                               TC_READREQ_RREQ |  // Read sync request flag, synchronizes COUNT register for reading
                               TC_READREQ_ADDR(0x10); // COUNT register address

    // The MC has 6 SERCOM, or SERial COMmunication, peripherals, SERCOM0:SERCOM5. Each SERCOM peripheral has 4 pads
    // associated with it. "Pad" just means an IO line for the peripheral, similar to channels in TCCs. MC pins are
    // associated with both a SERCOM and a pad. Some MC pins have a primary and alternate set. This is why the FUEL pins are
    // attached to peripheral E, while OX is attached to peripheral D. Both are attached to SERCOMs, but system requirements
    // meant that we had to use the alternate set for FUEL (note that the FUEL pins do not actually have a primary set, as
    // seen in the variant table). The function of each pad is defined by the chosen SERCOM operation protocol and
    // configuration. SERCOMs 1:5 are already reserved by the Arduino (this can be seen in variant.h), which leaves us with
    // only SERCOM0 to use as we want (without overriding an existing serial port connection). Fortunately, SERCOM1 has been
    // reserved as the stock SPI port, which means we actually have enough to do what we need.

    // SPI, or Serial Peripheral Interface, is a communications protocol developed for low level communications between
    // things like sensors, ICs, encoders, or other peripherals. It is what we are using on SERCOM0 and SERCOM 1. Our
    // encoders use a subset of SPI functionality to output their readings (they only output data, not take it in). The SPI
    // protocol is based around the shift register digital circuit, which Ben Eater has several fantastic YouTube videos on.
    // This means that we can also use it directly with any compatible shift register IC, even without a peripheral
    // abstraction. SAMD SERCOM's SPI mode supports full-duplex communications, which means it can read and write at the same
    // time. What this means for our application is we can control up to 4 single-direction devices with just two SERCOM
    // interfaces (2 inputs and 2 outputs). There are many online resources that do a better job of explaining SPI, but the
    // basics are as follows: full-duplex SPI has 4 pins: MISO, MOSI, SCK, and CS. MOSI and MISO are data pins, standing for
    // Master-Out-Slave-In and Master-In-Slave-Out. Fairly self-explanatory, in our case as the host MISO is the input and
    // MOSI is the output pin for the SERCOM. SCK is the Serial ClocK. SPI does not have a native baudrate, instead data is
    // only sent through MOSI and MISO on the leading or trailing edge of SCK's clock pulse (depending on configuration).
    // This type of shared-clock communication is described as Synchronous. This is in contrast to Asynchronous communication
    // protocols like UART (which SERCOM also supports), in which the devices decide on the communication frequency during
    // the initial handshake and then send data based on that with no regard for the other device's timing. The CS/Chip
    // Select, or SS/SPI Select (it has been called different things through different version of the protocol) line is for,
    // as the name suggests, selecting which chip the data is going to/coming from along MOSI and MISO. Notably, MISO, MOSI,
    // and SCK all can be connected to peripherals in parallel. There is one CS line ran from the host to each peripheral.
    // This means that SPI can scale to many devices relatively well, as you only need to allocate 1 extra pin per added
    // device. Pulling CS low unlatches the peripheral's shift register, allowing for data transfer to that device. We will
    // use this fact along with the full-duplex support to allow us to interact with multiple devices on the same SPI port,
    // in this case both the FUEL encoder and the LED status shift register, as was mentioned briefly above. Unfortunately
    // the encoders do not have a CS input, and so it is not possible to place them on the same SPI port without some circuit
    // modifications on our end. Though this is not impossible (all it would take is a discrete AND gate IC), it was decided
    // to maintain separate SPI ports in case we decide to explore parallelization in the future. The configuration for the
    // SPI SERCOMs is fairly self explanatory, as we are using many default configuration parameters. When using SPI with a
    // shift register (in this case the SN74HCT595 acting as our LED parallel display), connections should be made as
    // follows: MOSI->Serial input (SER), SCK->Serial clock (SRCLK), and CS->Storage register clock/latch pin (RCLK). See
    // Chapter 27 in the MC datasheet, the datasheets for the NME2 (encoder) and SN74HC595 (LED shift register), and the
    // circuit schematic for more info.

    // The other notable configuration step is the baudrate, or data transfer speed. The encoders support up to 10MHz (the
    // shift register much more than that), but we do not necessarily want to approach that as our system needs to be
    // reliable and tolerant of suboptimal EMI conditions. 1MHz was chosen due to it's convenient bitrate of 1 bit/us,
    // allowing for easy timing calculations. Depending on the operating mode of the SERCOM, the BAUD register
    // influences baudrate differently, but for synchronous operations (as is the case for SPI), the baud rate is defined as
    // f_ref/(2*(BAUD+1)), where f_ref is the reference frequency fed to the SERCOM by the GCLK. See section 25.6.2.3 in
    // MC datasheet for more info.

    // Note that SERCOM0 does not require a chip select pin, as it is only reading in data from the OX encoder.
    SERCOM0->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_DIPO(0x3u) |     // MISO is Pad 3
                             SERCOM_SPI_CTRLA_DOPO(0x0u) |     // MOSI pad 0, SCK Pad 1, CS Pad 2
                             SERCOM_SPI_CTRLA_MODE_SPI_MASTER; // Sets device mode as host
    SERCOM0->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_RXEN;            // Enable reciever/full-duplex operation

    SERCOM0->SPI.BAUD.reg = SERCOM_SPI_BAUD_BAUD(SERCOM_BAUD); // Sets baudrate to 8MHz/(2*([BAUD=3]+1)=1MHz

    // Same as SERCOM0. Most of this should already be configured by Arduino by default, but it should still be defined
    // explicitly.
    SERCOM1->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_DIPO(0x3u) |     // MISO is Pad 3
                             SERCOM_SPI_CTRLA_DOPO(0x0u) |     // MOSI pad 0, SCK Pad 1, CS Pad 2
                             SERCOM_SPI_CTRLA_MODE_SPI_MASTER; // Sets device mode as host
    SERCOM1->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_RXEN;            // Enable reciever/full-duplex operation
    SERCOM1->SPI.BAUD.reg = SERCOM_SPI_BAUD_BAUD(SERCOM_BAUD); // Sets baudrate to 8MHz/(2*([BAUD=3]+1)=1MHz
    // TODO: Serial2 setup, Configure GPIO lines, pull up/down resistors - Pull SER_EN high
}

// TODO: Document GCLK,
void configureClocks() {

    // No TCC3?
    PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0 | PM_APBCMASK_SERCOM1 | PM_APBCMASK_SERCOM2 | PM_APBCMASK_SERCOM3 |
                        PM_APBCMASK_SERCOM4 | PM_APBCMASK_SERCOM5 | PM_APBCMASK_TCC0 | PM_APBCMASK_TCC1 | PM_APBCMASK_TCC2 |
                        PM_APBCMASK_TC3 | PM_APBCMASK_TC4 | PM_APBCMASK_TC5 | PM_APBCMASK_TC6 | PM_APBCMASK_TC7;

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

    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_GEN_GCLK5 | // Select GCLK5 at 8MHz to edit |
                        GCLK_CLKCTRL_CLKEN |     // Enable GCLK5 as a clock source
                        GCLK_CLKCTRL_ID_TC4_TC5; // Feed GCLK5 to TC4 and TC5
    syncClock();

    // Link timer periphs to SERCOM0
    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(6u) |   // Edit GCLK6
                        GCLK_GENCTRL_IDC |      // Improve 50/50 PWM
                        GCLK_GENCTRL_GENEN |    // Enable generic clock
                        GCLK_GENCTRL_SRC_OSC8M; // Link to 8MHz source
    syncClock();

    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_GEN_GCLK6 |      // Select GCLK6 at 8MHz to edit |
                        GCLK_CLKCTRL_CLKEN |          // Enable GCLK6 as a clock source
                        GCLK_CLKCTRL_ID_SERCOM0_CORE; // Feed GCLK6 to SERCOM0 Baud Generator
    syncClock();

    // Link timer periph connected to SERCOM1
    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(7u) |   // Edit GCLK7
                        GCLK_GENCTRL_IDC |      // Improve 50/50 PWM
                        GCLK_GENCTRL_GENEN |    // Enable generic clock
                        GCLK_GENCTRL_SRC_OSC8M; // Link to 8MHz source
    syncClock();
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_GEN_GCLK7 |      // Select GCLK7 at 8MHz to edit |
                        GCLK_CLKCTRL_CLKEN |          // Enable GCLK7 as a clock source
                        GCLK_CLKCTRL_ID_SERCOM1_CORE; // Feed GCLK7 to SERCOM1 Baud Generator
    syncClock();
}