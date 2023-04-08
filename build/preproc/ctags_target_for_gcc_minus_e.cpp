# 1 "Z:\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
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

 - ArduinoSTL https://github.com/mike-matera/ArduinoSTL



OTMX Values:

CCx | WO[n]

CC0 | WO[0], WO[4]

CC1 | WO[1], WO[5]

CC2 | WO[2], WO[6]

CC3 | WO[3], WO[7]

*/
# 23 "Z:\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
# 24 "Z:\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino" 2
# 46 "Z:\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
// Error codes, 2-15





// Status codes, 0-7
# 77 "Z:\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
const array<char *, 8> STATUS_NAME = {"STATUS_INIT", "STATUS_CONN", "STATUS_HOME", "STATUS_WAIT",
                                      "STATUS_IDLE", "STATUS_ACTIVATE", "STATUS_RUN", "STATUS_ABORT"};

typedef struct {
    uint8_t port;
    uint8_t pin;
} PIN;

typedef struct {
    uint32_t origin; // Encoder readings when valve completely closed.
    uint32_t prev_t; // t_n-1, in ticks
    float target; // PID setpoint, in revs
    float totalRevs; // Current position in revs
    float accumulator; // PID integral term accumulator
    byte err; // Error code
    byte status; // Status code
    float P;
    float I;
    float D;
    Tc *TC;
    byte CCB;
    Sercom *SERCOM;
    PIN CTRL;
    PIN DIR_SEL;
    PIN STOP;
    PIN ENC_CLK;
    PIN ENC_DATA;
    PIN SER_OUT;
    PIN CS;
} Encoder;

struct {
    PIN BUTTON_ONE = {(uint8_t)g_APinDescription[A6].ulPort, (uint8_t)g_APinDescription[A6].ulPin};
    PIN BUTTON_TWO = {(uint8_t)g_APinDescription[A7].ulPort, (uint8_t)g_APinDescription[A7].ulPin};
    PIN SEL_SWITCH = {(uint8_t)g_APinDescription[2].ulPort, (uint8_t)g_APinDescription[2].ulPin};
    PIN TX = {(uint8_t)g_APinDescription[0].ulPort, (uint8_t)g_APinDescription[0].ulPin};
    PIN RX = {(uint8_t)g_APinDescription[1].ulPort, (uint8_t)g_APinDescription[1].ulPin};
} EXT;

Encoder Fuel, Ox;
Encoder *enc;

bool serEn = false;

PIN SER_EN = {1, 11}; // External Serial1 enable sensor pin. Active low.

void setup() {
    SerialUSB.begin(115200);
    Serial1.begin(115200);
    delay(100);
    while (!Serial1.available())
        ;
    SerialUSB.println("USB");
    sPrintln("UART");
    Fuel = {0, 0, 0.0, 0.0, 0.0, 1u /* No error present*/, 0u, 0.0, 0.0, 0.0, ((Tc *)0x42003000UL) /**< \brief (TC4) APB Base Address */, 1u, ((Sercom *)0x42000800UL) /**< \brief (SERCOM0) APB Base Address */};
    Ox = {0, 0, 0.0, 0.0, 0.0, 1u /* No error present*/, 0u, 0.0, 0.0, 0.0, ((Tc *)0x42003400UL) /**< \brief (TC5) APB Base Address */, 0, ((Sercom *)0x42000C00UL) /**< \brief (SERCOM1) APB Base Address */};

    Fuel.CTRL = {(uint8_t)g_APinDescription[A2].ulPort, (uint8_t)g_APinDescription[A2].ulPin};
    Fuel.DIR_SEL = {(uint8_t)g_APinDescription[10].ulPort, (uint8_t)g_APinDescription[10].ulPin};
    Fuel.STOP = {(uint8_t)g_APinDescription[A1].ulPort, (uint8_t)g_APinDescription[A1].ulPin};
    Fuel.ENC_CLK = {(uint8_t)g_APinDescription[13].ulPort, (uint8_t)g_APinDescription[13].ulPin};
    Fuel.ENC_DATA = {(uint8_t)g_APinDescription[12].ulPort, (uint8_t)g_APinDescription[12].ulPin};
    Fuel.SER_OUT = {(uint8_t)g_APinDescription[11].ulPort, (uint8_t)g_APinDescription[11].ulPin};
    Fuel.CS = {(uint8_t)g_APinDescription[8].ulPort, (uint8_t)g_APinDescription[8].ulPin};

    Ox.CTRL = {(uint8_t)g_APinDescription[A3].ulPort, (uint8_t)g_APinDescription[A3].ulPin};
    Ox.DIR_SEL = {(uint8_t)g_APinDescription[9].ulPort, (uint8_t)g_APinDescription[9].ulPin};
    Ox.STOP = {(uint8_t)g_APinDescription[A0].ulPort, (uint8_t)g_APinDescription[A0].ulPin};
    Ox.ENC_CLK = {(uint8_t)g_APinDescription[5].ulPort, (uint8_t)g_APinDescription[5].ulPin};
    Ox.ENC_DATA = {(uint8_t)g_APinDescription[4].ulPort, (uint8_t)g_APinDescription[4].ulPin};
    Ox.SER_OUT = {(uint8_t)g_APinDescription[6].ulPort, (uint8_t)g_APinDescription[6].ulPin};
    Ox.CS = {(uint8_t)g_APinDescription[7].ulPort, (uint8_t)g_APinDescription[7].ulPin};

    enc = &Fuel;

    sPrintln("before");
    Serial1.flush();
    configureClocks();
    sPrintln("clocks");
    attachPins();
    sPrintln("after");
    // -TODO: Init periphs
    Fuel.status = 1u;
    Ox.status = 1u;
    // -TODO: Establish coms
    Fuel.status = 2u;
    Ox.status = 2u;
    sPrintln("waiting");
    while (!Serial1.available())
        ;
    sPrintln("testing");
}

void loop() {
    // sPrintln(STATUS_NAME[enc->status]);
    sPrint("BUTTON_ONE: ");
    sPrintln((!!(((Port *)0x60000000UL) /**< \brief (PORT) IOBUS Base Address */->Group[EXT.BUTTON_ONE.port].IN.reg & (1 << EXT.BUTTON_ONE.pin))));
    sPrint("BUTTON_TWO: ");
    sPrintln((!!(((Port *)0x60000000UL) /**< \brief (PORT) IOBUS Base Address */->Group[EXT.BUTTON_TWO.port].IN.reg & (1 << EXT.BUTTON_TWO.pin))));
    sPrint("SEL_SWITCH: ");
    sPrintln((!!(((Port *)0x60000000UL) /**< \brief (PORT) IOBUS Base Address */->Group[EXT.SEL_SWITCH.port].IN.reg & (1 << EXT.SEL_SWITCH.pin))));
    delay(500);
    switch (enc->status) {
        case 2u:
            while ((!!(((Port *)0x60000000UL) /**< \brief (PORT) IOBUS Base Address */->Group[EXT.SEL_SWITCH.port].IN.reg & (1 << EXT.SEL_SWITCH.pin)))) {
                if (!(!!(((Port *)0x60000000UL) /**< \brief (PORT) IOBUS Base Address */->Group[EXT.BUTTON_ONE.port].IN.reg & (1 << EXT.BUTTON_ONE.pin)))) {
                    (true ? ((Port *)0x60000000UL) /**< \brief (PORT) IOBUS Base Address */->Group[enc->DIR_SEL.port].OUTSET.reg |= (1 << enc->DIR_SEL.pin) : ((Port *)0x60000000UL) /**< \brief (PORT) IOBUS Base Address */->Group[enc->DIR_SEL.port].OUTCLR.reg |= (1 << enc->DIR_SEL.pin));
                    ((Tcc *)0x42002000UL) /**< \brief (TCC0) APB Base Address */->CCB[enc->CCB].reg = .15 * 480u /* 48MHz / (2 + [480]) = .05MHz = 50KHz*/;
                    sPrintln("MoveLeft");
                } else if (!(!!(((Port *)0x60000000UL) /**< \brief (PORT) IOBUS Base Address */->Group[EXT.BUTTON_TWO.port].IN.reg & (1 << EXT.BUTTON_TWO.pin)))) {
                    (false ? ((Port *)0x60000000UL) /**< \brief (PORT) IOBUS Base Address */->Group[enc->DIR_SEL.port].OUTSET.reg |= (1 << enc->DIR_SEL.pin) : ((Port *)0x60000000UL) /**< \brief (PORT) IOBUS Base Address */->Group[enc->DIR_SEL.port].OUTCLR.reg |= (1 << enc->DIR_SEL.pin));
                    ((Tcc *)0x42002000UL) /**< \brief (TCC0) APB Base Address */->CCB[enc->CCB].reg = .15 * 480u /* 48MHz / (2 + [480]) = .05MHz = 50KHz*/;
                    sPrintln("MoveRight");
                } else {
                    ((Tcc *)0x42002000UL) /**< \brief (TCC0) APB Base Address */->CCB[enc->CCB].reg = 0;
                    sPrintln("Stop");
                }
                delay(100);
                // sPrintln("movement on");
            }
            if (!(!!(((Port *)0x60000000UL) /**< \brief (PORT) IOBUS Base Address */->Group[EXT.BUTTON_ONE.port].IN.reg & (1 << EXT.BUTTON_ONE.pin))) && !(!!(((Port *)0x60000000UL) /**< \brief (PORT) IOBUS Base Address */->Group[EXT.BUTTON_TWO.port].IN.reg & (1 << EXT.BUTTON_TWO.pin)))) {
                if (!(!!(((Port *)0x60000000UL) /**< \brief (PORT) IOBUS Base Address */->Group[EXT.SEL_SWITCH.port].IN.reg & (1 << EXT.SEL_SWITCH.pin)))) {
                    enc->target = (byte)((90 / 360) * 15u /* Revs into gearbox per 1 revolution out*/) /* Number of rotations to get within one rev of fully open*/;
                    enc->status = 3u;
                }
            }
            break;
        case 3u:
            if (Fuel.status == 3u && Ox.status == 3u) {
                Fuel.status = 4u;
                Ox.status = 4u;
            }
            break;
        case 4u:
            // TODO:
            // -If serEn, receive data from SERCOM5
            //     -If data=launch:
            //         enc->status = STATUS_ACTIVATE;
            //     -If data=abort:
            //         enc->status = STATUS_ABORT;
            // -If !serEn:
            //     -If PORT_READ(PORT_EXT_TX, PIN_EXT_TX):
            //         enc->status = STATUS_ACTIVATE;
            if (serEn) {
                //-If data=launch:
                //    enc->status = STATUS_ACTIVATE;
                //-If data=abort:
                //    enc->status = STATUS_ABORT;
            } else {
                if (!(!!(((Port *)0x60000000UL) /**< \brief (PORT) IOBUS Base Address */->Group[EXT.TX.port].IN.reg & (1 << EXT.TX.pin)))) {
                    enc->status = 5u;
                }
            }
            break;
        case 5u:
            enc->TC->COUNT16.CTRLA.bit.ENABLE = 1;
            enc->status = 6u;
            break;
        case 6u:
            update(*enc);
            break;
        case 7u:
            enc->target = enc->origin;
            break;

        default:
            break;
    }
    // enc = (enc == &Fuel) ? &Ox : &Fuel;
}

void sPrint(int i, uint8_t base = 10) {
    Serial1.print(i, base);
    Serial1.flush();
}

void sPrintln(int i, uint8_t base = 10) {
    sPrint(i, base);
    sPrintln();
}

void sPrint(const char *str) {
    Serial1.print(str);
    Serial1.flush();
}

void sPrintln(const char *str) {
    sPrint(str);
    sPrintln();
}

void sPrintln(bool b) {
    sPrint((int)b);
    sPrintln();
}

void sPrintln(void) { sPrint("\r\n"); }

// PID loop. General implementation.
void update(Encoder enc) {
    uint16_t dt = enc.TC->COUNT16.COUNT.reg; // Time since last loop, in us
    enc.TC->COUNT16.CTRLBSET.reg |= (0x1ul /**< \brief (TC_CTRLBSET) Force a start, restart or retrigger */ << 6 /**< \brief (TC_CTRLBSET) Command */);
    float theta_n = enc.totalRevs + getPos(enc);

    // Integral approximation based on trapezoidal Riemann sum
    enc.accumulator += (1 / 2) * dt * enc.target * (theta_n + enc.totalRevs - 2);

    // Calculate PID output
    float O = (enc.P * (theta_n - enc.target)) + (enc.I * enc.accumulator) + (enc.D * ((theta_n - enc.totalRevs) / dt));

    (O > 0 ? ((Port *)0x60000000UL) /**< \brief (PORT) IOBUS Base Address */->Group[enc.DIR_SEL.port].OUTSET.reg |= (1 << enc.DIR_SEL.pin) : ((Port *)0x60000000UL) /**< \brief (PORT) IOBUS Base Address */->Group[enc.DIR_SEL.port].OUTCLR.reg |= (1 << enc.DIR_SEL.pin));

    ((Tcc *)0x42002000UL) /**< \brief (TCC0) APB Base Address */->CCB[enc.CCB].reg = (((O)>0?(O):-(O)) > 480u /* 48MHz / (2 + [480]) = .05MHz = 50KHz*/) ? 480u /* 48MHz / (2 + [480]) = .05MHz = 50KHz*/ : ((O)>0?(O):-(O));

    enc.totalRevs = theta_n;
}

// Returns encoder position, centered on the home value established during startup. For use with PID loop, as this
// enables it to pick the most efficient route to the target position. Must reset clock before calling.
float getPos(Encoder enc) {
    uint32_t npos = readRawEncData(enc);
    uint32_t ppos = enc.prev_t;
    while (enc.TC->COUNT16.COUNT.reg < 20)
        ;
    if (readRawEncData(enc) ^ npos != 0) {
        enc.err = 4u /* Encoder reads do not match*/;
        error(false);
    } else {
        enc.TC->COUNT16.CTRLBSET.reg |= (0x1ul /**< \brief (TC_CTRLBSET) Force a start, restart or retrigger */ << 6 /**< \brief (TC_CTRLBSET) Command */);

        npos = (npos ^ 0b011111111111111111000000 /* Bits [22:6]*/) >> 6 /* Number of digits to drop at right to read from data after masking*/; // Extract value from raw bits

        enc.prev_t = npos;
        if (((npos - ppos)>0?(npos - ppos):-(npos - ppos)) < (1 << (17 - 1))) {
            return (npos - ppos) / (1 << 17);
        } else {
            return (npos - ppos) + ((((((int32_t)(1 << 17) ^ (npos > ppos)) + ((npos > ppos) & 1))) /* Negates x if y (bit) is 1, only works for 2s complement*/));
        }
    }
}

// Takes reading from fuel encoder. Throws error if first latch bit is not 1, or encoder reading returns internal error state
// (last bit is 0).
uint32_t readRawEncData(Encoder enc) {
    uint32_t rawData = 0;
    for (byte i = 0; i < 24u /* Total number of bits in encoder packet.*/; i += 8) {
        while (!enc.SERCOM->SPI.INTFLAG.bit.DRE)
            ;
        enc.SERCOM->SPI.DATA.reg = ((Fuel.err << 4u) | (Ox.err << 0u));

        while (!enc.SERCOM->SPI.INTFLAG.bit.RXC)
            ;
        rawData |= enc.SERCOM->SPI.DATA.reg << 24u /* Total number of bits in encoder packet.*/ - (i * 8);
    }

    if (rawData & 1 << 24u /* Total number of bits in encoder packet.*/) {
        enc.err = 2u /* Bad connection with encoder*/;
        error(false);
    }
    if (!(rawData & 1)) { // Error bit is 0.
        enc.err = 3u /* Encoder passes internal error state*/;
        error(false);
    }
    return rawData;
}

// Outputs error info to Serial1, if fatal error closes valves.
void error(bool fatal) {
    sPrintln("Error occured!");
    sPrintln("isFatal: " + fatal);
    sPrintln("errorCode: " + ((Fuel.err << 4u) | (Ox.err << 0u)));
    sPrintln("Status: " + ((Fuel.err << 5u) | (Ox.err << 2u)) /* Merge errors/statuses for LED output*/);

    while (!((Sercom *)0x42000800UL) /**< \brief (SERCOM0) APB Base Address */->SPI.INTFLAG.bit.DRE)
        ;
    ((Sercom *)0x42000800UL) /**< \brief (SERCOM0) APB Base Address */->SPI.DATA.reg = ((Fuel.err << 4u) | (Ox.err << 0u));

    if (fatal) {
        Fuel.status = 7u;
        Ox.status = 7u;
    }
}

// Configure GPIO pins per 23.6.3 of SAMD datasheet
void configureGPIO(byte portNo, byte pinNo, bool DIR, bool INEN = 0, bool PULLEN = 0, bool OUT = 0, bool SAMPLE = 0) {
    ((Port *)0x41004400UL) /**< \brief (PORT) APB Base Address */->Group[portNo].DIR.reg |= DIR << pinNo;
    ((Port *)0x41004400UL) /**< \brief (PORT) APB Base Address */->Group[portNo].PINCFG[pinNo].bit.INEN = INEN;
    ((Port *)0x41004400UL) /**< \brief (PORT) APB Base Address */->Group[portNo].PINCFG[pinNo].bit.PULLEN = PULLEN;
    ((Port *)0x41004400UL) /**< \brief (PORT) APB Base Address */->Group[portNo].OUT.reg |= OUT << pinNo;
    ((Port *)0x41004400UL) /**< \brief (PORT) APB Base Address */->Group[portNo].CTRL.reg |= SAMPLE << pinNo;
}
// Checks to see if Serial1 communications port has been enabled (active low)
bool serialEnabled() {
    configureGPIO(SER_EN.port, SER_EN.pin, 0, 1, 1, INPUT_PULLUP);
    return !(!!(((Port *)0x60000000UL) /**< \brief (PORT) IOBUS Base Address */->Group[SER_EN.port].IN.reg & (1 << SER_EN.pin)));
}
// Configures internal peripherals and attaches to physical Arduino pins.
// A peripheral is an internal microcontroller node that has functions
// independent of the main controller thread. For example, TCs (Timer/Counters)
// maintain a clock rate, but do not have to be toggled on and off (eg. by interrupts)
// once they have been started. SERCOM (or Serial1 COMmunication ports) allow for communication
// through various Serial1 interfaces to external devices, but once they have been given data
// to transmit or receive they handle all pin modulation, clock timing, etc. independently.
// All one has to do is configure the SERCOM (or whatever peripheral)
// and read/write the associated data register and it handles everything else.
// Different pins have different possible peripheral connections,
// and which pins connect where can be found in Nano_33iot_variant.xlsx,
// which was compiled from the variant.h and variant.cpp files found
// in the Arduino Core, which is in turn created using the specific Arduino pin
// schematic and the SAMD21 datasheet, Table 7.1. You will see some peripherals
// throughout the program require read- and write-synchronization. This is because they runbit
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
    ((Port *)0x41004400UL) /**< \brief (PORT) APB Base Address */->Group[Fuel.CTRL.port].PINCFG[Fuel.CTRL.pin].bit.PMUXEN = 1;
    ((Port *)0x41004400UL) /**< \brief (PORT) APB Base Address */->Group[Ox.CTRL.port].PINCFG[Ox.CTRL.port].bit.PMUXEN = 1;
    sPrintln("375");

    // Enable the port multiplexer for Encoder coms and status register pins
    ((Port *)0x41004400UL) /**< \brief (PORT) APB Base Address */->Group[Fuel.ENC_CLK.port].PINCFG[Fuel.ENC_CLK.pin].bit.PMUXEN = 1;
    ((Port *)0x41004400UL) /**< \brief (PORT) APB Base Address */->Group[Fuel.ENC_DATA.port].PINCFG[Fuel.ENC_DATA.pin].bit.PMUXEN = 1;
    ((Port *)0x41004400UL) /**< \brief (PORT) APB Base Address */->Group[Fuel.SER_OUT.port].PINCFG[Fuel.SER_OUT.pin].bit.PMUXEN = 1;
    ((Port *)0x41004400UL) /**< \brief (PORT) APB Base Address */->Group[Fuel.CS.port].PINCFG[Fuel.CS.pin].bit.PMUXEN = 1;
    sPrintln("382");
    // Status LED shift registers MOSI and CS technically part of the encoder SERCOMs. As it is not neccesary to
    // write data to the encoders, and we are out of SERCOMs to communicate with the registers, they have been connected to
    // the communication pins (MOSI and CS, as well as SCK in parallel) for the encoders (Green->FUEL, Red->OX). This means
    // that the shift registers can be set during any read of the encoders. Normally this would be suboptimal as it would
    // constrain the output only to times the encoder is able to be read from, but seeing as the LEDs
    // are only a human information interface, the timing is very non-critical and as long as we maintain the >20us
    // turnaround time between encoder reads, it will not present any issues.

    // SERCOM0 SCK and MISO pins for OX

    // Enable the port multiplexer for PWM pins
    ((Port *)0x41004400UL) /**< \brief (PORT) APB Base Address */->Group[Ox.CTRL.port].PINCFG[Ox.CTRL.pin].bit.PMUXEN = 1;
    ((Port *)0x41004400UL) /**< \brief (PORT) APB Base Address */->Group[Ox.CTRL.port].PINCFG[Ox.CTRL.port].bit.PMUXEN = 1;
    sPrintln("396");

    // Enable the port multiplexer for Encoder coms and status register pins
    ((Port *)0x41004400UL) /**< \brief (PORT) APB Base Address */->Group[Ox.ENC_CLK.port].PINCFG[Ox.ENC_CLK.pin].bit.PMUXEN = 1;
    ((Port *)0x41004400UL) /**< \brief (PORT) APB Base Address */->Group[Ox.ENC_DATA.port].PINCFG[Ox.ENC_DATA.pin].bit.PMUXEN = 1;
    ((Port *)0x41004400UL) /**< \brief (PORT) APB Base Address */->Group[Ox.SER_OUT.port].PINCFG[Ox.SER_OUT.pin].bit.PMUXEN = 1;
    ((Port *)0x41004400UL) /**< \brief (PORT) APB Base Address */->Group[Ox.CS.port].PINCFG[Ox.CS.pin].bit.PMUXEN = 1;
    sPrintln("403");

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
    // mask to maintain the LS value. This is good practice when writing to any register that contains multiple
    // parameters. If we wanted to replace an ID (without knowing the initial state or wanting to clear the other ID), we
    // would have to do: x.reg &= [15 (00001111) for replacing first four | 240 (11110000) for replacing last 4]; x.reg
    // |= [bitmask]; The ID mask names are formatted as follows: PORT_PMUX_PMUX[O (odd)| E (even)]_[A:H (peripheral ID)].
    // More info can be found in 23.8.12

    // Attach PWM pins to timer peripherals
    ((Port *)0x41004400UL) /**< \brief (PORT) APB Base Address */->Group[Fuel.CTRL.port].PMUX[Fuel.CTRL.pin >> 1].reg |=
        ((Fuel.CTRL.pin % 2) == 0) ? (0x4ul /**< \brief (PORT_PMUX) Peripheral function E selected */ << 0 /**< \brief (PORT_PMUX) Peripheral Multiplexing Even */) : (0x4ul /**< \brief (PORT_PMUX) Peripheral function E selected */ << 4 /**< \brief (PORT_PMUX) Peripheral Multiplexing Odd */);
    ((Port *)0x41004400UL) /**< \brief (PORT) APB Base Address */->Group[Ox.CTRL.port].PMUX[Ox.CTRL.pin >> 1].reg |= ((Ox.CTRL.pin % 2) == 0) ? (0x4ul /**< \brief (PORT_PMUX) Peripheral function E selected */ << 0 /**< \brief (PORT_PMUX) Peripheral Multiplexing Even */) : (0x4ul /**< \brief (PORT_PMUX) Peripheral function E selected */ << 4 /**< \brief (PORT_PMUX) Peripheral Multiplexing Odd */);
    sPrintln("428");

    // Attach Encoder coms and status register pins to FUEL SERCOM
    ((Port *)0x41004400UL) /**< \brief (PORT) APB Base Address */->Group[Fuel.ENC_CLK.port].PMUX[Fuel.ENC_CLK.pin >> 1].reg |=
        ((Fuel.ENC_CLK.pin % 2) == 0) ? (0x2ul /**< \brief (PORT_PMUX) Peripheral function C selected */ << 0 /**< \brief (PORT_PMUX) Peripheral Multiplexing Even */) : (0x2ul /**< \brief (PORT_PMUX) Peripheral function C selected */ << 4 /**< \brief (PORT_PMUX) Peripheral Multiplexing Odd */);
    ((Port *)0x41004400UL) /**< \brief (PORT) APB Base Address */->Group[Fuel.ENC_DATA.port].PMUX[Fuel.ENC_DATA.pin >> 1].reg |=
        ((Fuel.ENC_DATA.pin % 2) == 0) ? (0x2ul /**< \brief (PORT_PMUX) Peripheral function C selected */ << 0 /**< \brief (PORT_PMUX) Peripheral Multiplexing Even */) : (0x2ul /**< \brief (PORT_PMUX) Peripheral function C selected */ << 4 /**< \brief (PORT_PMUX) Peripheral Multiplexing Odd */);
    ((Port *)0x41004400UL) /**< \brief (PORT) APB Base Address */->Group[Fuel.SER_OUT.port].PMUX[Fuel.SER_OUT.pin >> 1].reg |=
        ((Fuel.SER_OUT.pin % 2) == 0) ? (0x2ul /**< \brief (PORT_PMUX) Peripheral function C selected */ << 0 /**< \brief (PORT_PMUX) Peripheral Multiplexing Even */) : (0x2ul /**< \brief (PORT_PMUX) Peripheral function C selected */ << 4 /**< \brief (PORT_PMUX) Peripheral Multiplexing Odd */);
    ((Port *)0x41004400UL) /**< \brief (PORT) APB Base Address */->Group[Fuel.CS.port].PMUX[Fuel.CS.pin >> 1].reg |= ((Fuel.CS.pin % 2) == 0) ? (0x2ul /**< \brief (PORT_PMUX) Peripheral function C selected */ << 0 /**< \brief (PORT_PMUX) Peripheral Multiplexing Even */) : (0x2ul /**< \brief (PORT_PMUX) Peripheral function C selected */ << 4 /**< \brief (PORT_PMUX) Peripheral Multiplexing Odd */);
    sPrintln("438");

    // Attach Encoder coms and status register pins to OX SERCOM
    ((Port *)0x41004400UL) /**< \brief (PORT) APB Base Address */->Group[Ox.ENC_CLK.port].PMUX[Ox.ENC_CLK.pin >> 1].reg |=
        ((Ox.ENC_CLK.pin % 2) == 0) ? (0x2ul /**< \brief (PORT_PMUX) Peripheral function C selected */ << 0 /**< \brief (PORT_PMUX) Peripheral Multiplexing Even */) : (0x2ul /**< \brief (PORT_PMUX) Peripheral function C selected */ << 4 /**< \brief (PORT_PMUX) Peripheral Multiplexing Odd */);
    ((Port *)0x41004400UL) /**< \brief (PORT) APB Base Address */->Group[Ox.ENC_DATA.port].PMUX[Ox.ENC_DATA.pin >> 1].reg |=
        ((Ox.ENC_DATA.pin % 2) == 0) ? (0x2ul /**< \brief (PORT_PMUX) Peripheral function C selected */ << 0 /**< \brief (PORT_PMUX) Peripheral Multiplexing Even */) : (0x2ul /**< \brief (PORT_PMUX) Peripheral function C selected */ << 4 /**< \brief (PORT_PMUX) Peripheral Multiplexing Odd */);
    ((Port *)0x41004400UL) /**< \brief (PORT) APB Base Address */->Group[Ox.SER_OUT.port].PMUX[Ox.SER_OUT.pin >> 1].reg |=
        ((Ox.SER_OUT.pin % 2) == 0) ? (0x2ul /**< \brief (PORT_PMUX) Peripheral function C selected */ << 0 /**< \brief (PORT_PMUX) Peripheral Multiplexing Even */) : (0x2ul /**< \brief (PORT_PMUX) Peripheral function C selected */ << 4 /**< \brief (PORT_PMUX) Peripheral Multiplexing Odd */);
    ((Port *)0x41004400UL) /**< \brief (PORT) APB Base Address */->Group[Ox.CS.port].PMUX[Ox.CS.pin >> 1].reg |= ((Ox.CS.pin % 2) == 0) ? (0x2ul /**< \brief (PORT_PMUX) Peripheral function C selected */ << 0 /**< \brief (PORT_PMUX) Peripheral Multiplexing Even */) : (0x2ul /**< \brief (PORT_PMUX) Peripheral function C selected */ << 4 /**< \brief (PORT_PMUX) Peripheral Multiplexing Odd */);
    sPrintln("448");

    // See configureClocks() for information regarding GCLK configuration and linking to periphs.

    // TCCs, or Timer/Counters for Control applications, are a Timer/Counter peripheral with added logical functionality
    // in order to function as controllers for other peripherals or external devices. Their main advantage over standard
    // TCs, at least for our application, is their greatly improved waveform generation and double-buffered setup which
    // allows for more nuanced PWM control. This is not currently neccessary for our application, but it very well could
    // be in future iterations. There are 4 TCC instances, TCC0 through TCC3, each with slightly different specs. Each
    // TCC is linked to a GCLK through a prescaler.  In our case with waveform generation, this event is toggling the
    // waveform output to create a PWM wave. Each TCC has up to four Compare/Capture channels, CC(B)[0:3]. CCBx is double
    // buffered through CCx, which means that writing to CCBx takes several clock cycles longer to take effect but is
    // synchronized as to prevent glitches during waveform period transitions.. Each CC channel is linked to 2 waveform
    // generators WO[0:8].This fact is not made very clear in the datasheet, so I provided the default connection
    // configuration in the program header. More information about waveform generation, PWM frequency vs resolution, and
    // pulse width can be found in Section 31.6 of SAMD datasheet.

    // Configure TCC0
    // Setup basic dual slope PWM on TCC0 (See 31.6.2.5.6 in SAMD datasheet). Note that we are writing, not ORing, the
    // registers as we need them to only contain the value we give it and do not care about any previous settings.
    // THE PWM pins on the Arduino were chosen such that both pins connect to the same TCC (in this case TCC0) on
    // separate waveform outputs, which means we only have to spin up and configure a single peripheral, and just change
    // the corresponding CCBx value to change the outputs. Note that every TCC does not neccessarily share the same
    // configuration options or parameters. Should it be neccesary to change pins and spin up a different TCC, while it
    // may be possible to just copy-paste the original config routine, that should not be assumed.
    ((Tcc *)0x42002000UL) /**< \brief (TCC0) APB Base Address */->WAVE.reg = (0x2ul /**< \brief (TCC_WAVE) Normal PWM */ << 0 /**< \brief (TCC_WAVE) Waveform Generation */);
    while (((Tcc *)0x42002000UL) /**< \brief (TCC0) APB Base Address */->SYNCBUSY.bit.WAVE)
        ; // Sync

    ((Tcc *)0x42002000UL) /**< \brief (TCC0) APB Base Address */->PER.reg = 480u /* 48MHz / (2 + [480]) = .05MHz = 50KHz*/; // Set PWM frequency to 50KHz (f=GCLK/(2+PER))
    while (((Tcc *)0x42002000UL) /**< \brief (TCC0) APB Base Address */->SYNCBUSY.bit.PER)
        ; // Sync

    ((Tcc *)0x42002000UL) /**< \brief (TCC0) APB Base Address */->CTRLA.bit.ENABLE = 1u; // Enable the TCC0 counter
    while (((Tcc *)0x42002000UL) /**< \brief (TCC0) APB Base Address */->SYNCBUSY.bit.ENABLE)
        ; // Sync

    sPrintln("485");

    // Much of the TCC information applies with TCs as well. The biggest difference is that TCs lack a double buffered
    // input, and instead only have CCx. This is fine as we are only using TC4 and TC5 as timers for the PID controllers,
    // and do not need any waveform generation from them. The counter increments from 0 or decrements from the max value
    // depending on configuration, and COUNT register is compared to the value in the Compare/Capture register (CCx).
    // Every time the count hits CCx, it resets to 0/max and triggers an event. See Chapter 30 for more details. TC4 and
    // TC5 are configured identically, as we are using them to run the FUEL and OX PID controllers respectively.

    //
    ((Tc *)0x42003000UL) /**< \brief (TC4) APB Base Address */->COUNT16.CTRLA.reg =
        (0x0ul /**< \brief (TC_CTRLA) Counter in 16-bit mode */ << 2 /**< \brief (TC_CTRLA) TC Mode */) | // Select 16 bit mode for TC4
        (0x3ul /**< \brief (TC_CTRLA) Prescaler: GCLK_TC/8 */ << 8 /**< \brief (TC_CTRLA) Prescaler */) | // Divides frequency by 8, to get f_TC of 1MHz, or period=1us
        (0x1ul /**< \brief (TC_CTRLA) Reload or reset the counter on next prescaler clock */ << 12 /**< \brief (TC_CTRLA) Prescaler and Counter Synchronization */); // Clock reset is connected to prescaler clock not GCLK. Keeps 1us period.

    while (((Tc *)0x42003000UL) /**< \brief (TC4) APB Base Address */->COUNT16.STATUS.bit.SYNCBUSY)
        ; // sync

    // Sets continuous read synchronization of count register. This means it is not neccesary to set the RREQ flag and
    // wait to synchronize every time we want to read the COUNT value.See section 30.10.2 in MC datasheet.
    ((Tc *)0x42003000UL) /**< \brief (TC4) APB Base Address */->COUNT16.READREQ.reg = (0x1ul << 14 /**< \brief (TC_READREQ) Read Continuously */) | // Sets RCONT flag, disabling automatic clearing of RREQ flag after sync
                               (0x1ul << 15 /**< \brief (TC_READREQ) Read Request */) | // Read sync request flag, synchronizes COUNT register for reading
                               ((0x1Ful << 0 /**< \brief (TC_READREQ) Address */) & ((0x10) << 0 /**< \brief (TC_READREQ) Address */)); // COUNT register address

    sPrintln("509");
    // Same as TC4
    ((Tc *)0x42003400UL) /**< \brief (TC5) APB Base Address */->COUNT16.CTRLA.reg =
        (0x0ul /**< \brief (TC_CTRLA) Counter in 16-bit mode */ << 2 /**< \brief (TC_CTRLA) TC Mode */) | // Select 16-bit mode for TC5.
        (0x3ul /**< \brief (TC_CTRLA) Prescaler: GCLK_TC/8 */ << 8 /**< \brief (TC_CTRLA) Prescaler */) | // Divides GCLK frequency by 8, to get f_TC of 8Mhz/8=1MHz, or period 1us
        (0x1ul /**< \brief (TC_CTRLA) Reload or reset the counter on next prescaler clock */ << 12 /**< \brief (TC_CTRLA) Prescaler and Counter Synchronization */); // Clock reset is connected to prescaler clock not GCLK. Keeps 1us period.
    while (((Tc *)0x42003400UL) /**< \brief (TC5) APB Base Address */->COUNT16.STATUS.bit.SYNCBUSY)
        ; // sync
    // Same as TC4
    ((Tc *)0x42003400UL) /**< \brief (TC5) APB Base Address */->COUNT16.READREQ.reg = (0x1ul << 14 /**< \brief (TC_READREQ) Read Continuously */) | // Sets RCONT flag, disabling automatic clearing of RREQ flag after sync
                               (0x1ul << 15 /**< \brief (TC_READREQ) Read Request */) | // Read sync request flag, synchronizes COUNT register for reading
                               ((0x1Ful << 0 /**< \brief (TC_READREQ) Address */) & ((0x10) << 0 /**< \brief (TC_READREQ) Address */)); // COUNT register address

    sPrintln("522");

    // The MC has 6 SERCOM, or Serial1 COMmunication, peripherals, SERCOM0:SERCOM5. Each SERCOM peripheral has 4 pads
    // associated with it. "Pad" just means an IO line for the peripheral, similar to channels in TCCs. MC pins are
    // associated with both a SERCOM and a pad. Some MC pins have a primary and alternate set. This is why the FUEL pins
    // are attached to peripheral E, while OX is attached to peripheral D. Both are attached to SERCOMs, but system
    // requirements meant that we had to use the alternate set for FUEL (note that the FUEL pins do not actually have a
    // primary set, as seen in the variant table). The function of each pad is defined by the chosen SERCOM operation
    // protocol and configuration. SERCOMs 1:5 are already reserved by the Arduino (this can be seen in variant.h), which
    // leaves us with only SERCOM0 to use as we want (without overriding an existing Serial1 port connection).
    // Fortunately, SERCOM1 has been reserved as the stock SPI port, which means we actually have enough to do what we
    // need.

    // SPI, or Serial1 Peripheral Interface, is a communications protocol developed for low level communications between
    // things like sensors, ICs, encoders, or other peripherals. It is what we are using on SERCOM0 and SERCOM 1. Our
    // encoders use a subset of SPI functionality to output their readings (they only output data, not take it in). The
    // SPI protocol is based around the shift register digital circuit, which Ben Eater has several fantastic YouTube
    // videos on. This means that we can also use it directly with any compatible shift register IC, even without a
    // peripheral abstraction. SAMD SERCOM's SPI mode supports full-duplex communications, which means it can read and
    // write at the same time. What this means for our application is we can control up to 4 single-direction devices
    // with just two SERCOM interfaces (2 inputs and 2 outputs). There are many online resources that do a better job of
    // explaining SPI, but the basics are as follows: full-duplex SPI has 4 pins: MISO, MOSI, SCK, and CS. MOSI and MISO
    // are data pins, standing for Master-Out-Slave-In and Master-In-Slave-Out. Fairly self-explanatory, in our case as
    // the host MISO is the input and MOSI is the output pin for the SERCOM. SCK is the Serial1 ClocK. SPI does not have a
    // native baudrate, instead data is only sent through MOSI and MISO on the leading or trailing edge of SCK's clock
    // pulse (depending on configuration). This type of shared-clock communication is described as Synchronous. This is
    // in contrast to Asynchronous communication protocols like UART (which SERCOM also supports), in which the devices
    // decide on the communication frequency during the initial handshake and then send data based on that with no regard
    // for the other device's timing. The CS/Chip Select, or SS/SPI Select (it has been called different things through
    // different version of the protocol) line is for, as the name suggests, selecting which chip the data is going
    // to/coming from along MOSI and MISO. Notably, MISO, MOSI, and SCK all can be connected to peripherals in parallel.
    // There is one CS line ran from the host to each peripheral. This means that SPI can scale to many devices
    // relatively well, as you only need to allocate 1 extra pin per added device. Pulling CS low unlatches the
    // peripheral's shift register, allowing for data transfer to that device. We will use this fact along with the
    // full-duplex support to allow us to interact with multiple devices on the same SPI port, in this case both the FUEL
    // encoder and the LED status shift register, as was mentioned briefly above. Unfortunately the encoders do not have
    // a CS input, and so it is not possible to place them on the same SPI port without some circuit modifications on our
    // end. Though this is not impossible (all it would take is a discrete AND gate IC), it was decided to maintain
    // separate SPI ports in case we decide to explore parallelization in the future. The configuration for the SPI
    // SERCOMs is fairly self explanatory, as we are using many default configuration parameters. When using SPI with a
    // shift register (in this case the SN74HCT595 acting as our LED parallel display), connections should be made as
    // follows: MOSI->Serial1 input (SER), SCK->Serial1 clock (SRCLK), and CS->Storage register clock/latch pin (RCLK). See
    // Chapter 27 in the MC datasheet, the datasheets for the NME2 (encoder) and SN74HC595 (LED shift register), and the
    // circuit schematic for more info.

    // The other notable configuration step is the baudrate, or data transfer speed. The encoders support up to 10MHz
    // (the shift register much more than that), but we do not necessarily want to approach that as our system needs to
    // be[ reliable and tolerant of suboptimal EMI conditions. 1MHz was chosen due to it's convenient bitrate of 1
    // bit/us, allowing for easy timing calculations. Depending on the operating mode of the SERCOM, the BAUD register
    // influences baudrate differently, but for synchronous operations (as is the case for SPI), the baud rate is defined
    // as f_ref/(2*(BAUD+1)), where f_ref is the reference frequency fed to the SERCOM by the GCLK. See section 25.6.2.3
    // in MC datasheet for more info.

    ((Sercom *)0x42000800UL) /**< \brief (SERCOM0) APB Base Address */->SPI.CTRLA.bit.ENABLE = 0;
    while (((Sercom *)0x42000800UL) /**< \brief (SERCOM0) APB Base Address */->SPI.SYNCBUSY.bit.ENABLE)
        ;
    ((Sercom *)0x42000800UL) /**< \brief (SERCOM0) APB Base Address */->SPI.CTRLA.reg = ((0x3ul << 20 /**< \brief (SERCOM_SPI_CTRLA) Data In Pinout */) & ((0x3u) << 20 /**< \brief (SERCOM_SPI_CTRLA) Data In Pinout */)) | // MISO is Pad 3
                             ((0x3ul << 16 /**< \brief (SERCOM_SPI_CTRLA) Data Out Pinout */) & ((0x0u) << 16 /**< \brief (SERCOM_SPI_CTRLA) Data Out Pinout */)) | // MOSI pad 0, SCK Pad 1, CS Pad 2
                             (0x3ul /**< \brief (SERCOM_SPI_CTRLA) SPI mode with internal clock */ << 2 /**< \brief (SERCOM_SPI_CTRLA) Operating Mode */); // Sets device mode as host
    ((Sercom *)0x42000800UL) /**< \brief (SERCOM0) APB Base Address */->SPI.CTRLB.reg = (0x1ul << 17 /**< \brief (SERCOM_SPI_CTRLB) Receiver Enable */) | (0x1ul << 13 /**< \brief (SERCOM_SPI_CTRLB) Master Slave Select Enable */); // Enable reciever/full-duplex operation
    while (((Sercom *)0x42000800UL) /**< \brief (SERCOM0) APB Base Address */->SPI.SYNCBUSY.bit.CTRLB)
        ;
    ((Sercom *)0x42000800UL) /**< \brief (SERCOM0) APB Base Address */->SPI.BAUD.reg = ((0xFFul << 0 /**< \brief (SERCOM_SPI_BAUD) Baud Rate Value */) & ((3u /* Baud value for SERCOM communications*/) << 0 /**< \brief (SERCOM_SPI_BAUD) Baud Rate Value */)); // Sets baudrate to 8MHz/(2*([BAUD=3]+1)=1MHz
    ((Sercom *)0x42000800UL) /**< \brief (SERCOM0) APB Base Address */->SPI.CTRLA.bit.ENABLE = 1;
    while (((Sercom *)0x42000800UL) /**< \brief (SERCOM0) APB Base Address */->SPI.SYNCBUSY.bit.ENABLE)
        ;

    sPrintln(1);
    ((Sercom *)0x42000C00UL) /**< \brief (SERCOM1) APB Base Address */->SPI.CTRLA.bit.ENABLE = 0;
    while (((Sercom *)0x42000C00UL) /**< \brief (SERCOM1) APB Base Address */->SPI.SYNCBUSY.bit.ENABLE)
        ;
    sPrintln(2);
    ((Sercom *)0x42000C00UL) /**< \brief (SERCOM1) APB Base Address */->SPI.CTRLA.reg = ((0x3ul << 20 /**< \brief (SERCOM_SPI_CTRLA) Data In Pinout */) & ((0x3u) << 20 /**< \brief (SERCOM_SPI_CTRLA) Data In Pinout */)) | // MISO is Pad 3
                             ((0x3ul << 16 /**< \brief (SERCOM_SPI_CTRLA) Data Out Pinout */) & ((0x0u) << 16 /**< \brief (SERCOM_SPI_CTRLA) Data Out Pinout */)) | // MOSI pad 0, SCK Pad 1, CS Pad 2
                             (0x3ul /**< \brief (SERCOM_SPI_CTRLA) SPI mode with internal clock */ << 2 /**< \brief (SERCOM_SPI_CTRLA) Operating Mode */); // Sets device mode as host
    ((Sercom *)0x42000C00UL) /**< \brief (SERCOM1) APB Base Address */->SPI.CTRLB.reg = (0x1ul << 17 /**< \brief (SERCOM_SPI_CTRLB) Receiver Enable */) | (0x1ul << 13 /**< \brief (SERCOM_SPI_CTRLB) Master Slave Select Enable */); // Enable reciever/full-duplex operation

    sPrintln(3);
    while (((Sercom *)0x42000C00UL) /**< \brief (SERCOM1) APB Base Address */->SPI.SYNCBUSY.bit.CTRLB)
        ;

    sPrintln(4);
    ((Sercom *)0x42000C00UL) /**< \brief (SERCOM1) APB Base Address */->SPI.BAUD.reg = ((0xFFul << 0 /**< \brief (SERCOM_SPI_BAUD) Baud Rate Value */) & ((3u /* Baud value for SERCOM communications*/) << 0 /**< \brief (SERCOM_SPI_BAUD) Baud Rate Value */)); // Sets baudrate to 8MHz/(2*([BAUD=3]+1)=1MHz

    sPrintln(5);
    ((Sercom *)0x42000C00UL) /**< \brief (SERCOM1) APB Base Address */->SPI.CTRLA.bit.ENABLE = 1;

    sPrintln(6);
    while (((Sercom *)0x42000C00UL) /**< \brief (SERCOM1) APB Base Address */->SPI.SYNCBUSY.bit.ENABLE)
        ;

    sPrintln(7);

    serEn = true;
    /*if (serialEnabled()) {

        Serial1.begin(115200);

        // TODO: Init coms/handshake

    } else {



        sPrintln("613");

        // Disable Serial1 USART

        SERCOM5->SPI.CTRLA.bit.ENABLE = 0;

        while (SERCOM1->SPI.SYNCBUSY.bit.ENABLE)

            ;

        SERCOM5->USART.CTRLA.bit.SWRST = 1;

        while (SERCOM5->USART.SYNCBUSY.bit.SWRST)

            ;

        // Configure TX as plain input

        configureGPIO(EXT.TX.port, EXT.TX.pin, INPUT, 1, 1, INPUT_PULLDOWN);



        sPrintln("624");

    }

    */
# 675 "Z:\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
    configureGPIO(Fuel.DIR_SEL.port, Fuel.DIR_SEL.pin, 1);
    configureGPIO(Fuel.STOP.port, Fuel.STOP.pin, 1);

    configureGPIO(Ox.DIR_SEL.port, Ox.DIR_SEL.pin, 1);
    configureGPIO(Ox.STOP.port, Ox.STOP.pin, 1);

    configureGPIO(EXT.BUTTON_ONE.port, EXT.BUTTON_ONE.pin, 0, 1, 1, 1, 1);
    configureGPIO(EXT.BUTTON_TWO.port, EXT.BUTTON_TWO.pin, 0, 1, 1, 1, 1);
    configureGPIO(EXT.SEL_SWITCH.port, EXT.SEL_SWITCH.pin, 0, 1, 1, 1, 1);

    sPrintln("637");
}

// TODO: Documentation
void configureClocks() {

    ((Pm *)0x40000400UL) /**< \brief (PM) APB Base Address */->APBCMASK.reg |= (0x1ul << 2 /**< \brief (PM_APBCMASK) SERCOM0 APB Clock Enable */) | (0x1ul << 3 /**< \brief (PM_APBCMASK) SERCOM1 APB Clock Enable */) | (0x1ul << 4 /**< \brief (PM_APBCMASK) SERCOM2 APB Clock Enable */) | (0x1ul << 5 /**< \brief (PM_APBCMASK) SERCOM3 APB Clock Enable */) |
                        (0x1ul << 6 /**< \brief (PM_APBCMASK) SERCOM4 APB Clock Enable */) | (0x1ul << 7 /**< \brief (PM_APBCMASK) SERCOM5 APB Clock Enable */) | (0x1ul << 8 /**< \brief (PM_APBCMASK) TCC0 APB Clock Enable */) | (0x1ul << 12 /**< \brief (PM_APBCMASK) TC4 APB Clock Enable */) | (0x1ul << 13 /**< \brief (PM_APBCMASK) TC5 APB Clock Enable */);

    // Link timer periphs to GCLK for PWM
    ((Gclk *)0x40000C00UL) /**< \brief (GCLK) APB Base Address */->GENCTRL.reg = ((0xFul << 0 /**< \brief (GCLK_GENCTRL) Generic Clock Generator Selection */) & ((4u) << 0 /**< \brief (GCLK_GENCTRL) Generic Clock Generator Selection */)) | // Edit GCLK4
                        (0x1ul << 17 /**< \brief (GCLK_GENCTRL) Improve Duty Cycle */) | // Improve 50/50 PWM
                        (0x1ul << 16 /**< \brief (GCLK_GENCTRL) Generic Clock Generator Enable */) | // Enable generic clock
                        (0x7ul /**< \brief (GCLK_GENCTRL) DFLL48M output */ << 8 /**< \brief (GCLK_GENCTRL) Source Select */); // Link to 48MHz source
    while (((Gclk *)0x40000C00UL) /**< \brief (GCLK) APB Base Address */->STATUS.bit.SYNCBUSY) ; /* Wait for clock synchronization*/;

    ((Gclk *)0x40000C00UL) /**< \brief (GCLK) APB Base Address */->CLKCTRL.reg = (0x4ul /**< \brief (GCLK_CLKCTRL) Generic clock generator 4 */ << 8 /**< \brief (GCLK_CLKCTRL) Generic Clock Generator */) | // Select GCLK4 at 48MHz to edit |
                        (0x1ul << 14 /**< \brief (GCLK_CLKCTRL) Clock Enable */) | // Enable GCLK4 as a clock source
                        (0x1Aul /**< \brief (GCLK_CLKCTRL) TCC0_TCC1 */ << 0 /**< \brief (GCLK_CLKCTRL) Generic Clock Selection ID */); // Feed GCLK4 to TCC0 and TCC1
    while (((Gclk *)0x40000C00UL) /**< \brief (GCLK) APB Base Address */->STATUS.bit.SYNCBUSY) ; /* Wait for clock synchronization*/;

    ((Gclk *)0x40000C00UL) /**< \brief (GCLK) APB Base Address */->CLKCTRL.reg = (0x4ul /**< \brief (GCLK_CLKCTRL) Generic clock generator 4 */ << 8 /**< \brief (GCLK_CLKCTRL) Generic Clock Generator */) | // Select GCLK4 at 48MHz to edit |
                        (0x1ul << 14 /**< \brief (GCLK_CLKCTRL) Clock Enable */) | // Enable GCLK4 as a clock source
                        (0x5ul /**< \brief (GCLK_CLKCTRL) EIC */ << 0 /**< \brief (GCLK_CLKCTRL) Generic Clock Selection ID */); // Feed GCLK4 to EIC
    while (((Gclk *)0x40000C00UL) /**< \brief (GCLK) APB Base Address */->STATUS.bit.SYNCBUSY) ; /* Wait for clock synchronization*/;

    // Link timer periphs for PID loops.
    ((Gclk *)0x40000C00UL) /**< \brief (GCLK) APB Base Address */->GENCTRL.reg = ((0xFul << 0 /**< \brief (GCLK_GENCTRL) Generic Clock Generator Selection */) & ((5u) << 0 /**< \brief (GCLK_GENCTRL) Generic Clock Generator Selection */)) | // Edit GCLK5
                        (0x1ul << 17 /**< \brief (GCLK_GENCTRL) Improve Duty Cycle */) | // Improve 50/50 PWM
                        (0x1ul << 16 /**< \brief (GCLK_GENCTRL) Generic Clock Generator Enable */) | // Enable generic clock
                        (0x6ul /**< \brief (GCLK_GENCTRL) OSC8M oscillator output */ << 8 /**< \brief (GCLK_GENCTRL) Source Select */); // Link to 8MHz source
    while (((Gclk *)0x40000C00UL) /**< \brief (GCLK) APB Base Address */->STATUS.bit.SYNCBUSY) ; /* Wait for clock synchronization*/;

    ((Gclk *)0x40000C00UL) /**< \brief (GCLK) APB Base Address */->CLKCTRL.reg = (0x5ul /**< \brief (GCLK_CLKCTRL) Generic clock generator 5 */ << 8 /**< \brief (GCLK_CLKCTRL) Generic Clock Generator */) | // Select GCLK5 at 8MHz to edit |
                        (0x1ul << 14 /**< \brief (GCLK_CLKCTRL) Clock Enable */) | // Enable GCLK5 as a clock source
                        (0x1Cul /**< \brief (GCLK_CLKCTRL) TC4_TC5 */ << 0 /**< \brief (GCLK_CLKCTRL) Generic Clock Selection ID */); // Feed GCLK5 to TC4 and TC5

    ((Gclk *)0x40000C00UL) /**< \brief (GCLK) APB Base Address */->CLKCTRL.reg = (0x5ul /**< \brief (GCLK_CLKCTRL) Generic clock generator 5 */ << 8 /**< \brief (GCLK_CLKCTRL) Generic Clock Generator */) | // Select GCLK5 at 8MHz to edit |
                        (0x1ul << 14 /**< \brief (GCLK_CLKCTRL) Clock Enable */) | // Enable GCLK5 as a clock source
                        (0x1Dul /**< \brief (GCLK_CLKCTRL) TC6_TC7 */ << 0 /**< \brief (GCLK_CLKCTRL) Generic Clock Selection ID */); // Feed GCLK5 to TC4 and TC5

    ((Gclk *)0x40000C00UL) /**< \brief (GCLK) APB Base Address */->CLKCTRL.reg = (0x5ul /**< \brief (GCLK_CLKCTRL) Generic clock generator 5 */ << 8 /**< \brief (GCLK_CLKCTRL) Generic Clock Generator */) | // Select GCLK5 at 8MHz to edit |
                        (0x1ul << 14 /**< \brief (GCLK_CLKCTRL) Clock Enable */) | // Enable GCLK5 as a clock source
                        (0x14ul /**< \brief (GCLK_CLKCTRL) SERCOM0_CORE */ << 0 /**< \brief (GCLK_CLKCTRL) Generic Clock Selection ID */); // Feed GCLK5 to SERCOM0 Baud Generator
    while (((Gclk *)0x40000C00UL) /**< \brief (GCLK) APB Base Address */->STATUS.bit.SYNCBUSY) ; /* Wait for clock synchronization*/;
    ((Gclk *)0x40000C00UL) /**< \brief (GCLK) APB Base Address */->CLKCTRL.reg = (0x5ul /**< \brief (GCLK_CLKCTRL) Generic clock generator 5 */ << 8 /**< \brief (GCLK_CLKCTRL) Generic Clock Generator */) | // Select GCLK5 at 8MHz to edit |
                        (0x1ul << 14 /**< \brief (GCLK_CLKCTRL) Clock Enable */) | // Enable GCLK5 as a clock source
                        (0x15ul /**< \brief (GCLK_CLKCTRL) SERCOM1_CORE */ << 0 /**< \brief (GCLK_CLKCTRL) Generic Clock Selection ID */); // Feed GCLK7 to SERCOM1 Baud Generator
    while (((Gclk *)0x40000C00UL) /**< \brief (GCLK) APB Base Address */->STATUS.bit.SYNCBUSY) ; /* Wait for clock synchronization*/;
}
