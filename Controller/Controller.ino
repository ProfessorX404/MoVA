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

OTMX Values:
CCx | WO[n]
CC0 | WO[0], WO[4]
CC1 | WO[1], WO[5]
CC2 | WO[2], WO[6]
CC3 | WO[3], WO[7]

IMPORTANT NOTE: ***MUST*** change pin definitions in variant.cpp for D4, D5, D6 to PIO_SERCOM_ALT instead of PIO_DIGITAL

*/
#include <SPI.h>
#include <array.h>

#define PULLUP                 (1u)
#define PULLDOWN               (0u)
#define ACTIVE_LOW             (0)
#define ACTIVE_HIGH            (1)
#define SAMPLING_ON            (1u)
#define SAMPLING_OFF           (0u)
#define ENC_TOT_BIT_CT         (24u)                        // Total number of bits in encoder packet.
#define ENC_DATA_BIT_CT        (17u)                        //
#define ENC_DATA_MASK          (0b011111111111111111000000) // Bits [22:6]
#define ENC_END_SHIFT          (6u) // Number of digits to drop at right to read from data after masking
#define ENC_TIMEOUT            (20u)
#define ENC_TICS_PER_MOTOR_REV (0x20000u) // Number of encoder tics in mechanical revolution (per datasheet)
#define GEARBOX_RATIO          (15u)      // Revs into gearbox per 1 revolution out

#define PACKET_LENGTH (3u)
#define PACKET_HEADER (0b10101010)
#define LAUNCH_CODE   (30881) // full launch packet in DEC is 11172001
#define ABORT_CODE    (43082) // full abort packet in DEC is 11184202

#define PWM_FREQ_COEF (1262) // 38kHz: 48MHz / (1262 + 1) = 38kHz
#define SPI_FREQ      (8000000l)
#define syncClock()                                                                                                         \
    while (GCLK->STATUS.bit.SYNCBUSY)                                                                                       \
        ; // Wait for clock synchronization

// Error codes, 2-15
#define ERR_CLR          (1u) // No error present
#define ERR_ENC_CONN     (2u) // Bad connection with encoder
#define ERR_ENC_INT      (3u) // Encoder passes internal error state
#define ERR_ENC_MISMATCH (4u) // Encoder reads do not match
#define ERR_BAD_PACKET   (4u) // Invalid or corrupt Serial1 data

// Status codes, 0-7
#define STATUS_INIT     (0u)
#define STATUS_CONN     (1u)
#define STATUS_HOME     (2u)
#define STATUS_WAIT     (3u)
#define STATUS_IDLE     (4u)
#define STATUS_ACTIVATE (5u)
#define STATUS_RUN      (6u)
#define STATUS_ABORT    (7u)

#define LED_GREEN          (SERCOM1)
#define LED_RED            (SERCOM0)
#define SERCOM_BAUD        (3u) // Baud value for SERCOM communications
#define FUEL_STATUS_OFFSET (5u)
#define OX_STATUS_OFFSET   (2u)
#define FUEL_ERR_OFFSET    (4u)
#define OX_ERR_OFFSET      (0u)

#define PORT_WRITE(p, b)                                                                                                    \
    (b == p.active ? PORT_IOBUS->Group[p.port].OUTSET.reg |= (1 << p.pin)                                                   \
                   : PORT_IOBUS->Group[p.port].OUTCLR.reg |= (1 << p.pin))
#define PORT_READ(p)    (int)(p.active == (!!(PORT_IOBUS->Group[p.port].IN.reg & (1 << p.pin))))
#define statusCode      ((Fuel.err << FUEL_STATUS_OFFSET) | (Ox.err << OX_STATUS_OFFSET)) // Merge errors/statuses for LED output
#define errorCode       ((Fuel.err << FUEL_ERR_OFFSET) | (Ox.err << OX_ERR_OFFSET))
#define neg(x, y)       (((x ^ y) + (y & 1))) // Negates x if y (bit) is 1, only works for 2s complement
#define homeShift(x, y) ((x - y.origin) + ((x < y.origin) & (1 << ENC_DATA_BIT_CT)))

const uint32_t ENC_TICS_PER_VALVE_REV = ((ENC_TICS_PER_MOTOR_REV) * (GEARBOX_RATIO)); // Encoder tics per valve revolution
const float TARGET_REVS = (((90 / 360) * GEARBOX_RATIO)); // Number of rotations to get fully open

const array<char *, 8> STATUS_NAME = {"STATUS_INIT", "STATUS_CONN",     "STATUS_HOME", "STATUS_WAIT",
                                      "STATUS_IDLE", "STATUS_ACTIVATE", "STATUS_RUN",  "STATUS_ABORT"};

const array<char *, 15> ERR_NAME = {
    "null", "ERR_CLR", "ERR_ENC_CONN", "ERR_ENC_INT", "ERR_ENC_MISMATCH", "ERR_BAD_PACKET", "", "", "", "", "", "", "",
    "",     ""

};

typedef struct {
    uint8_t port;
    uint8_t pin;
    bool active;
} Pin;

typedef struct {
    uint32_t origin;   // Encoder readings when valve completely closed.
    uint32_t prev_t;   // t_n-1, in ticks
    float target;      // PID setpoint, in revs
    float totalRevs;   // Current position in revs
    float accumulator; // PID integral term accumulator
    byte err;          // Error code
    byte status;       // Status code
    float P;
    float I;
    float D;
    Tc *TC;
    byte CCB;
    SERCOM *serc;
    Pin CTRL;
    Pin DIR_SEL;
    Pin STOP;
    Pin SCK;
    Pin MISO;
    Pin MOSI;
    Pin CS;
    SPIClassSAMD *COM;
} Encoder;

struct {
    Pin BUTTON_ONE = {(uint8_t)g_APinDescription[A6].ulPort, (uint8_t)g_APinDescription[A6].ulPin, ACTIVE_LOW};
    Pin BUTTON_TWO = {(uint8_t)g_APinDescription[A7].ulPort, (uint8_t)g_APinDescription[A7].ulPin, ACTIVE_LOW};
    Pin SEL_SWITCH = {(uint8_t)g_APinDescription[2].ulPort, (uint8_t)g_APinDescription[2].ulPin, ACTIVE_HIGH};
    Pin TX = {(uint8_t)g_APinDescription[0].ulPort, (uint8_t)g_APinDescription[0].ulPin};
    Pin RX = {(uint8_t)g_APinDescription[1].ulPort, (uint8_t)g_APinDescription[1].ulPin};
} EXT;

SPISettings spiSettings = SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE3);

Encoder Fuel, Ox;
Encoder *enc;

bool serEn = false;
// External Serial1 enable sensor pin.
Pin SER_EN = {(uint8_t)g_APinDescription[3].ulPort, (uint8_t)g_APinDescription[3].ulPin, ACTIVE_LOW};

void sPrintln(int, uint8_t);
void sPrint32(uint32_t, uint8_t);
void sPrintln32(uint32_t, uint8_t);
uint32_t readSPIs(SPIClassSAMD, bool);
uint32_t readSPIs(SPIClassSAMD);

// Configure GPIO pins per 23.6.3 of SAMD datasheet
void configureGPIO(Pin p, bool DIR, bool INEN = 0, bool PULLEN = 0, bool OUT = 0, bool SAMPLE = 0) {
    PORT->Group[p.port].DIR.reg |= DIR << p.pin;
    PORT->Group[p.port].PINCFG[p.pin].bit.INEN = INEN;
    PORT->Group[p.port].PINCFG[p.pin].bit.PULLEN = PULLEN;
    PORT->Group[p.port].OUT.reg |= OUT << p.pin;
    PORT->Group[p.port].CTRL.reg |= SAMPLE << p.pin;
}

void setup() {
    Serial1.begin(115200);
    Fuel = {0, 0, 3.5, 0, 0, ERR_CLR, STATUS_CONN, 1.0, .15, .05, TC4, 2, &sercom1};
    Fuel.CTRL = {(uint8_t)g_APinDescription[A3].ulPort, (uint8_t)g_APinDescription[A3].ulPin};
    Fuel.DIR_SEL = {(uint8_t)g_APinDescription[10].ulPort, (uint8_t)g_APinDescription[10].ulPin, ACTIVE_HIGH};
    Fuel.STOP = {(uint8_t)g_APinDescription[A1].ulPort, (uint8_t)g_APinDescription[A1].ulPin, ACTIVE_LOW};
    Fuel.SCK = {(uint8_t)g_APinDescription[13].ulPort, (uint8_t)g_APinDescription[13].ulPin};
    Fuel.MISO = {(uint8_t)g_APinDescription[12].ulPort, (uint8_t)g_APinDescription[12].ulPin};
    Fuel.MOSI = {(uint8_t)g_APinDescription[11].ulPort, (uint8_t)g_APinDescription[11].ulPin};
    Fuel.CS = {(uint8_t)g_APinDescription[8].ulPort, (uint8_t)g_APinDescription[8].ulPin, ACTIVE_LOW};
    Fuel.COM = new SPIClassSAMD(Fuel.serc, (uint8_t)12, (uint8_t)13, (uint8_t)11, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_3);

    Ox = {0, 0, 3.5, 0.0, 0.0, ERR_CLR, STATUS_CONN, 0.0, 0.0, 0.0, TC5, 3, &sercom0};
    Ox.CTRL = {(uint8_t)g_APinDescription[A2].ulPort, (uint8_t)g_APinDescription[A2].ulPin};
    Ox.DIR_SEL = {(uint8_t)g_APinDescription[9].ulPort, (uint8_t)g_APinDescription[9].ulPin, ACTIVE_HIGH};
    Ox.STOP = {(uint8_t)g_APinDescription[A0].ulPort, (uint8_t)g_APinDescription[A0].ulPin, ACTIVE_LOW};
    Ox.SCK = {(uint8_t)g_APinDescription[5].ulPort, (uint8_t)g_APinDescription[5].ulPin};
    Ox.MISO = {(uint8_t)g_APinDescription[4].ulPort, (uint8_t)g_APinDescription[4].ulPin};
    Ox.MOSI = {(uint8_t)g_APinDescription[6].ulPort, (uint8_t)g_APinDescription[6].ulPin};
    Ox.CS = {(uint8_t)g_APinDescription[7].ulPort, (uint8_t)g_APinDescription[7].ulPin, ACTIVE_LOW};
    Ox.COM = new SPIClassSAMD(Ox.serc, (uint8_t)4, (uint8_t)5, (uint8_t)6, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_3);

    configureGPIO(Fuel.DIR_SEL, OUTPUT);
    configureGPIO(Fuel.STOP, OUTPUT);
    configureGPIO(Fuel.CS, OUTPUT);

    configureGPIO(Ox.DIR_SEL, OUTPUT);
    configureGPIO(Ox.STOP, OUTPUT);
    configureGPIO(Ox.CS, OUTPUT);

    configureGPIO(EXT.BUTTON_ONE, INPUT, 1, 1, PULLDOWN, SAMPLING_ON);
    configureGPIO(EXT.BUTTON_TWO, INPUT, 1, 1, PULLDOWN, SAMPLING_ON);
    configureGPIO(EXT.SEL_SWITCH, INPUT, 1, 1, PULLDOWN, SAMPLING_ON);
    configureClocks();
    attachPins();

    TCC0->CCB[Fuel.CCB].reg = 0;
    while (TCC0->SYNCBUSY.bit.CCB2)
        ;
    TCC0->CCB[Ox.CCB].reg = 0;
    while (TCC0->SYNCBUSY.bit.CCB3)
        ;

    while (!Serial1.available()) {}

    sPrintln("starting");
    enc = (bool)Serial1.parseInt() ? &Ox : &Fuel;
    sPrint("Enc:");
    sPrintln((enc->CCB - 2) ? "OX" : "FUEL");
    initSPIs(&Fuel);
    initSPIs(&Ox);
    TCC0->CCB[Fuel.CCB].reg = .10 * PWM_FREQ_COEF;
    while (true) {
        if (Serial1.available()) {
            PORT_WRITE(Fuel.DIR_SEL, Serial1.parseInt() > 0 ? 0 : 1);
        }
    }
    // while (true) {
    //     if (Serial1.available()) {
    //         int ct = Serial1.parseInt();
    //         Serial1.println(ct > 0);
    //         uint16_t b = ((float)abs(ct) / 100) * PWM_FREQ_COEF;
    //         b = min(PWM_FREQ_COEF, b);
    //         TCC0->CCB[Fuel.CCB].reg = b;
    //         PORT_WRITE(Fuel.DIR_SEL, ct > 0);
    //         Serial1.print(ct);
    //         Serial1.print(" ");
    //         Serial1.println(b);
    //     }
    // }
}

void loop() {
    switch (enc->status) {
        case STATUS_CONN:

        {
            while (!PORT_READ(EXT.BUTTON_ONE)) {
                sPrint((enc->CCB - 2) ? "Ox: " : "FUEL: ");
                while (enc->TC->COUNT16.STATUS.bit.SYNCBUSY)
                    ;
                sPrint("dT: ");
                float dt = getDeltaTheta(enc);
                enc->totalRevs += dt;
                sPrint(dt);
                sPrint(";  Pos:");
                sPrintln(enc->totalRevs);
                delay(100);
            }

            enc->status = STATUS_HOME;
            break;
        }
        case STATUS_HOME: {
            while (!PORT_READ(EXT.SEL_SWITCH)) {
                if (PORT_READ(EXT.BUTTON_ONE)) {
                    PORT_WRITE(enc->STOP, false);
                    PORT_WRITE(enc->DIR_SEL, 1);
                    TCC0->CCB[enc->CCB].reg = .15 * PWM_FREQ_COEF;
                    sPrintln("MoveLeft");
                } else if (PORT_READ(EXT.BUTTON_TWO)) {
                    PORT_WRITE(enc->STOP, false);
                    PORT_WRITE(enc->DIR_SEL, 0);
                    TCC0->CCB[enc->CCB].reg = .15 * PWM_FREQ_COEF;
                    sPrintln("MoveRight");
                } else {
                    PORT_WRITE(enc->STOP, true);
                    TCC0->CCB[enc->CCB].reg = 0;
                    sPrintln("Stop");
                }
                sPrint(PORT_READ(EXT.SEL_SWITCH));
                sPrint(" ");
                sPrintln((int)digitalRead(2), DEC);

                delay(100);
            }
            PORT_WRITE(enc->STOP, true);
            if (PORT_READ(EXT.SEL_SWITCH)) {
                sPrintln("STOPPED. PRESS BOTH BUTTONS TO CONFIRM HOME POSITION");
                while (!(PORT_READ(EXT.BUTTON_ONE) && PORT_READ(EXT.BUTTON_TWO))) {
                    if (!PORT_READ(EXT.SEL_SWITCH)) {
                        return;
                    }
                }
                enc->target = TARGET_REVS;
                enc->status = STATUS_WAIT;
                break;
            }
            break;
        }
        case STATUS_WAIT: {
            PORT_WRITE(enc->STOP, true);
            if (((&Fuel)->status == STATUS_WAIT) && ((&Ox)->status == STATUS_WAIT)) {
                sPrintln("Set SEL_SWITCH to ON and press both buttons to arm valves");
                delay(200);
                while (!PORT_READ(EXT.SEL_SWITCH))
                    ;
                while (!(PORT_READ(EXT.BUTTON_ONE) && PORT_READ(EXT.BUTTON_TWO)))
                    ;
                (&Fuel)->status = STATUS_IDLE;
                (&Ox)->status = STATUS_IDLE;
                sPrintln("idle");
                delay(200);
            }
            break;
        }
        case STATUS_IDLE: {
            PORT_WRITE(enc->STOP, true);
            sPrintln("idle");
            if (serEn) {
                while (Serial1.available() < PACKET_LENGTH)
                    ;

                byte header = Serial1.read();
                uint16_t pData = ((Serial1.read() << 8) | Serial1.read());
                sPrintln(header, BIN);
                sPrintln(pData, BIN);

                if (header == PACKET_HEADER) {
                    if (pData == LAUNCH_CODE) {
                        (&Fuel)->status = STATUS_ACTIVATE;
                        (&Ox)->status = STATUS_ACTIVATE;
                    } else if (pData == ABORT_CODE) {
                        (&Fuel)->status = STATUS_ABORT;
                        (&Ox)->status = STATUS_ABORT;
                    } else {
                        enc->err = ERR_BAD_PACKET;
                        error(false);
                    }
                }
                // TODO implement Hamming

            } else {
                if (PORT_READ(EXT.TX)) {
                    (&Fuel)->status = STATUS_ACTIVATE;
                    (&Ox)->status = STATUS_ACTIVATE;
                } else if (!PORT_READ(EXT.SEL_SWITCH)) {
                    if (PORT_READ(EXT.BUTTON_ONE) || PORT_READ(EXT.BUTTON_TWO)) {
                        (&Fuel)->status = STATUS_ABORT;
                        (&Ox)->status = STATUS_ABORT;
                    }
                }
            }
        } break;
        case STATUS_ACTIVATE: {
            sPrintln("activated");
            enc->TC->COUNT16.CTRLA.bit.ENABLE = 1;
            enc->status = STATUS_RUN;
        } break;
        case STATUS_RUN: {
            PORT_WRITE(enc->STOP, false);
            enc->target = 3.5;
            update(enc);
        } break;
        case STATUS_ABORT: {
            PORT_WRITE(Fuel.STOP, false);
            PORT_WRITE(Ox.STOP, false);
            (&Fuel)->target = 0;
            (&Ox)->target = 0;
            update(&Fuel);
            update(&Ox);
        } break;

        default:
            break;
    }
    enc = ((enc->CCB - 2) ? &Fuel : &Ox);
    // sPrintln("Switch");
    // delay(200);
}

void sPrint(int i, uint8_t base = DEC) {
    Serial1.print(i, base);
    Serial1.flush();
}

void sPrintln(int i, uint8_t base = DEC) {
    sPrint(i, base);
    sPrintln();
}

void sPrintln32(uint32_t i, uint8_t base = DEC) {
    Serial1.println(i, base);
    Serial1.flush();
}
void sPrint32(uint32_t i, uint8_t base = DEC) {
    Serial1.print(i, base);
    Serial1.flush();
}

// void sPrint(int i, uint8_t base = DEC) { sPrint(i, base); }
// void sPrintln(int i, uint8_t base = DEC) { sPrintln(i, base); }

void sPrint(const char *str) {
    Serial1.print(str);
    Serial1.flush();
}

void sPrint(float f) {
    Serial1.print(f);
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

void sPrintln(float f) {
    sPrint(f);
    sPrintln();
}

void sPrintln(void) { sPrint("\r\n"); }
// PID loop. General implementation.
void update(Encoder *enc) {
    float dt = enc->TC->COUNT16.COUNT.reg; // Time since last loop, in us
    float theta_n = enc->totalRevs + getDeltaTheta(enc);
    sPrint((enc->CCB - 2) ? "OX" : "FUEL");
    sPrint("  dt: ");
    sPrint(dt);
    sPrint("  theta_n: ");
    sPrint(theta_n);
    sPrint("  TARGET: ");
    sPrint(enc->target);
    sPrint("  last term: ");
    sPrint((theta_n + enc->totalRevs - 2));

    // Integral approximation based on trapezoidal Riemann sum
    enc->accumulator += (dt / 2) * enc->target * (theta_n + enc->totalRevs - 2);
    sPrint("  dA: ");
    sPrint((dt / 2) * enc->target * (theta_n + enc->totalRevs - 2));
    sPrint("  accumulator: ");
    sPrint(enc->accumulator);

    // Calculate PID output
    float O =
        (enc->P * (theta_n - enc->target)) + (enc->I * enc->accumulator) + (enc->D * ((theta_n - enc->totalRevs) / dt));

    sPrint("  O: ");
    sPrintln(O);
    PORT_WRITE(enc->DIR_SEL, O < 0);

    TCC0->CCB[enc->CCB].reg = min(PWM_FREQ_COEF, abs(O));

    enc->totalRevs = theta_n;
}

// Returns change in encoder position since last call in revolutions. For use with PID loop, as this
// enables it to pick the most efficient route to the target position.
float getDeltaTheta(Encoder *enc) {
    uint32_t npos = readRawEncData(enc);

    enc->TC->COUNT16.CTRLBSET.reg |= TC_CTRLBSET_CMD_RETRIGGER;
    while (enc->TC->COUNT16.COUNT.reg < ENC_TIMEOUT)
        ;
    uint32_t nnpos = readRawEncData(enc);

    if (nnpos ^ npos == 0) {

        npos = (npos ^ ENC_DATA_MASK) >> ENC_END_SHIFT; // Extract value from raw bits
        float dif;
        dif = ((signed long)npos - (signed long)enc->prev_t);

        if (abs(dif) > ENC_TICS_PER_MOTOR_REV * .5) { // If the shortest path between npos and ppos crosses the 0 point (eg
                                                      // npos and ppos are 11 o clock and 1 o clock)
            if (dif > 0) { // If 0 point crossed in the negative direction (prev_t = 1 o clock, npos = 11 o clock)
                enc->prev_t = npos;
                dif = dif - ENC_TICS_PER_MOTOR_REV;
            } else { // If it crossed 0 point travelling in the positive direciton (prev_t = 11 o clock, npos = 1 o clock)
                enc->prev_t = npos;
                dif = ENC_TICS_PER_MOTOR_REV - dif;
            }
        }
        enc->prev_t = npos;
        return dif / ENC_TICS_PER_MOTOR_REV; // If the movement did not cross the 0 point, return straightforward difference
                                             // between npos and prev_t

        // if (abs(npos - ppos) < (1 << (ENC_DATA_BIT_CT - 2))) {
        //     sPrint(1);
        //     dif = ((signed long)npos - (signed long)ppos);
        // } else {
        //     sPrint(2);
        //     dif = ((signed long)npos - (signed long)ppos) + ((neg((1l << ENC_DATA_BIT_CT), (npos > ppos))));
        // }
        // return (float)(dif / (1 << ENC_DATA_BIT_CT - 1));
    } else {
        sPrint("npos: ");
        sPrintln32(npos, BIN);
        sPrint("nnpos: ");
        sPrintln32(nnpos, BIN);
        sPrintln32(npos ^ nnpos, BIN);
        sPrintln("err 353 !");
        enc->err = ERR_ENC_MISMATCH;
        error(false);
    }
}

// Takes reading from fuel encoder. Throws error if first latch bit is not 1, or encoder reading returns internal error state
// (last bit is 0).
uint32_t readRawEncData(Encoder *enc) {
    SPIClassSAMD spi = *(enc->COM);
    PORT_WRITE(enc->CS, true);
    spi.beginTransaction(SPISettings(200000, MSBFIRST, SERCOM_SPI_MODE_0));
    uint32_t data = 0;
    uint32_t buf;
    uint32_t d2 = 0;
    // uint16_t b1 = spi.transfer16(0b10101010);
    byte b1 = spi.transfer(0b10101010);
    byte b2 = spi.transfer(0b10101010);
    byte b3 = spi.transfer(0b10101010);
    // spi.endTransaction();
    // data |= b1 << 7;
    data |= b1 << 16;
    data |= b1 << 8;
    data |= b3 << 0;
    spi.endTransaction();
    PORT_WRITE(enc->CS, false);
    // sPrint("rawData:");
    // sPrintln(rawData, BIN);
    if (data & ENC_TICS_PER_MOTOR_REV) { // First bit is 1 = latch bit working, increased probability of good read
        // sPrintln("err 388 !");
        // enc->err = ERR_ENC_CONN;
        // // enc->SERCOM->SPI.DATA.reg = errorCode;
        // error(true);
    }
    if (!(data & 1l)) { // Last bit is 0 = enc internal error
        sPrint("data: ");
        sPrintln32(data, BIN);
        sPrint("error bit: ");
        sPrintln32(data & 1);
        enc->err = ERR_ENC_INT;
        sPrintln("err 395 !");
        error(true);
    }
    return data;
}

// Outputs error info to Serial1, if fatal error closes valves.
void error(bool fatal) {
    sPrintln();
    sPrintln("Error occured!");
    sPrint("isFatal: ");
    sPrintln(fatal);
    sPrint("Fuel.err: ");
    sPrintln(ERR_NAME[Fuel.err]);
    sPrint("Ox.err: ");
    sPrintln(ERR_NAME[Ox.err]);
    sPrint("Fuel.status: ");
    sPrintln(STATUS_NAME[Fuel.status]);
    sPrint("Ox.status: ");
    sPrintln(STATUS_NAME[Ox.status]);

    while (!SERCOM0->SPI.INTFLAG.bit.DRE)
        ;
    SERCOM0->SPI.DATA.reg = errorCode;

    if (fatal) {
        Fuel.status = STATUS_ABORT;
        Ox.status = STATUS_ABORT;
    }
}

// Checks to see if Serial1 communications port has been enabled (active low)
bool serialEnabled() {
    configureGPIO(SER_EN, INPUT, 1, 1, INPUT_PULLUP);
    return PORT_READ(SER_EN);
}

void initSPIs(Encoder *enc) {
    enc->serc->resetSPI();
    enc->COM->begin();
}

uint32_t readSPIs(SPIClassSAMD spi, bool verbose = false) {
    spi.beginTransaction(SPISettings(800000, MSBFIRST, SERCOM_SPI_MODE_0));
    uint32_t data = 0;
    uint32_t buf;
    uint32_t d2 = 0;
    // uint16_t b1 = spi.transfer16(0b10101010);
    byte b1 = spi.transfer(0b10101010);
    byte b2 = spi.transfer(0b10101010);
    byte b3 = spi.transfer(0b10101010);
    // spi.endTransaction();
    // data |= b1 << 7;
    data |= b1 << 16;
    data |= b1 << 8;
    data |= b3 << 0;
    spi.endTransaction();
    buf = data;
    if (verbose) {
        Serial1.print("OD: ");
        Serial1.print(data, BIN);
        char db[24];
        for (int i = 0; i < 24; i++) {
            if (buf % 2) {
                db[i] = '1';
            } else {
                db[i] = '0';
            }

            buf = buf >> 1;
        }
        Serial1.print(", DB_full: ");
        for (int i = 23; i >= 0; i--) {
            // Serial1.println();
            // Serial1.print(i);
            // Serial1.print(" : ");
            // Serial1.print(db[i]);
            Serial1.print(db[i]);
        }
        Serial1.print(", DB_act: ");
        for (int i = 16; i >= 0; i--) {
            Serial1.print(db[i + 5]);
            d2 |= (db[i + 5] == '1' ? 1 : 0) << i;
        }
        Serial1.print(", D2_2: ");
        Serial1.print(d2, BIN);
        Serial1.print(", D2_10:");
        Serial1.print(d2);
        Serial1.println();
    } else {
        buf = buf >> 5;
        for (int i = 0; i < 17; i++) {
            d2 |= (buf % 2) << i;
            buf = buf >> 1;
        }
    }

    return d2;
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
    PORT->Group[Fuel.CTRL.port].PINCFG[Fuel.CTRL.pin].bit.PMUXEN = 1;
    PORT->Group[Ox.CTRL.port].PINCFG[Ox.CTRL.port].bit.PMUXEN = 1;
    sPrintln("375");

    // Enable the port multiplexer for Encoder coms and status register pins
    // PORT->Group[Fuel.SCK.port].PINCFG[Fuel.SCK.pin].bit.PMUXEN = 1;
    // PORT->Group[Fuel.MISO.port].PINCFG[Fuel.MISO.pin].bit.PMUXEN = 1;
    // PORT->Group[Fuel.MOSI.port].PINCFG[Fuel.MOSI.pin].bit.PMUXEN = 1;
    // PORT->Group[Fuel.CS.port].PINCFG[Fuel.CS.pin].bit.PMUXEN = 1;
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
    PORT->Group[Ox.CTRL.port].PINCFG[Ox.CTRL.pin].bit.PMUXEN = 1;
    PORT->Group[Ox.CTRL.port].PINCFG[Ox.CTRL.port].bit.PMUXEN = 1;
    sPrintln("396");

    // Enable the port multiplexer for Encoder coms and status register pins
    // PORT->Group[Ox.SCK.port].PINCFG[Ox.SCK.pin].bit.PMUXEN = 1;
    // PORT->Group[Ox.MISO.port].PINCFG[Ox.MISO.pin].bit.PMUXEN = 1;
    // PORT->Group[Ox.MOSI.port].PINCFG[Ox.MOSI.pin].bit.PMUXEN = 1;
    // PORT->Group[Ox.CS.port].PINCFG[Ox.CS.pin].bit.PMUXEN = 1;
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
    PORT->Group[Fuel.CTRL.port].PMUX[Fuel.CTRL.pin >> 1].reg |=
        (!(Fuel.CTRL.pin % 2)) ? PORT_PMUX_PMUXE_F : PORT_PMUX_PMUXO_F;
    PORT->Group[Ox.CTRL.port].PMUX[Ox.CTRL.pin >> 1].reg |= (!(Ox.CTRL.pin % 2)) ? PORT_PMUX_PMUXE_F : PORT_PMUX_PMUXO_F;
    sPrintln("428");
    /*
    // Attach Encoder coms and status register pins to FUEL SERCOM
    PORT->Group[Fuel.SCK.port].PMUX[Fuel.SCK.pin >> 1].reg |=
        ((Fuel.SCK.pin % 2) == 0) ? PORT_PMUX_PMUXE_C : PORT_PMUX_PMUXO_C;
    PORT->Group[Fuel.MISO.port].PMUX[Fuel.MISO.pin >> 1].reg |=
        ((Fuel.MISO.pin % 2) == 0) ? PORT_PMUX_PMUXE_C : PORT_PMUX_PMUXO_C;
    PORT->Group[Fuel.MOSI.port].PMUX[Fuel.MOSI.pin >> 1].reg |=
        ((Fuel.MOSI.pin % 2) == 0) ? PORT_PMUX_PMUXE_C : PORT_PMUX_PMUXO_C;
    PORT->Group[Fuel.CS.port].PMUX[Fuel.CS.pin >> 1].reg |= ((Fuel.CS.pin % 2) == 0) ? PORT_PMUX_PMUXE_C : PORT_PMUX_PMUXO_C;
    sPrintln("438");

    // // Attach Encoder coms and status register pins to OX SERCOM
    PORT->Group[Ox.SCK.port].PMUX[Ox.SCK.pin >> 1].reg |=
        ((Ox.SCK.pin % 2) == 0) ? PORT_PMUX_PMUXE_C : PORT_PMUX_PMUXO_C;
    PORT->Group[Ox.MISO.port].PMUX[Ox.MISO.pin >> 1].reg |=
        ((Ox.MISO.pin % 2) == 0) ? PORT_PMUX_PMUXE_C : PORT_PMUX_PMUXO_C;
    PORT->Group[Ox.MOSI.port].PMUX[Ox.MOSI.pin >> 1].reg |=
        ((Ox.MOSI.pin % 2) == 0) ? PORT_PMUX_PMUXE_C : PORT_PMUX_PMUXO_C;
    PORT->Group[Ox.CS.port].PMUX[Ox.CS.pin >> 1].reg |= ((Ox.CS.pin % 2) == 0) ? PORT_PMUX_PMUXE_C : PORT_PMUX_PMUXO_C;
    sPrintln("448");
    */
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
    TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;
    while (TCC0->SYNCBUSY.bit.WAVE)
        ; // Sync

    TCC0->PER.reg = PWM_FREQ_COEF; // Set PWM frequency to 50KHz (f=GCLK/(2+PER))
    while (TCC0->SYNCBUSY.bit.PER)
        ; // Sync

    TCC0->CTRLA.bit.ENABLE = 1; // Enable the TCC0 counter
    while (TCC0->SYNCBUSY.bit.ENABLE)
        ; // Sync

    sPrintln("485");

    // Much of the TCC information applies with TCs as well. The biggest difference is that TCs lack a double buffered
    // input, and instead only have CCx. This is fine as we are only using TC4 and TC5 as timers for the PID controllers,
    // and do not need any waveform generation from them. The counter increments from 0 or decrements from the max value
    // depending on configuration, and COUNT register is compared to the value in the Compare/Capture register (CCx).
    // Every time the count hits CCx, it resets to 0/max and triggers an event. See Chapter 30 for more details. TC4 and
    // TC5 are configured identically, as we are using them to run the FUEL and OX PID controllers respectively.

    //
    TC4->COUNT16.CTRLA.reg =
        TC_CTRLA_MODE_COUNT16 |   // Select 16 bit mode for TC4
        TC_CTRLA_PRESCALER_DIV8 | // Divides frequency by 8, to get f_TC of 1MHz, or period=1us
        TC_CTRLA_PRESCSYNC_PRESC; // Clock reset is connected to prescaler clock not GCLK. Keeps 1us period.

    while (TC4->COUNT16.STATUS.bit.SYNCBUSY)
        ; // sync

    // Sets continuous read synchronization of count register. This means it is not neccesary to set the RREQ flag and
    // wait to synchronize every time we want to read the COUNT value.See section 30.10.2 in MC datasheet.
    TC4->COUNT16.READREQ.reg = TC_READREQ_RCONT | // Sets RCONT flag, disabling automatic clearing of RREQ flag after sync
                               TC_READREQ_RREQ |  // Read sync request flag, synchronizes COUNT register for reading
                               TC_READREQ_ADDR(0x10); // COUNT register address

    TC4->COUNT16.CTRLA.bit.ENABLE = 1;
    while (TC4->COUNT16.STATUS.bit.SYNCBUSY)
        ; // sync

    sPrintln("509");
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
    TC5->COUNT16.CTRLA.bit.ENABLE = 1;
    while (TC5->COUNT16.STATUS.bit.SYNCBUSY)
        ; // sync

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
    /*
    SERCOM0->SPI.CTRLA.bit.ENABLE = 0;
    while (SERCOM0->SPI.SYNCBUSY.bit.ENABLE)
        ;
    // SERCOM0->SPI.CTRLA.bit.SWRST = 1;
    // while (SERCOM0->SPI.SYNCBUSY.bit.SWRST)
    //     ;
    SERCOM0->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_DIPO(0x3u) |                   // MISO is Pad 3
                             SERCOM_SPI_CTRLA_DOPO(0x0u) |                   // MOSI pad 0, SCK Pad 1, CS Pad 2
                             SERCOM_SPI_CTRLA_MODE_SPI_MASTER |              // Sets device mode as host
                             SERCOM_SPI_CTRLA_CPOL |                         // Clock high when idle
                             SERCOM_SPI_CTRLA_CPHA;                          // Sample data on rising edge
    SERCOM0->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_RXEN | SERCOM_SPI_CTRLB_MSSEN; // Enable reciever/full-duplex operation
    while (SERCOM0->SPI.SYNCBUSY.bit.CTRLB)
        ;
    SERCOM0->SPI.BAUD.reg = SERCOM_SPI_BAUD_BAUD(SERCOM_BAUD); // Sets baudrate to 8MHz/(2*([BAUD=3]+1)=1MHz
    SERCOM0->SPI.CTRLA.bit.ENABLE = 1;
    while (SERCOM0->SPI.SYNCBUSY.bit.ENABLE)
        ;

    sPrintln(1);
    SERCOM1->SPI.CTRLA.bit.ENABLE = 0;
    while (SERCOM1->SPI.SYNCBUSY.bit.ENABLE)
        ;
    // SERCOM0->SPI.CTRLA.bit.SWRST = 1;
    // while (SERCOM0->SPI.SYNCBUSY.bit.SWRST)
    //     ;
    sPrintln(2);
    SERCOM1->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_DIPO(0x3u) |                   // MISO is Pad 3
                             SERCOM_SPI_CTRLA_DOPO(0x0u) |                   // MOSI pad 0, SCK Pad 1, CS Pad 2
                             SERCOM_SPI_CTRLA_MODE_SPI_MASTER |              // Sets device mode as host
                             SERCOM_SPI_CTRLA_CPOL |                         // Clock high when idle
                             SERCOM_SPI_CTRLA_CPHA;                          // Sample data on rising edge
    SERCOM1->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_RXEN | SERCOM_SPI_CTRLB_MSSEN; // Enable reciever/full-duplex operation

    sPrintln(3);
    while (SERCOM1->SPI.SYNCBUSY.bit.CTRLB)
        ;

    sPrintln(4);
    SERCOM1->SPI.BAUD.reg = SERCOM_SPI_BAUD_BAUD(SERCOM_BAUD); // Sets baudrate to 8MHz/(2*([BAUD=3]+1)=1MHz
    sPrintln(5);
    SERCOM1->SPI.CTRLA.bit.ENABLE = 1;

    sPrintln(6);
    while (SERCOM1->SPI.SYNCBUSY.bit.ENABLE)
        ;
    */
    sPrintln(7);

    // serEn = serialEnabled();
    serEn = true;
    if (serEn) {
        // Serial1.begin(115200);
        // while (!Serial1)
        //     ;
    } else {

        sPrintln("613");
        // Disable Serial1 USART
        SERCOM5->SPI.CTRLA.bit.ENABLE = 0;
        while (SERCOM1->SPI.SYNCBUSY.bit.ENABLE)
            ;
        SERCOM5->SPI.CTRLA.bit.SWRST = 1;
        while (SERCOM5->SPI.SYNCBUSY.bit.SWRST)
            ;
        // Configure TX as plain input
        configureGPIO(EXT.TX, INPUT, 1, 1, PULLDOWN, SAMPLING_ON);

        sPrintln("624");
    }

    sPrintln("637");
}

// TODO: Documentation
void configureClocks() {

    PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0 | PM_APBCMASK_SERCOM1 | PM_APBCMASK_SERCOM2 | PM_APBCMASK_SERCOM3 |
                        PM_APBCMASK_SERCOM4 | PM_APBCMASK_SERCOM5 | PM_APBCMASK_TCC0 | PM_APBCMASK_TC4 | PM_APBCMASK_TC5;

    // Link timer periphs to GCLK for PWM
    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(4u) |     // Edit GCLK4
                        GCLK_GENCTRL_IDC |        // Improve 50/50 PWM
                        GCLK_GENCTRL_GENEN |      // Enable generic clock
                        GCLK_GENCTRL_SRC_DFLL48M; // Link to 48MHz source
    syncClock();

    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_GEN_GCLK4 |   // Select GCLK4 at 48MHz to edit |
                        GCLK_CLKCTRL_CLKEN |       // Enable GCLK4 as a clock source
                        GCLK_CLKCTRL_ID_TCC0_TCC1; // Feed GCLK4 to TCC0 and TCC0
    syncClock();

    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_GEN_GCLK4 | // Select GCLK4 at 48MHz to edit |
                        GCLK_CLKCTRL_CLKEN |     // Enable GCLK4 as a clock source
                        GCLK_CLKCTRL_ID_EIC;     // Feed GCLK4 to EIC
    syncClock();

    // Link timer periphs for PID loops.
    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(5u) |   // Edit GCLK5
                        GCLK_GENCTRL_IDC |      // Improve 50/50 PWM
                        GCLK_GENCTRL_GENEN |    // Enable generic clock
                        GCLK_GENCTRL_SRC_OSC8M; // Link to 8MHz source
    syncClock();

    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_GEN_GCLK5 | // Select GCLK5 at 8MHz to edit |
                        GCLK_CLKCTRL_CLKEN |     // Enable GCLK5 as a clock source
                        GCLK_CLKCTRL_ID_TC4_TC5; // Feed GCLK5 to TC4 and TC5

    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_GEN_GCLK5 | // Select GCLK5 at 8MHz to edit |
                        GCLK_CLKCTRL_CLKEN |     // Enable GCLK5 as a clock source
                        GCLK_CLKCTRL_ID_TC6_TC7; // Feed GCLK5 to TC4 and TC5

    // GCLK->CLKCTRL.reg = GCLK_CLKCTRL_GEN_GCLK5 |      // Select GCLK5 at 8MHz to edit |
    //                     GCLK_CLKCTRL_CLKEN |          // Enable GCLK5 as a clock source
    //                     GCLK_CLKCTRL_ID_SERCOM0_CORE; // Feed GCLK5 to SERCOM0 Baud Generator
    // syncClock();
    // GCLK->CLKCTRL.reg = GCLK_CLKCTRL_GEN_GCLK5 |      // Select GCLK5 at 8MHz to edit |
    //                     GCLK_CLKCTRL_CLKEN |          // Enable GCLK5 as a clock source
    //                     GCLK_CLKCTRL_ID_SERCOM1_CORE; // Feed GCLK7 to SERCOM1 Baud Generator
    syncClock();
}