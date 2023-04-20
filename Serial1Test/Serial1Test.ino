#include <SPI.h>
const int oCS = 7;
const int oMOSI = 6;
const int oSCK = 5;
const int oMISO = 4;
const int fCS = 8;
const int fMOSI = 11;
const int fSCK = 13;
const int fMISO = 12;
#define PIN_A A3
#define PIN_B 7
#define FREQ  1262

SERCOM *os = &sercom0;
SERCOM *fs = &sercom1;
SPIClassSAMD ox = SPIClassSAMD(os, oMISO, oSCK, oMOSI, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_3);
SPIClassSAMD fuel = SPIClassSAMD(fs, fMISO, fSCK, fMOSI, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_3);
SPIClassSAMD mySPI = fuel;
int CS = fCS;
void setup() {
    Serial1.begin(115200);
    pinMode(CS, OUTPUT);
    // pinMode(A3, OUTPUT);
    // pinMode(A2, OUTPUT);
    // digitalWrite(A2, LOW);
    // digitalWrite(A3, LOW);
    // pinMode(oMISO, INPUT);
    // while (true) {
    //     Serial1.println(digitalRead(oMISO));
    //     Serial1.flush();
    // }
    sercom0.resetSPI();
    mySPI.begin();
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |       // Enable GCLK0 as a clock source
                        GCLK_CLKCTRL_GEN_GCLK0 |   // Select GCLK0 at 48MHz
                        GCLK_CLKCTRL_ID_TCC0_TCC1; // Feed GCLK0 to TCC0 and TCC1
    while (GCLK->STATUS.bit.SYNCBUSY)
        ; // Wait for synchronization

    // Enable the port multiplexer for pins D7
    PORT->Group[g_APinDescription[PIN_A].ulPort].PINCFG[g_APinDescription[PIN_A].ulPin].bit.PMUXEN = 1;
    PORT->Group[g_APinDescription[PIN_B].ulPort].PINCFG[g_APinDescription[PIN_B].ulPin].bit.PMUXEN = 1;

    // D7 is on EVEN port pin PA06 and TCC1/WO[0] channel 0 is on peripheral E
    PORT->Group[g_APinDescription[PIN_A].ulPort].PMUX[g_APinDescription[PIN_A].ulPin >> 1].reg =
        ((g_APinDescription[PIN_A].ulPin % 2) == 0) ? PORT_PMUX_PMUXE_F : PORT_PMUX_PMUXO_F;
    PORT->Group[g_APinDescription[PIN_B].ulPort].PMUX[g_APinDescription[PIN_B].ulPin >> 1].reg =
        ((g_APinDescription[PIN_B].ulPin % 2) == 0) ? PORT_PMUX_PMUXE_E : PORT_PMUX_PMUXO_E;

    // Normal (single slope) PWM operation: timer countinuouslys count up to PER register value and then is reset to 0
    TCC0->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM; // Setup single slope PWM on TCC1
    while (TCC0->SYNCBUSY.bit.WAVE)
        ;                                    // Wait for synchronization
    TCC1->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM; // Setup single slope PWM on TCC1
    while (TCC1->SYNCBUSY.bit.WAVE)
        ; // Wait for synchronization

    TCC0->PER.reg = FREQ; // Set the frequency of the PWM on TCC1 to 38kHz: 48MHz / (1262 + 1) = 38kHz
    while (TCC0->SYNCBUSY.bit.PER)
        ;                 // Wait for synchronization
    TCC1->PER.reg = FREQ; // Set the frequency of the PWM on TCC1 to 38kHz: 48MHz / (1262 + 1) = 38kHz
    while (TCC1->SYNCBUSY.bit.PER)
        ; // Wait for synchronization

    // TCC0->CC[0].reg = .75 * FREQ; // TCC0 CC0 - 50% duty cycle on D7
    // while (TCC0->SYNCBUSY.bit.CC0)
    //     ;                         // Wait for synchronization
    // TCC1->CC[0].reg = .75 * FREQ; // TCC0 CC0 - 50% duty cycle on D7
    // while (TCC1->SYNCBUSY.bit.CC0)
    ; // Wait for synchronization

    TCC0->CTRLA.bit.ENABLE = 1; // Enable the TCC1 counter
    while (TCC0->SYNCBUSY.bit.ENABLE)
        ;
    TCC1->CTRLA.bit.ENABLE = 1; // Enable the TCC1 counter
    while (TCC1->SYNCBUSY.bit.ENABLE)
        ; // Wait for synchronization

    TCC0->CCB[2].reg = .05 * FREQ; // TCC1 CCB1 - 25% duty cycle on D7
    while (TCC0->SYNCBUSY.bit.CCB0)
        ;
}

void loop() {
    delay(10);
    digitalWrite(CS, LOW);
    mySPI.beginTransaction(SPISettings(400000, MSBFIRST, SERCOM_SPI_MODE_1));
    uint32_t data = 0;
    uint32_t buf;
    // uint16_t b1 = mySPI.transfer16(0b10101010);
    byte b1 = mySPI.transfer(0b10101010);
    byte b2 = mySPI.transfer(0b10101010);
    byte b3 = mySPI.transfer(0b10101010);
    // mySPI.endTransaction();
    // data |= b1 << 7;
    data |= b1 << 16;
    data |= b1 << 8;
    data |= b3 << 0;
    mySPI.endTransaction();
    digitalWrite(CS, HIGH);
    Serial1.print("OD: ");
    Serial1.print(data, BIN);
    buf = data;
    char db[24];
    for (int i = 0; i < 24; i++) {
        if (buf % 2) {
            db[i] = '1';
        } else {
            db[i] = '0';
        }

        buf = buf >> 1;
    }
    uint32_t d2 = 0;
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
}
