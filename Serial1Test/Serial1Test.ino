#include <SPI.h>
const int oCS = 7;
const int oMOSI = 6;
const int oSCK = 5;
const int oMISO = 4;
const int fCS = 8;
const int fMOSI = 11;
const int fSCK = 13;
const int fMISO = 12;
SERCOM *os = &sercom0;
SERCOM *fs = &sercom1;
SPIClassSAMD ox = SPIClassSAMD(os, oMISO, oSCK, oMOSI, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_3);
SPIClassSAMD fuel = SPIClassSAMD(fs, fMISO, fSCK, fMOSI, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_3);
SPIClassSAMD mySPI = ox;
int CS = oCS;
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
}

void loop() {
    delay(100);
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
