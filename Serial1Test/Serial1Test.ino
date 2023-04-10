#include <SPI.h>
SPIClassSAMD mySPI = SPIClassSAMD(&sercom1, (uint8_t)12, (uint8_t)13, (uint8_t)11, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_3);
void setup() {
    Serial1.begin(115200);
    pinMode(8, OUTPUT);
    // pinMode(A3, OUTPUT);
    // pinMode(A2, OUTPUT);
    // digitalWrite(A2, LOW);
    // digitalWrite(A3, LOW);
    mySPI.begin();
    Serial1.print("?");

    // mySPI.setDataMode(SERCOM_SPI_MODE_3);
}

void loop() {
    delay(100);
    digitalWrite(8, LOW);
    mySPI.beginTransaction(SPISettings(1000000, MSBFIRST, SERCOM_SPI_MODE_0));
    uint32_t data = 0;
    uint32_t buf;
    uint16_t b1 = mySPI.transfer16(0b10101010);
    // mySPI.endTransaction();
    // mySPI.beginTransaction(SPISettings(SPISettings().getClockFreq(), MSBFIRST, SERCOM_SPI_MODE_3));
    byte b3 = mySPI.transfer(0b10101010);
    mySPI.endTransaction();
    data |= b1 << 7;
    // Serial1.print(b1, BIN);
    // Serial1.print("    ");
    // Serial1.flush();
    // byte b2 = mySPI.transfer(0b10101010);
    // data |= b2 << 8;
    // Serial1.print(b2, BIN);
    // Serial1.print("    ");
    // Serial1.flush();
    data |= b3 << 0;
    // Serial1.print(b3, BIN);
    // Serial1.print("    ");
    // Serial1.flush();
    // data |= mySPI.transfer(0b10101010) << 15;
    // data |= mySPI.transfer(0b10101010) << 7;
    // data |= mySPI.transfer(0b10101010) << 0;
    mySPI.endTransaction();
    digitalWrite(8, HIGH);
    Serial1.print("OD: ");
    Serial1.print(data, BIN);
    // Serial1.print(", ");
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
    // Serial1.print(", ");
    // Serial1.print(db);
    Serial1.print(", DB_act: ");
    for (int i = 16; i >= 0; i--) {
        Serial1.print(db[i + 5]);
        d2 |= (db[i + 5] == '1' ? 1 : 0) << i;
    }
    Serial1.print(", D2_2: ");
    Serial1.print(d2, BIN);
    Serial1.print(", D2_10:");
    Serial1.print(d2);
    // Serial1.print(d2 == data ? ", true" : ", false");
    Serial1.println();
}
