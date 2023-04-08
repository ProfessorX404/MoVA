int incomingByte = 0; // for incoming serial data

void setup() {
    Serial.begin(115200); // opens serial port, sets data rate to 9600 bps
    Serial1.begin(115200);
}

void loop() {
    // send data only when you receive data:
    if (Serial.available() > 0) {
        // read the incoming byte:
        incomingByte = Serial.read();

        // say what you got:
        Serial.print("S0 received: ");
        Serial.println((char)incomingByte);
        Serial.flush();
        Serial1.print("S0 received: ");
        Serial1.println((char)incomingByte);
        Serial1.flush();
    }
    if (Serial1.available() > 0) {
        // read the incoming byte:
        incomingByte = Serial1.read();

        Serial.print("S1 received: ");
        Serial.println((char)incomingByte);
        Serial.flush();
        Serial1.print("S1 received: ");
        Serial1.println((char)incomingByte);
        Serial1.flush();
    }
}