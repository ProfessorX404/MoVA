void setup() {
    Serial.begin(9600);
    Serial1.begin(9600);
    delay(200);
}

void loop() {
    if (Serial.available()) {
        String msg = Serial.readString();
        Serial.println(msg);
        Serial1.println(msg);
    }
    if (Serial1.available()) {
        String msg = Serial1.readString();
        Serial.println(msg);
        Serial1.println(msg);
    }
}