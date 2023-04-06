void setup() {
    Serial.begin(9600);
    Serial1.begin(9600);
    delay(200);
    while (!Serial.available())
        ;
    bool led = false;
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.println("here1");
    while (!Serial1.available()) {
        Serial1.print(":(");
        delay(500);
        led = !led;
        digitalWrite(LED_BUILTIN, led);
    }
    Serial.println(Serial1.readString());

    Serial.println("done!");
    char message[] = "It works!";
    Serial1.println(message);
    Serial1.flush();
}

void loop() {
    Serial1.println("Is it working yet?");
    Serial1.flush();
}