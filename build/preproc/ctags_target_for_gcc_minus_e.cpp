# 1 "C:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
# 2 "C:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino" 2
# 3 "C:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino" 2

# 3 "C:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
// Encoder used is Nanotec NME2-SSI-V06-12-C
# 21 "C:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
// Uses built in routine to skip clock cycles for timing purposes.

double output = 0; // Signed PID output from -255 to 255.
double k_pid[3] = {0.0, 0.0, 0.0}; // PID constants, in format [kP, kI, kD]
double target = 0.0 /* Encoder value for valve being fully closed*/; // Current valve position target. Init'ed to closed
double pos = 0.0; // Current valve position
int errorCode = 0; // Global error code variable for fault tracking.

/* TODO:

-Homing routine

-Change placeholder values to VALVE_OPEN and VALVE_CLOSED

- TUNE PID AND WIND UP CONSTANTS

- Work out how to count revolutions

*/
# 35 "C:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
ArduPID pid; // PID instance

void setup() {
    Serial.begin(57600);

    pinMode(3 /* Motor control pin*/, 0x1);
    pinMode(5 /* Directional control pin*/, 0x1);
    pinMode(5 /* Encoder clock PWM pin*/, 0x1);
    pinMode(6 /* Encoder data input pin*/, 0x0);
    analogWrite(3 /* Motor control pin*/, 0);
    digitalWrite(5 /* Directional control pin*/, 1 /* Normalized forward vector. Swap to 0 if reversed.*/);

    updateValvePos();
    pid.begin(&pos, &output, &target, k_pid[0], k_pid[1], k_pid[2]);
    pid.setSampleTime(ENC_MIN_TIME_MS);
    pid.setOutputLimits(-255, 255); // Motor can be actuated from 0-255 in either direction
    pid.setWindUpLimits(-10.0 /* Integral growth bound min const*/, 10.0 /* Integral growth bound max const*/); // Growth bounds for the integral term to prevent integral wind-up

    pid.start();
}

void loop() {

    if (updateValvePos()) {
        pid.stop();
    } else {
        pid.start();
    }
    pid.compute();
    updateEngineSpeed();

    /*

    if (Serial.available() > 0) {

        String input = Serial.readString();

        input.trim();

        if (input.substring(0, 1) == "!") {

            switch ((char)input.substring(1, 2)) {

                case 'P':

                    p = input.substring(2).toFloat();

                    Serial.print("Set P to: ");

                    Serial.println(p);

                    break;

                case 'I':

                    i = input.substring(2).toFloat();

                    Serial.print("Set I to: ");

                    Serial.println(I);

                    break;

                case 'D':

                    d = input.substring(2).toFloat();

                    Serial.print("Set D to: ");

                    Serial.println(d);

                    break;

            }

            pid.setCoefficients(p, i, d);

        } else if (input.substring(0, 1) == "%") {

            float incoming = parseCommand(input.substring(1).toFloat());

            byte newDir = (incoming >= 0) ? C_FORWARD : C_REVERSE;

            byte newVal = (byte)abs((255 * incoming));

            Serial.print(">Set duty cycle to " + (String)(incoming * 100.0) + "% or ");

            Serial.println((newDir == C_FORWARD ? "" : "-") + (String)newVal + "/255");

            updateEngineSpeed(newVal, newDir);

        } else {

            int incoming = input.toInt();

            byte newDir = (incoming >= 0) ? C_FORWARD : C_REVERSE;

            byte newVal = (byte)abs(incoming);

            Serial.println(">Set duty cycle to " + (String)(newDir == C_FORWARD ? "" : "-") + (String)(newVal) +

                           "/255");

            updateEngineSpeed(newVal, newDir);

        }

    }

    */
# 106 "C:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
}

void updateEngineSpeed() {
    digitalWrite(5 /* Directional control pin*/, output > 0 ? 1 /* Normalized forward vector. Swap to 0 if reversed.*/ : ((1 /* Normalized forward vector. Swap to 0 if reversed.*/ - 1)>0?(1 /* Normalized forward vector. Swap to 0 if reversed.*/ - 1):-(1 /* Normalized forward vector. Swap to 0 if reversed.*/ - 1)) /* Normalized reverse vector. Always opposite of C_FORWARD.*/);
    analogWrite(3 /* Motor control pin*/, ((output)>0?(output):-(output)));
}
/*

float parseCommand(float input) {

    if (abs(input) > 100) {

        while (abs(input) > 1) {

            input = input / 10;

        }

    } else {

        input = input / 100;

    }

    return input;

}

*/
# 125 "C:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
bool updateValvePos() {
    unsigned long sample1 = readEncData();
    unsigned long sample2 = readEncData();

    if ((sample1 != sample2)) {
        errorCode = 1;
        error(false);
        return true;
    }
    pos = sample1;
    return false;
}

unsigned long readEncData() {
    
# 139 "C:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino" 3
   __asm__ __volatile__ ("cli" ::: "memory")
# 139 "C:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
                 ;
    digitalWrite(5 /* Encoder clock PWM pin*/, 0x0); // First bit is latch, always 1.
    delayMicroseconds(1);
    digitalWrite(5 /* Encoder clock PWM pin*/, 0x1);
    delayMicroseconds(1);
    if (!digitalRead(6 /* Encoder data input pin*/)) {
        errorCode = 2;
        error(false);
        return -1.0;
    }

    unsigned long data = 0; // If latch reads successfully, continue to data bits
    for (int i = 0; i < 24 /* Total number of bits in encoder packet. Last bit is error bit, success=1.*/; i++) {
        data <<= 1;
        digitalWrite(5 /* Encoder clock PWM pin*/, 0x0);
        delayMicroseconds(1);
        digitalWrite(5 /* Encoder clock PWM pin*/, 0x1);
        delayMicroseconds(1);

        data |= digitalRead(6 /* Encoder data input pin*/);
    }

    if (!(data & ~(~0U << 1))) { // If last error bit is 0, internal encoder error occured
        errorCode = 3;
        error(true);
    }
    delayMicroseconds(ENC_MIN_TIME_MS * 10);
    
# 166 "C:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino" 3
   __asm__ __volatile__ ("sei" ::: "memory")
# 166 "C:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
               ;
    return data >> 24 /* Total number of bits in encoder packet. Last bit is error bit, success=1.*/ - 17 /* Data bits in encoder packet.*/;
}

void error(bool isFatal) {
    Serial.println("Error occured!");
    Serial.println("isFatal: " + isFatal);
    Serial.println("errorCode: " + errorCode);

    while (isFatal) {};
}
