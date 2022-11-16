# 1 "C:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
# 2 "C:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino" 2
// Encoder used is Nanotec NME2-SSI-V06-12-C
// URL: https://us.nanotec.com/products/8483-nme2-ssi-v06-12-c
// Motor used is Nanotec DB59l024035-A
// URL: https://us.nanotec.com/products/2870-db59l024035-a
# 26 "C:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
// Uses built in routine to skip clock cycles for timing purposes.

const double HALL_COMBOS[6][3] = // Combinations of hall sensors based on motor angle
    {
  //    H1,H2,H3}
        {1, 0, 1}, //  0->45
        {0, 0, 1}, //  45->90
        {0, 1, 1}, //  90->135
        {0, 1, 0}, //  135->180
        {1, 1, 0}, //  180->225
        {1, 0, 0} //  225->270
};

double output = 0; // Signed PID output from -255 to 255.
double k_pid[3] = {0.0, 0.0, 0.0}; // PID constants, in format [kP, kI, kD]
double target = 0.0 /* Encoder value for valve being fully closed*/ * 0x20000 /* Number of encoder tics in mechanical revolution (per datasheet)*/ * 15 /* Revs into gearbox per 1 revolution out*/ /* Post-gearbox encoder tics per valve revolution*/ / 360 /* Post-gearbox encoder tics / degree*/; // Current valve position target. Init'ed to closed
double pos = 0.0; // Current valve position
int errorCode = 0; // Global error code variable for fault tracking.

/* TODO:

- Homing routine

- Change placeholder values to VALVE_OPEN and VALVE_CLOSED

- TUNE PID AND WIND UP CONSTANTS

- Count revolutions

*/
# 52 "C:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
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
    pid.setOutputLimits(-255, 255); // Motor can be actuated from 0-255 in either direction
    pid.setWindUpLimits(-10.0 /* Integral growth bound min const*/, 10.0 /* Integral growth bound max const*/); // Growth bounds to prevent integral wind-up

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
}

void updateEngineSpeed() {
    digitalWrite(5 /* Directional control pin*/, output > 0 ? 1 /* Normalized forward vector. Swap to 0 if reversed.*/ : ((1 /* Normalized forward vector. Swap to 0 if reversed.*/ - 1)>0?(1 /* Normalized forward vector. Swap to 0 if reversed.*/ - 1):-(1 /* Normalized forward vector. Swap to 0 if reversed.*/ - 1)) /* Normalized reverse vector. Always opposite of C_FORWARD.*/);
    analogWrite(3 /* Motor control pin*/, ((output)>0?(output):-(output)));
}

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
    
# 102 "C:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino" 3
   __asm__ __volatile__ ("cli" ::: "memory")
# 102 "C:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
                 ;
    digitalWrite(5 /* Encoder clock PWM pin*/, 0x0); // First bit is latch, always 1.
    __builtin_avr_delay_cycles(1 * 0x10 /* HARDWARE DEPENDENT!!! For accurate data reading timings. Eq. to Clocks/us*/);
    digitalWrite(5 /* Encoder clock PWM pin*/, 0x1);
    __builtin_avr_delay_cycles(1 * 0x10 /* HARDWARE DEPENDENT!!! For accurate data reading timings. Eq. to Clocks/us*/);
    if (!digitalRead(6 /* Encoder data input pin*/)) { // If latch reads successfully, continue to data bits
        errorCode = 2;
        error(false);
        return -1.0;
    }

    unsigned long data = 0;
    for (int i = 0; i < 24 /* Total number of bits in encoder packet. Last bit is error bit, success=1.*/ - 1; i++) { // Read in all 17 data bits
        data <<= 1;
        digitalWrite(5 /* Encoder clock PWM pin*/, 0x0);
        __builtin_avr_delay_cycles(1 * 0x10 /* HARDWARE DEPENDENT!!! For accurate data reading timings. Eq. to Clocks/us*/);
        digitalWrite(5 /* Encoder clock PWM pin*/, 0x1);
        __builtin_avr_delay_cycles(1 * 0x10 /* HARDWARE DEPENDENT!!! For accurate data reading timings. Eq. to Clocks/us*/);

        data |= digitalRead(6 /* Encoder data input pin*/);
    }

    if (!(data & ~(~0U << 1))) { // If last error bit is 0, internal encoder error occured
        errorCode = 3;
        error(true);
    }

    __builtin_avr_delay_cycles(20 * 0x10 /* HARDWARE DEPENDENT!!! For accurate data reading timings. Eq. to Clocks/us*/);
    
# 130 "C:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino" 3
   __asm__ __volatile__ ("sei" ::: "memory")
# 130 "C:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
               ;

    return data >> 24 /* Total number of bits in encoder packet. Last bit is error bit, success=1.*/ - 17 /* Data bits in encoder packet.*/;
}

void error(bool isFatal) {
    Serial.println("Error occured!");
    Serial.println("isFatal: " + isFatal);
    Serial.println("errorCode: " + errorCode);

    while (isFatal) {};
}
