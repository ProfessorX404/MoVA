# 1 "C:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
/*

Xavier Beech

UW SARP 2022-23



Encoder used is Nanotec NME2-SSI-V06-12-C

URL: https://us.nanotec.com/products/8483-nme2-ssi-v06-12-c

Motor used is Nanotec DB59l024035-A

URL: https://us.nanotec.com/products/2870-db59l024035-a



TODO:

- Homing routine

- Change placeholder values to VALVE_OPEN_DEG and VALVE_CLOSED_DEG

- TUNE PID AND WIND UP CONSTANTS

- Count revolutions w Halls

*/
# 17 "C:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
# 18 "C:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino" 2
# 40 "C:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
// Uses built in routine to skip clock cycles for timing purposes.

const double HALL_COMBOS[6][3] = // Combinations of hall sensors based on motor angle, at 45deg increments
    {
  //    {H1, H2, H3}
        {1, 0, 1}, //  0->45
        {0, 0, 1}, //  45->90
        {0, 1, 1}, //  90->135
        {0, 1, 0}, //  135->180
        {1, 1, 0}, //  180->225
        {1, 0, 0} //  225->270
};

double output = 0; // Signed PID output from -255 to 255.
double k_pid[3] = {0.0, 0.0, 0.0}; // PID constants, in format [kP, kI, kD]
double target = 0.0 /* Encoder value for valve being fully closed*/ * (int)(0x20000 /* Number of encoder tics in mechanical revolution (per datasheet)*/ * 15 /* Revs into gearbox per 1 revolution out*/ /* Post-gearbox encoder tics per valve revolution*/ / 360) /* Post-gearbox encoder tics / degree*/; // Current valve position target. Init'ed to closed
double pos = 0.0; // Current valve position
int errorCode = 0; // Global error code variable for fault tracking.
bool activated = false; // Flag for actuating valve.
bool withinOneRev = false; // Flag for activating PID
byte totalRevs = 0; // Revolution counter

ArduPID pid; // PID instance

void setup() {
    Serial.begin(57600); // Init serial connection

    // Init pin values
    pinMode(3 /* Motor control pin*/, 0x1); // Motor magnitude control
    pinMode(5 /* Directional control pin*/, 0x1); // Motor direction control
    pinMode(5 /* Encoder clock PWM pin*/, 0x1); // Encoder serial clock
    pinMode(6 /* Encoder data input pin*/, 0x0); // Encoder data

    analogWrite(3 /* Motor control pin*/, 0); // Set motor to 0, intiailize correct orientation
    digitalWrite(5 /* Directional control pin*/, 1 /* Normalized forward vector. Swap to 0 if reversed.*/);

    updateValvePos(); // Initialize valve position
    pid.begin(&pos, &output, &target, k_pid[0], k_pid[1], k_pid[2]); // Initialize PID
    pid.setOutputLimits(-255, 255); // Motor can be actuated from 0-255 in either direction
    pid.setWindUpLimits(-10.0 /* Integral growth bound min const*/, 10.0 /* Integral growth bound max const*/); // Growth bounds to prevent integral wind-up
}

void loop() {
    if (!activated) { // Wait for system to be activated
        while (!isActivated()) {};
    }
    activated = true;
    withinOneRev = getRevolutions() > (int)((90.0 /* Encoder value for valve being fully open.*/ / 360) * 15 /* Revs into gearbox per 1 revolution out*/) /* Number of rotations to get almost fully open*/;

    if (!withinOneRev) { // If system has been activated but hasn't gotten to within 180* of final open position
        output = 1 /* Normalized forward vector. Swap to 0 if reversed.*/ * 255;
        updateEngineSpeed();
        return;
    }

    if (updateValvePos()) { // If getting bad readings from enc, stop pid to prevent damage to system
        pid.stop();
    } else { // When recovered, continue. Depending on testing, may sub for fatal error.
        pid.start();
    }
    pid.compute(); // Update motor target
    updateEngineSpeed(); // Set motor to target
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
    
# 123 "C:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino" 3
   __asm__ __volatile__ ("cli" ::: "memory")
# 123 "C:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
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
    for (int i = 0; i < 24 /* Total number of bits in encoder packet. Last bit is error bit, success=1.*/ - 1; i++) {
        data <<= 1;

        /*

        // speed up I/O In order to meet the communication speed of this encoder.

        The correct form is:

        PORTD &= ~(1 << n); // Pin n goes low

        PORTD |= (1 << n); // Pin n goes high

        So:

        PORTD &= ~(1 << PD0); // PD0 goes low

        PORTD |= (1 << PD0); // PD0 goes high



        PORTD &= ~(1 << PD1); // PD1 goes low

        PORTD |= (1 << PD1); // PD1 goes high

        */
# 151 "C:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
        
# 151 "C:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino" 3
       (*(volatile uint8_t *)((0x0B) + 0x20)) 
# 151 "C:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
             &= ~(1 << 5 /* Encoder clock PWM pin*/); // clock pin goes low
        __builtin_avr_delay_cycles(1 * 0x10 /* HARDWARE DEPENDENT!!! For accurate data reading timings. Eq. to Clocks/us*/); // Wait for 16 CPU clock cycles @16MHz this is 1 uS
        
# 153 "C:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino" 3
       (*(volatile uint8_t *)((0x0B) + 0x20)) 
# 153 "C:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
             |= (1 << 5 /* Encoder clock PWM pin*/); // clock pin goes high
        __builtin_avr_delay_cycles(1 * 0x10 /* HARDWARE DEPENDENT!!! For accurate data reading timings. Eq. to Clocks/us*/); // Wait for 16 CPU clock cycles
        data |= digitalRead(6 /* Encoder data input pin*/);
    }

    if (!(data & ~(~0U << 1))) { // If last error bit is 0, internal encoder error occured
        errorCode = 3;
        error(true);
    }

    __builtin_avr_delay_cycles(20 * 0x10 /* HARDWARE DEPENDENT!!! For accurate data reading timings. Eq. to Clocks/us*/);
    
# 164 "C:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino" 3
   __asm__ __volatile__ ("sei" ::: "memory")
# 164 "C:\\Users\\xsegg\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
               ;

    return data >> 24 /* Total number of bits in encoder packet. Last bit is error bit, success=1.*/ - 17 /* Data bits in encoder packet.*/;
}

void error(bool isFatal) {
    Serial.println("Error occured!");
    Serial.println("isFatal: " + isFatal);
    Serial.println("errorCode: " + errorCode);

    while (isFatal) {};
}

bool isActivated() { return true; }

byte getRevolutions() { return (int)((90.0 /* Encoder value for valve being fully open.*/ / 360) * 15 /* Revs into gearbox per 1 revolution out*/) /* Number of rotations to get almost fully open*/; }
