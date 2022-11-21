# 1 "z:\\Documents\\Github\\motoractuatedvalve-controller\\Controller\\Controller.ino"
/*

Xavier Beech

UW SARP 2022-23

Motorized valve actuation controller software for 2022 Pacific Impulse.



Encoder used is Nanotec NME2-SSI-V06-12-C

URL: https://us.nanotec.com/products/8483-nme2-ssi-v06-12-c

Motor used is Nanotec DB59l024035-A

URL: https://us.nanotec.com/products/2870-db59l024035-a



Libraries used (attached in ../lib for convenience):

 - ArduPID https://github.com/PowerBroker2/ArduPID

 - FireTimer https://github.com/PowerBroker2/FireTimer (ArduPID dependency)

 - ArduinoSTL https://github.com/mike-matera/ArduinoSTL



If using VSCode:



- Place following lines in arduino.json to allow it to build for AVR boards.

      "buildPreferences": [

              [

                  "build.extra_flags",

                  "-D__AVR_ATmega328P__"

              ]

          ]



  - Arduino extension does not work (for me) with Arduino CLI even though the documentation says it does,

  I have to use legacy Arduino IDE v1.8.19



- The only consistent method I've found for getting the extension to find the libraries and include them in

  c_cpp_properties.json' includePath is to install them via the library manager, manual zip extraction does not work.



IMPORTANT!! Before deployment, verify F_CPU is correct for the board you are using. If it is not, all serial communication will

break.



TODO:

- Homing routine (may just end up being manual adjustment)

- Placeholders:

    - VALVE_OPEN_DEG

    - VALVE_CLOSED_DEG

    - HALL_CUTOFF

    - isActivated()

    - getRevolutions()*

- TUNE PID AND WIND UP CONSTANTS

- *Count revolutions w Halls

*/
# 47 "z:\\Documents\\Github\\motoractuatedvalve-controller\\Controller\\Controller.ino"
# 48 "z:\\Documents\\Github\\motoractuatedvalve-controller\\Controller\\Controller.ino" 2
# 49 "z:\\Documents\\Github\\motoractuatedvalve-controller\\Controller\\Controller.ino" 2
# 50 "z:\\Documents\\Github\\motoractuatedvalve-controller\\Controller\\Controller.ino" 2
# 76 "z:\\Documents\\Github\\motoractuatedvalve-controller\\Controller\\Controller.ino"
// Combinations of hall sensors based on motor angle, at 60 deg increments
static const std::array<std::array<bool, 3>, 6 /* Number of posssible Hall combinations*/> HALL_COMBOS = {
  //    {H1, H2, H3}
    {1, 0, 1}, //  0
    {0, 0, 1}, //  60
    {0, 1, 1}, //  120
    {0, 1, 0}, //  180
    {1, 1, 0}, //  240
    {1, 0, 0} //  300, loop around to 0/360
};

// Decimal approximation of how many revolutions each entry in HALL_COMBOS is from 0.
static const std::array<double, 6 /* Number of posssible Hall combinations*/> HALL_COMBO_DEGS = {0, 60 / 360, 120 / 360, 180 / 360, 240 / 360, 300 / 360};

double output = 0; // Signed PID output from -255 to 255.
std::array<double, 3> k_pid = {0.0, 0.0, 0.0}; // PID constants, in format [kP, kI, kD]
double target = 0.0 /* Encoder value for valve being fully closed*/ * (int)(0x20000 /* Number of encoder tics in mechanical revolution (per datasheet)*/ *15 /* Revs into gearbox per 1 revolution out*/ /* Post-gearbox encoder tics per valve revolution*/ / 360) /* Post-gearbox encoder tics / degree*/; // Current valve position target. Init'ed to closed
double pos = 0.0; // Current valve position
int errorCode = 0; // Global error code variable for fault tracking.
byte totalRevs = 0; // Revolution counter
std::array<bool, 3> hallStatus = {0, 0, 0}; // Initial hall effect status, 0deg calibration
byte initHallReading; // Index in HALL_COMBOS of the initial Hall sensor state

// Status flags
bool f_activated = false; // True if valve has been activated
bool f_withinOneRev = false; // True if totalRevs has passed TARGET_REVS
bool f_motorCharged = false; // True if motor has been charged long enough for Hall sensor readings to be valid.

ArduPID pid; // PID instance

void setup() {
    Serial.begin(57600); // Init serial connection

    // Init pin values
    pinMode(3 /* Motor control pin*/, 0x1); // Motor magnitude control
    pinMode(5 /* Directional control pin*/, 0x1); // Motor direction control
    pinMode(5 /* Encoder clock PWM pin*/, 0x1); // Encoder serial clock

    pinMode(6 /* Encoder data input pin*/, 0x0); // Encoder data
    pinMode(A1 /* Hall effect sensor pins.*/, 0x0); // Hall effect sensors
    pinMode(A2 /* Generally to be used digitally, but*/, 0x0);
    pinMode(A3 /* mapped to analog pins for noise-filtering*/, 0x0);

    analogWrite(3 /* Motor control pin*/, 0); // Set motor to 0, intiailize correct orientation
    digitalWrite(5 /* Directional control pin*/, 1 /* Normalized forward vector. Swap to 0 if reversed*/);

    updateValvePos(); // Initialize valve position
    pid.begin(&pos, &output, &target, k_pid[0], k_pid[1], k_pid[2]); // Initialize PID
    pid.setOutputLimits(-255, 255); // Motor can be actuated from 0-255 in either direction
    pid.setWindUpLimits(-10.0 /* Integral growth bound min const*/, 10.0 /* Integral growth bound max const*/); // Growth bounds to prevent integral wind-up
}

void loop() {
    if (!f_activated) { // Wait for system to be activated
        while (!isActivated()) {};
        f_activated = true;
    }

    f_withinOneRev = totalRevs > (int)((90.0 /* Encoder value for valve being fully open*/ / 360) * 15 /* Revs into gearbox per 1 revolution out*/) /* Number of rotations to get almost fully open*/;

    if (!f_withinOneRev) { // If system has been activated but hasn't gotten to within 180* of final open position
        output = 1 /* Normalized forward vector. Swap to 0 if reversed*/ * 255;
        updateEngineSpeed();
        if (f_motorCharged) {
            totalRevs = getRevolutions();
        } else {
            delay(2 /* Time in ms for magnetic fields to form large enough to be registered on Hall sensors*/);
            initHallReading = getHallSensorPosition(readHallSensors());
            f_motorCharged = true;
        }
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

// Takes input from global var, writes to pins accordingly
void updateEngineSpeed() {
    digitalWrite(5 /* Directional control pin*/, output > 0 ? 1 /* Normalized forward vector. Swap to 0 if reversed*/ : ((1 /* Normalized forward vector. Swap to 0 if reversed*/ - 1)>0?(1 /* Normalized forward vector. Swap to 0 if reversed*/ - 1):-(1 /* Normalized forward vector. Swap to 0 if reversed*/ - 1)) /* Normalized reverse vector. Always opposite of C_FORWARD*/);
    analogWrite(3 /* Motor control pin*/, ((output)>0?(output):-(output)));
}

// Takes two readings from encoders, compares, and if they match, updates global var
// Possible issues: if the encoder moves enough to update between 20us minimum,
// it will never return valid value even if encoder is working as designed.
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

// Returns binary representation of encoder readings. Returns -1 if reading fails.
unsigned long readEncData() {
    
# 182 "z:\\Documents\\Github\\motoractuatedvalve-controller\\Controller\\Controller.ino" 3
   __asm__ __volatile__ ("cli" ::: "memory")
# 182 "z:\\Documents\\Github\\motoractuatedvalve-controller\\Controller\\Controller.ino"
                 ; // Deactivate interrupts for more accurate timing
    digitalWrite(5 /* Encoder clock PWM pin*/, 0x0); // First bit is latch, always 1.
    _delay_us(1);
    digitalWrite(5 /* Encoder clock PWM pin*/, 0x1);
    _delay_us(1);
    if (!digitalRead(6 /* Encoder data input pin*/)) { // If latch reads successfully, continue to data bits
        errorCode = 2;
        error(false);
        return -1;
    }

    unsigned long data = 0;
    for (int i = 0; i < 24 /* Total number of bits in encoder packet. Last bit is error bit, success=1*/ - 1; i++) {
        data <<= 1;

        
# 197 "z:\\Documents\\Github\\motoractuatedvalve-controller\\Controller\\Controller.ino" 3
       (*(volatile uint8_t *)((0x0B) + 0x20)) 
# 197 "z:\\Documents\\Github\\motoractuatedvalve-controller\\Controller\\Controller.ino"
             &= ~(1 << 5 /* Encoder clock PWM pin*/); // clock pin goes low
        _delay_us(1); // Wait for 1us
        
# 199 "z:\\Documents\\Github\\motoractuatedvalve-controller\\Controller\\Controller.ino" 3
       (*(volatile uint8_t *)((0x0B) + 0x20)) 
# 199 "z:\\Documents\\Github\\motoractuatedvalve-controller\\Controller\\Controller.ino"
             |= (1 << 5 /* Encoder clock PWM pin*/); // clock pin goes high
        _delay_us(1); // Wait for 1us
        data |= digitalRead(6 /* Encoder data input pin*/);
    }

    if (!(data & ~(~0U << 1))) { // If last error bit is 0, internal encoder error occured
        errorCode = 3;
        error(true); // Throw fatal error
    }

    _delay_us(20);
    
# 210 "z:\\Documents\\Github\\motoractuatedvalve-controller\\Controller\\Controller.ino" 3
   __asm__ __volatile__ ("sei" ::: "memory")
# 210 "z:\\Documents\\Github\\motoractuatedvalve-controller\\Controller\\Controller.ino"
               ; // Reactivate interrupts

    return data >> (24 /* Total number of bits in encoder packet. Last bit is error bit, success=1*/ - 17 /* Data bits in encoder packet*/); // Return the first 17 bits of the data
}

// Outputs error info to serial, if fatal error stalls program
void error(bool isFatal) {
    Serial.println("Error occured!");
    Serial.println("isFatal: " + isFatal);
    Serial.println("errorCode: " + errorCode);

    while (isFatal) {};
}

// Returns the index of the entry in HALL_COMBOS which is the same
// as the given array. Returns -1 if no match found.
byte getHallSensorPosition(std::array<bool, 3> given) {
    for (int i = 0; i < HALL_COMBOS.size(); i++) {
        if ((given[0] == HALL_COMBOS[i][0]) && (given[1] == HALL_COMBOS[i][1]) && (given[2] == HALL_COMBOS[i][2])) {
            return i;
        }
    }
    return -1;
}

// Updates hallStats and returns Hall sensor readings
std::array<bool, 3> readHallSensors() {
    hallStatus[0] = (analogRead(A1 /* Hall effect sensor pins.*/) >= 1023 /* Voltage level cutoff for a positive hall effect status*/);
    hallStatus[1] = (analogRead(A2 /* Generally to be used digitally, but*/) >= 1023 /* Voltage level cutoff for a positive hall effect status*/);
    hallStatus[2] = (analogRead(A3 /* mapped to analog pins for noise-filtering*/) >= 1023 /* Voltage level cutoff for a positive hall effect status*/);
    return hallStatus;
}

// Returns true if valve has been actuated by master controller.
bool isActivated() { return true; } // Placeholder

// Returns the number of complete revolutions the motor has completed.
// Accurate to within 60deg.
byte getRevolutions() { return 0; } // Placeholder
