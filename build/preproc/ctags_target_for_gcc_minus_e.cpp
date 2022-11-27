# 1 "z:\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
/*

Xavier Beech

UW SARP 2022-23

Motorized valve actuation controller software for 2022 Pacific Impulse.

Must comment line 22 of [ArduinoSTL location]/ArduinoSTL/src/new_handler.cpp, or code will not compile.



Encoder used is Nanotec NME2-SSI-V06-12-C

URL: https://us.nanotec.com/products/8483-nme2-ssi-v06-12-c

Motor used is Nanotec DB59l024035-A

URL: https://us.nanotec.com/products/2870-db59l024035-a



Libraries used (attached in ../lib for convenience):

 - ArduPID https://github.com/PowerBroker2/ArduPID

 - FireTimer https://github.com/PowerBroker2/FireTimer (ArduPID dependency)

 - ArduinoSTL https://github.com/mike-matera/ArduinoSTL



If using VSCode:



- Place following lines in arduino.json to allow it to build for AVR boards. Also, if using the Arduino nano

  in the Avionics cabinet in KWT, you must use the "ATmega328P (Old bootloader)" option in the board manager..

      "buildPreferences": [

              [

                  "build.extra_flags",

                  "-D__AVR_ATmega328P__"

              ]

          ]



  - Arduino extension does not work (for me) with Arduino CLI even though the documentation says it does,

  I have to use legacy Arduino IDE v1.8.19



- Manual zip extraction does not work.for library installation, you must go through the library manager.



TODO:

- Manual adjustment

- Placeholders:

    - VALVE_OPEN_DEG

    - VALVE_CLOSED_DEG

    - HALL_CUTOFF

    - isActivated()

    - getRevolutions()*

- TUNE pid[fo] AND WIND UP CONSTANTS

- *Count revolutions w Halls

*/
# 45 "z:\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
# 46 "z:\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino" 2
# 47 "z:\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino" 2
# 48 "z:\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino" 2
# 49 "z:\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino" 2
# 81 "z:\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
static const int C_FORWARD = 1; // Normalized forward vector. Swap to 0 if reversed
static const int C_REVERSE = ((C_FORWARD - 1)>0?(C_FORWARD - 1):-(C_FORWARD - 1)); // Normalized reverse vector. Always opposite of C_FORWARD
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
static const std::array<std::array<byte, 10>, 2> PIN = {
  //  fo, Ox
    {10, 5, 1, 0, A4, A5, A6, 9, 2, A0}, //  CTRL, DIr, ENC_CLK, ENC_DATA, H1, H2, H3, ADJ_L, ADJ_R, ADJ_ACT
    {8, 4, 12, 11, A3, A2, A1, 9, 2, A7}
};

std::array<double, 2> output = {0, 0}; // Signed pid[fo] output from -255 to 255.
std::array<std::array<double, 3>, 2> k_pid = {
    {0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0}
}; // pid[fo] constants, in format [kP, kI, kD]
std::array<double, 2> target = {{0.0 /* Encoder value for valve being fully closed*/ * (int)((0x20000 /* Number of encoder tics in mechanical revolution (per datasheet)*/) * (15 /* Revs into gearbox per 1 revolution out*/) /* Post-gearbox encoder tics per valve revolution*/ / 360) /* Post-gearbox encoder tics / degree*/},
                                {0.0 /* Encoder value for valve being fully closed*/ * (int)((0x20000 /* Number of encoder tics in mechanical revolution (per datasheet)*/) * (15 /* Revs into gearbox per 1 revolution out*/) /* Post-gearbox encoder tics per valve revolution*/ / 360) /* Post-gearbox encoder tics / degree*/}}; // Current valve position target. Init'ed to closed
std::array<double, 2> pos = {0.0, 0.0}; // Current valve position
byte errorCode = 0; // Global error code variable for fault tracking.
std::array<byte, 2> totalRevs = {0, 0}; // Revolution counter
std::array<std::array<bool, 3>, 2> hallStatus = {
    {0, 0, 0},
    {0, 0, 0}
}; // Initial hall effect status, 0deg calibration
std::array<byte, 2> initHallReading; // Index in HALL_COMBOS of the initial Hall sensor state

// Status flags
bool f_activated = false; // True if valve has been activated
std::array<bool, 2> f_withinOneRev = {false, false}; // True if totalRevs has passed TARGET_REVS
std::array<bool, 2> f_motorCharged = {false, false}; // True if motor has been charged long enough for Hall sensor readings to be valid.

std::array<ArduPID, 2> pid; // pid instances

void setup() {
    Serial.begin(57600); // Init serial connection
    byte fo = 0;
    initPins(fo);
    initPins(1);

    analogWrite(PIN[fo][0], 0); // Set motor to 0, intiailize correct orientation
    digitalWrite(PIN[fo][1], C_FORWARD);

    updateValvePos(fo); // Initialize valve position
    pid[fo].begin(&pos[fo], &output[fo], &target[fo], k_pid[fo][0], k_pid[fo][1], k_pid[fo][2]); // Initialize pid[fo]
    pid[fo].setOutputLimits(-255, 255); // Motor can be actuated from 0-255 in either direction
    pid[fo].setWindUpLimits(-10.0 /* Integral growth bound min const*/, 10.0 /* Integral growth bound max const*/); // Growth bounds to prevent integral wind-up
}

void loop() {
    if (!f_activated) { // Wait for system to be activated
        while (!isActivated()) {};
        f_activated = true;
    }
    byte fo = fo;
    f_withinOneRev[fo] = totalRevs[fo] > (fo == fo ? (int)((90.0 /* Encoder value for valve being fully open*/ / 360) * 15 /* Revs into gearbox per 1 revolution out*/) /* Number of rotations to get almost fully open*/ : (int)((90.0 /* Encoder value for valve being fully open*/ / 360) * 15 /* Revs into gearbox per 1 revolution out*/) /* Number of rotations to get almost fully open*/);

    if (!f_withinOneRev[fo]) { // If system has been activated but hasn't gotten to within 180* of final open position
        output[fo] = C_FORWARD * 255;
        updateEngineSpeed(fo);
        if (f_motorCharged[fo]) {
            totalRevs[fo] = getRevolutions();
        } else {
            delay(2 /* Time in ms for magnetic fields to form large enough to be registered on Hall sensors*/);
            updateHallSensors(fo);
            initHallReading[fo] = getHallSensorPosition(hallStatus[fo]);
            f_motorCharged[fo] = true;
        }
        return;
    }

    if (updateValvePos(fo)) { // If getting bad readings from enc, stop pid[fo] to prevent damage to system
        pid[fo].stop();
    } else { // When recovered, continue. Depending on testing, may sub for fatal error.
        pid[fo].start();
    }
    pid[fo].compute(); // Update motor target
    updateEngineSpeed(fo); // Set motor to target
}

// Takes input from global var, writes to pins accordingly
void updateEngineSpeed(byte fo) {
    digitalWrite(PIN[fo][1], output[fo] > 0 ? C_FORWARD : C_REVERSE);
    analogWrite(PIN[fo][0], ((output[fo])>0?(output[fo]):-(output[fo])));
}

// Takes two readings from encoders, compares, and if they match, updates global var
// Possible issues: if the encoder moves enough to update between 20us minimum,
// it will never return valid value even if encoder is working as designed.
bool updateValvePos(byte fo) {
    unsigned long sample1 = readEncData(fo);
    unsigned long sample2 = readEncData(fo);

    if ((sample1 != sample2)) {
        errorCode = 1;
        error(false);
        return true;
    }
    pos[fo] = sample1;
    return false;
}

// Returns binary representation of encoder readings. Returns -1 if reading fails.
unsigned long readEncData(byte fo) {
    
# 195 "z:\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino" 3
   __asm__ __volatile__ ("cli" ::: "memory")
# 195 "z:\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
                 ; // Deactivate interrupts for more accurate timing
    digitalWrite(PIN[fo][2], 0x0); // First bit is latch, always 1.
    _delay_us(1);
    digitalWrite(PIN[fo][2], 0x1);
    _delay_us(1);
    if (!digitalRead(PIN[fo][3])) { // If latch reads successfully, continue to data bits
        errorCode = 2;
        error(false);
        return -1;
    }

    unsigned long data = 0;
    for (int i = 0; i < 24 /* Total number of bits in encoder packet. Last bit is error bit, success=1*/ - 1; i++) {
        data <<= 1;

        
# 210 "z:\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino" 3
       (*(volatile uint8_t *)((0x0B) + 0x20)) 
# 210 "z:\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
             &= ~(1 << PIN[fo][2]); // clock pin goes low
        _delay_us(1); // Wait for 1us
        
# 212 "z:\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino" 3
       (*(volatile uint8_t *)((0x0B) + 0x20)) 
# 212 "z:\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
             |= (1 << PIN[fo][2]); // clock pin goes high
        _delay_us(1); // Wait for 1us
        data |= digitalRead(PIN[fo][3]);
    }

    if (!(data & ~(~0U << 1))) { // If last error bit is 0, internal encoder error occured
        errorCode = 3;
        error(true); // Throw fatal error
    }

    _delay_us(20);
    
# 223 "z:\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino" 3
   __asm__ __volatile__ ("sei" ::: "memory")
# 223 "z:\\Documents\\Git\\motoractuatedvalve-controller\\Controller\\Controller.ino"
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

// Updates hallStatus
void updateHallSensors(byte fo) {
    hallStatus[fo][0] = (analogRead(PIN[fo][4]) >= 1023 /* Voltage level cutoff for a positive hall effect status*/);
    hallStatus[fo][1] = (analogRead(PIN[fo][5]) >= 1023 /* Voltage level cutoff for a positive hall effect status*/);
    hallStatus[fo][2] = (analogRead(PIN[fo][6]) >= 1023 /* Voltage level cutoff for a positive hall effect status*/);
}

// Returns true if valve has been actuated by master controller.
bool isActivated() { return true; } // Placeholder

// Returns the number of complete revolutions the motor has completed.
// Accurate to within 60deg.
byte getRevolutions() { return 0; } // Placeholder

void initPins(byte fo) {
    pinMode(PIN[fo][0], output[fo]); // Motor magnitude control
    pinMode(PIN[fo][1], output[fo]); // Motor direction control
    pinMode(PIN[fo][2], output[fo]); // Encoder serial clock

    pinMode(PIN[fo][3], 0x0); // Encoder data
    pinMode(PIN[fo][4], 0x0); // Hall effect sensors
    pinMode(PIN[fo][5], 0x0);
    pinMode(PIN[fo][6], 0x0);
}
