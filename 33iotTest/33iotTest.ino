#define PIN_A 6
#define PIN_B 7
#define FREQ  1262
int increment = 1;
bool up = true;

// Output 38kHz PWM on digital pin D7 on Nano33
void setup() {
    // Feed GCLK0 at 48MHz to TCC0 and TCC1
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |       // Enable GCLK0 as a clock source
                        GCLK_CLKCTRL_GEN_GCLK0 |   // Select GCLK0 at 48MHz
                        GCLK_CLKCTRL_ID_TCC0_TCC1; // Feed GCLK0 to TCC0 and TCC1
    while (GCLK->STATUS.bit.SYNCBUSY)
        ; // Wait for synchronization

    // Enable the port multiplexer for pins D7
    PORT->Group[g_APinDescription[PIN_A].ulPort].PINCFG[g_APinDescription[PIN_A].ulPin].bit.PMUXEN = 1;
    PORT->Group[g_APinDescription[PIN_B].ulPort].PINCFG[g_APinDescription[PIN_B].ulPin].bit.PMUXEN = 1;

    // D7 is on EVEN port pin PA06 and TCC1/WO[0] channel 0 is on peripheral E
    PORT->Group[g_APinDescription[PIN_A].ulPort].PMUX[g_APinDescription[PIN_A].ulPin >> 1].reg =
        ((g_APinDescription[PIN_A].ulPin % 2) == 0) ? PORT_PMUX_PMUXE_E : PORT_PMUX_PMUXO_E;
    PORT->Group[g_APinDescription[PIN_B].ulPort].PMUX[g_APinDescription[PIN_B].ulPin >> 1].reg =
        ((g_APinDescription[PIN_B].ulPin % 2) == 0) ? PORT_PMUX_PMUXE_E : PORT_PMUX_PMUXO_E;

    // Normal (single slope) PWM operation: timer countinuouslys count up to PER register value and then is reset to 0
    TCC0->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM; // Setup single slope PWM on TCC1
    while (TCC0->SYNCBUSY.bit.WAVE)
        ;                                    // Wait for synchronization
    TCC1->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM; // Setup single slope PWM on TCC1
    while (TCC1->SYNCBUSY.bit.WAVE)
        ; // Wait for synchronization

    TCC0->PER.reg = FREQ; // Set the frequency of the PWM on TCC1 to 38kHz: 48MHz / (1262 + 1) = 38kHz
    while (TCC0->SYNCBUSY.bit.PER)
        ;                 // Wait for synchronization
    TCC1->PER.reg = FREQ; // Set the frequency of the PWM on TCC1 to 38kHz: 48MHz / (1262 + 1) = 38kHz
    while (TCC1->SYNCBUSY.bit.PER)
        ; // Wait for synchronization

    TCC0->CC[0].reg = .75 * FREQ; // TCC0 CC0 - 50% duty cycle on D7
    while (TCC0->SYNCBUSY.bit.CC0)
        ;                         // Wait for synchronization
    TCC1->CC[0].reg = .75 * FREQ; // TCC0 CC0 - 50% duty cycle on D7
    while (TCC1->SYNCBUSY.bit.CC0)
        ; // Wait for synchronization

    TCC0->CTRLA.bit.ENABLE = 1; // Enable the TCC1 counter
    while (TCC0->SYNCBUSY.bit.ENABLE)
        ;
    TCC1->CTRLA.bit.ENABLE = 1; // Enable the TCC1 counter
    while (TCC1->SYNCBUSY.bit.ENABLE)
        ; // Wait for synchronization
}

void loop() {
    if (increment >= FREQ - 1) {
        up = false;
    } else if (increment <= 0) {
        up = true;
    }
    increment += (((int)up) * 2) - 1;
    // Using buffered counter compare registers (CCBx)
    TCC0->CCB[0].reg = increment; // TCC1 CCB1 - 25% duty cycle on D7
    while (TCC0->SYNCBUSY.bit.CCB0)
        ;                                // Wait for synchronization
    TCC1->CCB[0].reg = FREQ - increment; // TCC1 CCB1 - 25% duty cycle on D7
    while (TCC1->SYNCBUSY.bit.CCB0)
        ; // Wait for synchronization

    delay(1);
}