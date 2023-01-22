// Output 38kHz PWM on digital pin D7 on Nano33
void setup() {
    // Feed GCLK0 at 48MHz to TCC0 and TCC1
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |       // Enable GCLK0 as a clock source
                        GCLK_CLKCTRL_GEN_GCLK0 |   // Select GCLK0 at 48MHz
                        GCLK_CLKCTRL_ID_TCC0_TCC1; // Feed GCLK0 to TCC0 and TCC1
    while (GCLK->STATUS.bit.SYNCBUSY)
        ; // Wait for synchronization

    // Enable the port multiplexer for pins D7
    PORT->Group[g_APinDescription[2].ulPort].PINCFG[g_APinDescription[2].ulPin].bit.PMUXEN = 1;
    PORT->Group[g_APinDescription[6].ulPort].PINCFG[g_APinDescription[6].ulPin].bit.PMUXEN = 1;

    // D7 is on EVEN port pin PA06 and TCC1/WO[0] channel 0 is on peripheral E
    PORT->Group[g_APinDescription[2].ulPort].PMUX[g_APinDescription[2].ulPin >> 1].reg = /*PORT_PMUX_PMUXO_E |*/ PORT_PMUX_PMUXE_F;
    PORT->Group[g_APinDescription[6].ulPort].PMUX[g_APinDescription[6].ulPin >> 1].reg = /*PORT_PMUX_PMUXO_E |*/ PORT_PMUX_PMUXE_E;

    // Normal (single slope) PWM operation: timer countinuouslys count up to PER register value and then is reset to 0
    TCC0->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM; // Setup single slope PWM on TCC1
    while (TCC0->SYNCBUSY.bit.WAVE)
        ; // Wait for synchronization

    TCC0->PER.reg = 1262; // Set the frequency of the PWM on TCC1 to 38kHz: 48MHz / (1262 + 1) = 38kHz
    while (TCC0->SYNCBUSY.bit.PER)
        ; // Wait for synchronization

    TCC0->CTRLA.bit.ENABLE = 1; // Enable the TCC1 counter
    while (TCC0->SYNCBUSY.bit.ENABLE)
        ; // Wait for synchronization

    pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
    digitalWrite(LED_BUILTIN, LOW);
    // Using buffered counter compare registers (CCBx)
    TCC0->CCB[0].reg = 0; // TCC1 CCB1 - 25% duty cycle on D2
    while (TCC0->SYNCBUSY.bit.CCB0)
        ;                    // Wait for synchronization
    TCC0->CCB[1].reg = 1261; // TCC1 CCB1 - 25% duty cycle on D6
    while (TCC0->SYNCBUSY.bit.CCB1)
        ;        // Wait for synchronization
    delay(1000); // Wait for 1 second
    digitalWrite(LED_BUILTIN, HIGH);
    TCC0->CCB[0].reg = 1261; // TCC1 CCB1 - 75% duty cycle on D2
    while (TCC0->SYNCBUSY.bit.CCB0)
        ;                 // Wait for synchronization
    TCC0->CCB[1].reg = 0; // TCC1 CCB1 - 75% duty cycle on D6
    while (TCC0->SYNCBUSY.bit.CCB1)
        ;        // Wait for synchronization
    delay(1000); // Wait for 1 second
}