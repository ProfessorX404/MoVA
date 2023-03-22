import numpy as np
import matplotlib.pyplot as plt

timespan = .1
timestep = 1e-6

x = np.linspace(0, timespan, timespan // timestep)

'''
Ported from:
void update(Encoder enc) {
    uint16_t dt = enc.TC->COUNT16.COUNT.reg; // Time since last loop, in us
    enc.TC->COUNT16.CTRLBSET.reg |= TC_CTRLBSET_CMD_RETRIGGER;
    float theta_n = enc.totalRevs + getPos(enc);

    // Integral approximation based on trapezoidal Riemann sum
    enc.accumulator += (1 / 2) * dt * enc.target * (theta_n + enc.totalRevs - 2);

    // Calculate PID output
    float O = (enc.P * (theta_n - enc.target)) + (enc.I * enc.accumulator) + (enc.D * ((theta_n - enc.totalRevs) / dt));

    PORT_WRITE(enc.DIR_SEL.port, enc.DIR_SEL.pin, O > 0);

    TCC0->CCB[enc.CCB].reg = (abs(O) > PWM_FREQ_COEF) ? PWM_FREQ_COEF : abs(O);

    enc.totalRevs = theta_n;
}
'''
