"""

Updated and maintained by destination0b10unknown@gmail.com
Copyright 2022 destination2unknown

Licensed under the MIT License;
you may not use this file except in compliance with the License.

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

"""
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
import tkinter as tk
from scipy.optimize import minimize, NonlinearConstraint, show_options


class PID(object):
    def __init__(
        self,
        Kp=1.0,
        Ki=0.1,
        Kd=0.01,
        setpoint=50,
        output_limits=(0, 100),
    ):

        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self._setpoint = setpoint
        self._min_output, self._max_output = 0, 100
        self._proportional = 0
        self._integral = 0
        self._derivative = 0
        self.output_limits = output_limits
        self._last_eD = 0
        self._lastCV = 0
        self._d_init = 0
        self.fig = plt.figure()
        self.ax1 = self.fig.add_subplot(3, 1, 1)
        self.ax2 = self.fig.add_subplot(3, 1, 2)
        self.ax3 = self.fig.add_subplot(3, 1, 3)
        self.l_SP = None
        self.l_CV = None
        self.l_PV = None
        self.l_kP = None
        self.l_kI = None
        self.l_kD = None
        self.reset()

    def __call__(self, PV=0, SP=0):
        # PID calculations
        # P term
        e = SP - PV
        self._proportional = self.Kp * e

        # I Term
        if self._lastCV < 100 and self._lastCV > 0:
            self._integral += self.Ki * e
        # Allow I Term to change when Kp is set to Zero
        if self.Kp == 0 and self._lastCV == 100 and self.Ki * e < 0:
            self._integral += self.Ki * e
        if self.Kp == 0 and self._lastCV == 0 and self.Ki * e > 0:
            self._integral += self.Ki * e

        # D term
        eD = -PV
        self._derivative = self.Kd*(eD - self._last_eD)

        # init D term
        if self._d_init == 0:
            self._derivative = 0
            self._d_init = 1

        # Controller Output
        CV = self._proportional + self._integral + self._derivative
        CV = self._clamp(CV, self.output_limits)

        # update stored data for next iteration
        self._last_eD = eD
        self._lastCV = CV
        return CV

    @property
    def components(self):
        return self._proportional, self._integral, self._derivative

    @property
    def tunings(self):
        return self.Kp, self.Ki, self.Kd

    @tunings.setter
    def tunings(self, tunings):
        self.Kp, self.Ki, self.Kd = tunings

    @property
    def output_limits(self):
        return self._min_output, self._max_output

    @output_limits.setter
    def output_limits(self, limits):
        if limits is None:
            self._min_output, self._max_output = 0, 100
            return
        min_output, max_output = limits
        self._min_output = min_output
        self._max_output = max_output
        self._integral = self._clamp(self._integral, self.output_limits)

    @property
    def setpoint(self):
        return self._setpoint

    @setpoint.setter
    def setpoint(self, SP):
        self._setpoint = SP

    def reset(self):
        # Reset
        self._proportional = 0
        self._integral = 0
        self._derivative = 0
        self._integral = self._clamp(self._integral, self.output_limits)
        self._last_eD = 0
        self._lastCV = 0
        self._last_eD = 0

    def _clamp(self, value, limits):
        lower, upper = limits
        if value is None:
            return None
        elif (upper is not None) and (value > upper):
            return upper
        elif (lower is not None) and (value < lower):
            return lower
        return value


class FOPDTModel(object):
    def __init__(self, PlantParams, ModelData):
        self.CV = PlantParams
        self.Gain, self.TimeConstant, self.DeadTime, self.Bias = ModelData

    def calc(self, PV, ts):
        if (ts-self.DeadTime) <= 0:
            um = 0
        elif int(ts-self.DeadTime) >= len(self.CV):
            um = self.CV[-1]
        else:
            um = self.CV[int(ts-self.DeadTime)]
        dydt = (-(PV-self.Bias) + self.Gain * um)/self.TimeConstant
        return dydt

    def update(self, PV, ts):
        y = odeint(self.calc, PV, ts)
        return y[-1]


def getData(model_params, pid_params, perm_err=.0125, mode='full'):
    igain, itau, ideadtime, startofstep, ibias = model_params
    ikp, iki, ikd, iSP = pid_params
    # Find the size of the range needed
    if (ideadtime+itau)*6 < minsize:
        rangesize = minsize
    elif (ideadtime+itau)*6 > maxsize:
        rangesize = maxsize
    else:
        rangesize = int((ideadtime+itau)*6)

    # setup time intervals
    t = np.arange(start=0, stop=rangesize, step=1)
    # Setup data arrays
    SP = np.zeros(len(t))
    PV = np.zeros(len(t))
    CV = np.zeros(len(t))
    pterm = np.zeros(len(t))
    iterm = np.zeros(len(t))
    dterm = np.zeros(len(t))
    global noise
    noise = np.resize(noise, len(t))
    # noise = np.zeros(len(t)) #no noise

    # Packup data
    global pid
    # pid = PID(ikp, iki, ikd, SP[0])
    PIDGains = (ikp, iki, ikd)
    ModelData = (igain, itau, ideadtime, ibias)

    # PID Instantiation
    pid.tunings = (PIDGains)

    # plant Instantiation
    plant = FOPDTModel(CV, ModelData)

    # Start Value
    PV[0] = ibias+noise[0]

    if True:  # mode == 'full':
        # Loop through timestamps
        for i in t:
            if i < len(t)-1:
                if i < startofstep:
                    SP[i] = ibias
                # elif i < rangesize*0.6:
                #     SP[i] = 60 + ibias
                else:
                    SP[i] = 40 + ibias

                # Find current controller output
                CV[i] = pid(PV[i], SP[i])
                ts = [t[i], t[i+1]]
                # Send step data
                plant.CV = CV
                # Find calculated PV
                PV[i+1] = plant.update(PV[i], ts)
                PV[i+1] += noise[i]
                # Store indiv. terms
                pterm[i], iterm[i], dterm[i] = pid.components
            else:
                # cleanup endpoint
                SP[i] = SP[i-1]
                CV[i] = CV[i-1]
                pterm[i] = pterm[i-1]
                iterm[i] = iterm[i-1]
                dterm[i] = dterm[i-1]

            itae = 0 if i < startofstep else itae + \
                (i-startofstep)*abs(SP[i]-PV[i])
        return (t, SP, CV, PV, pterm, iterm, dterm, itae)

    # elif mode == 'quick':
    #     for i in t:
    #         if i < len(t)-1:
    #             if i < startofstep:
    #                 SP[i] = ibias
    #             # elif i < rangesize*0.6:
    #             #     SP[i] = 60 + ibias
    #             else:
    #                 SP[i] = 40 + ibias
    #             # Find current controller output
    #             CV[i] = pid(PV[i], SP[i])
    #             itae = 0 if i < startofstep else itae + \
    #                 (i-startofstep)*abs(SP[i]-PV[i])
    #             if(itae <= perm)
    #             return (t[i], err, itae)
    #             ts = [t[i], t[i+1]]
    #             # Send step data
    #             plant.CV = CV
    #             # Find calculated PV
    #             PV[i+1] = plant.update(PV[i], ts)
    #             PV[i+1] += noise[i]
    #             # Store indiv. terms
    #             pterm[i], iterm[i], dterm[i] = pid.components
    #         else:
    #             return -1


def timeToStable(t, PV, SP, back_steps=5, percent_err=.05):
    err = np.subtract(PV, SP)
    max_err = np.abs((np.max(err) - np.min(err)) * percent_err)
    for i in len(t):
        l = min(len(t), i + back_steps)
        if np.abs(np.average(err[i:l])) < max_err:
            return t[i]


def gradient(curr, prev, dx):
    grad = np.zeros(len(curr))
    for i in range(len(curr)):
        grad[i] = (curr[i] - prev[i]) / dx
    return grad


def optimize(buttons):

    model_params = tuple([float(x.get()) for x in buttons[0:5]])
    pid_params = tuple([float(x.get()) for x in buttons[5:8]])
    gc_params = tuple([float(x.get()) for x in buttons[8:]])
    n_iter, perm_error, tolerance, back_steps, act_time = gc_params
    def data(Ks): return getData(*model_params, *Ks,
                                 pid_params[-1], backwards_steps=back_steps, perm_err=perm_error, mode='quick')

    def tts(Ks): return [0]
    def itae_func(Ks): return data
    tts_con = NonlinearConstraint(tts, 0, act_time)
    err_con = NonlinearConstraint(itae_func, -perm_error, perm_error)
    kP_opt, kI_opt, kD_opt = minimize(tts, pid_params[0:-1], constraints=(
        tts_con, err_con), tol=tolerance, options={'maxiter': n_iter})


def refresh(buttons):
    global pid
    model_params = tuple([float(x.get()) for x in buttons[0:5]])
    pid_params = tuple([float(x.get()) for x in buttons[5:8]])
    # get values from tkinter
    igain, itau, ideadtime, startofstep, ibias = model_params
    ikp, iki, ikd, iSP = pid_params

    t, SP, CV, PV, pterm, iterm, dterm = getData(
        model_params, pid_params)
    # Plots
    if pid.l_SP is None:
        # Display itae value
        itae_text.set(round(itae/len(t), 2))  # measure PID performance
        pid.l_SP, = pid.ax1.plot(t, SP, color="blue", linewidth=2, label='SP')
        pid.l_CV, = pid.ax1.plot(
            t, CV, color="darkgreen", linewidth=2, label='CV')
        pid.l_PV, = pid.ax1.plot(t, PV, color="red", linewidth=2, label='PV')
        pid.ax1.set_ylabel('EU')
        pid.fig.suptitle("ITAE: %s" % round(itae/len(t), 2))
        pid.ax1.set_title("Kp:%s   Ki:%s  Kd:%s" %
                          (ikp, iki, ikd), fontsize=10)
        pid.ax1.legend(loc='best')
        pid.l_kP, = pid.ax2.plot(t, pterm, color="lime",
                                 linewidth=2, label='P Term')
        pid.l_kI, = pid.ax2.plot(t, iterm, color="orange",
                                 linewidth=2, label='I Term')
        pid.l_kD, = pid.ax2.plot(t, dterm, color="purple",
                                 linewidth=2, label='D Term')
        pid.ax2.set_xlabel('Time [usec]')
        pid.ax2.legend(loc='best')

        def itae(): return 0 if i < startofstep else itae + \
            (i-startofstep)*abs(SP[i]-PV[i])
        pid.ax3.plot(t[0:-1], error_set)

        pid.fig.show()
    else:
        # Display itae value
        itae_text.set(round(itae/len(t), 2))  # measure PID performance
        pid.l_SP.set_ydata(SP)
        pid.l_CV.set_ydata(CV)
        pid.l_PV.set_ydata(PV)
        pid.l_kP.set_ydata(pterm)
        pid.l_kI.set_ydata(iterm)
        pid.l_kD.set_ydata(dterm)
        pid.fig.canvas.draw()
        pid.fig.canvas.flush_events()


if __name__ == "__main__":
    # Random Noise between -0.25 and 0.25, same set used for each run. Created once at runtime.
    minsize = 600
    maxsize = 18000
    noise = np.random.rand(maxsize)/2
    noise -= 0.25

    # Gui
    root = tk.Tk()
    root.title('PID Simulator')
    root.resizable(True, True)
    root.geometry('650x150')

    # Labels
    tk.Label(root, text=" ").grid(row=0, column=0)
    tk.Label(root, text="FOPDT").grid(row=0, column=1)
    tk.Label(root, text="Model Gain: ").grid(row=1, sticky="E")
    tk.Label(root, text="TimeConstant: ").grid(row=2, sticky="E")
    tk.Label(root, text="DeadTime: ").grid(row=3, sticky="E")
    tk.Label(root, text="Start of step: ").grid(row=4, sticky="E")
    tk.Label(root, text="ibias: ").grid(row=5, column=0, sticky="E")
    tk.Label(root, text="                ").grid(row=0, column=2)
    tk.Label(root, text="                ").grid(row=1, column=2)
    tk.Label(root, text="usec             ").grid(row=2, column=2)
    tk.Label(root, text="usec             ").grid(row=3, column=2)
    tk.Label(root, text="PID Gains").grid(row=0, column=4)
    tk.Label(root, text="Kp:").grid(row=1, column=3)
    tk.Label(root, text="Ki:").grid(row=2, column=3)
    tk.Label(root, text="Kd:").grid(row=3, column=3)
    tk.Label(root, text="Setpoint: ").grid(row=4, column=3)
    tk.Label(root, text="revs").grid(row=4, column=5)
    tk.Label(root, text="1/usec").grid(row=2, column=5, sticky="W")
    tk.Label(root, text="usec").grid(row=3, column=5, sticky="W")
    tk.Label(root, text="Optimization").grid(row=0, column=8)
    tk.Label(root, text="Max ITAE: ").grid(row=1, column=6)
    tk.Label(root, text="GC iterations: ").grid(row=2, column=6)
    tk.Label(root, text="Stable err: ").grid(row=3, column=6)
    tk.Label(root, text="Tolerance: ").grid(row=4, column=6)
    tk.Label(root, text="Grad steps: ").grid(row=1, column=8)
    tk.Label(root, text="Actuation time: ").grid(row=2, column=8)
    tk.Label(root, text="usec").grid(row=2, column=10)

    # Entry Boxes
    tK = tk.Entry(root, width=8)
    ttau = tk.Entry(root, width=8)
    tdt = tk.Entry(root, width=8)
    tSoS = tk.Entry(root, width=8)
    tibias = tk.Entry(root, width=8)
    tKp = tk.Entry(root, width=8)
    tKi = tk.Entry(root, width=8)
    tKd = tk.Entry(root, width=8)
    tSP = tk.Entry(root, width=8)
    max_itae = tk.Entry(root, width=8)
    n_iter = tk.Entry(root, width=8)
    perm_error = tk.Entry(root, width=8)
    tolerance = tk.Entry(root, width=8)
    grad_steps = tk.Entry(root, width=8)
    act_time = tk.Entry(root, width=8)

    # Defaults
    tK.insert(10, "2.25")
    ttau.insert(10, "60.5")
    tdt.insert(10, "9.99")
    tSoS.insert(10, "10")
    tibias.insert(10, "10")
    tKp.insert(10, "1.1")
    tKi.insert(10, "0.1")
    tKd.insert(10, "0.09")
    tSP.insert(10, "3.5")
    max_itae.insert(10, "100")
    n_iter.insert(10, "1000")
    perm_error.insert(10, str(float(1/12)))
    tolerance.insert(10, "1e-6")
    grad_steps.insert(10, "5")
    act_time.insert(10, "800")

    # Placement
    tK.grid(row=1, column=1)
    ttau.grid(row=2, column=1)
    tdt.grid(row=3, column=1)
    tSoS.grid(row=4, column=1)
    tibias.grid(row=5, column=1)
    tKp.grid(row=1, column=4)
    tKi.grid(row=2, column=4)
    tKd.grid(row=3, column=4)
    tSP.grid(row=4, column=4)
    max_itae.grid(row=1, column=7)
    n_iter.grid(row=2, column=7)
    perm_error.grid(row=3, column=7)
    tolerance.grid(row=4, column=7)
    grad_steps.grid(row=1, column=9)
    act_time.grid(row=2, column=9)

    # Buttons
    buttons = (tK, ttau, tdt, tSoS, tibias, tKp, tKi, tKd, tSP, max_itae,
               n_iter, perm_error, tolerance, grad_steps, act_time)
    pid = PID(float(tKp.get()), float(tKi.get()),
              float(tKd.get()), float(tSP.get()))
    button_calc = tk.Button(root, text="Refresh",
                            command=lambda: refresh(buttons))
    button_optimize = tk.Button(
        root, text="Optimize", command=lambda: optimize(buttons))
    tk.Label(root, text="itae:").grid(row=5, column=3)
    itae_text = tk.StringVar()
    tk.Label(root, textvariable=itae_text).grid(row=5, column=4)
    button_calc.grid(row=7, column=0)
    button_optimize.grid(row=7, column=2)

    root.mainloop()
