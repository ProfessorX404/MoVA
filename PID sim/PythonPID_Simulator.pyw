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
from scipy.optimize import minimize, NonlinearConstraint, show_options, fmin


class PID(object):
    def __init__(
        self,
        Kp=1.0,
        Ki=0.1,
        Kd=0.01,
        setpoint=50,
        output_limits=(0, 100),
        model_params=None
    ):

        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.prev_tunings = (0, 0, 0)
        self.prev_itae = 0
        self._setpoint = setpoint
        self._min_output, self._max_output = 0, 100
        self._model_params = model_params
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
        self.l_itae = None
        self._has_new_values = True
        self.results = None
        self.iterations = 0
        self.reset()

    def calc(self, PV=0, SP=0):
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
        self._derivative = self.Kd * (eD - self._last_eD)

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
    def model_params(self):
        return self._model_params

    @model_params.setter
    def model_params(self, model_params):
        for i in range(len(model_params)):
            if self._model_params is None or self._model_params[i] != model_params[i]:
                self._model_params = model_params
                self._has_new_values = True
                break

    @property
    def components(self):
        return self._proportional, self._integral, self._derivative

    @property
    def tunings(self):
        return self.Kp, self.Ki, self.Kd

    @tunings.setter
    def tunings(self, tunings):
        if(tunings[0] != self.Kp or tunings[1] != self.Ki or tunings[2] != self.Kd):
            self.prev_tunings = (self.Kp, self.Ki, self.Kd)
            self.Kp, self.Ki, self.Kd = tunings
            self._has_new_values = True

    @ property
    def output_limits(self):
        return self._min_output, self._max_output

    @ output_limits.setter
    def output_limits(self, limits=None):
        if limits is None:
            if self._min_output != 0 or self._max_output != 100:
                self._has_new_values = True
            self._min_output, self._max_output = 0, 100
            return
        min_output, max_output = limits
        if self._min_output != min_output or self._max_output != max_output:
            self._has_new_values = True
        self._min_output = min_output
        self._max_output = max_output
        self._integral = self._clamp(self._integral, self.output_limits)

    @ property
    def setpoint(self):
        return self._setpoint

    @ setpoint.setter
    def setpoint(self, SP):
        if self.setpoint != SP:
            self._setpoint = SP
            self._has_new_values = True

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

    def __call__(self):
        if self._has_new_values == False:
            return self.results

        self.iterations += 1
        igain, itau, ideadtime, ibias, startofstep = self._model_params
        # Find the size of the range needed
        if (ideadtime + itau) * 6 < minsize:
            rangesize = minsize
        elif (ideadtime + itau) * 6 > maxsize:
            rangesize = maxsize
        else:
            rangesize = int((ideadtime + itau) * 6)

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

        # plant Instantiation
        plant = FOPDTModel(CV, self.model_params[:4])

        # Start Value
        PV[0] = ibias + noise[0]

        # if mode == 'full':
        # Loop through timestamps
        for i in t:
            if i < len(t) - 1:
                # if i < startofstep:
                #     SP[i] = ibias
                # elif i < rangesize*0.6:
                #     SP[i] = 60 + ibias
                # else:
                #     SP[i] = 40 + ibias
                SP[i] = self.setpoint

                # Find current controller output
                CV[i] = self.calc(PV[i], SP[i])
                ts = [t[i], t[i + 1]]
                # Send step data
                plant.CV = CV
                # Find calculated PV
                PV[i + 1] = plant.update(PV[i], ts)
                PV[i + 1] += noise[i]
                # Store indiv. terms
                pterm[i], iterm[i], dterm[i] = self.components
            else:
                # cleanup endpoint
                SP[i] = SP[i - 1]
                CV[i] = CV[i - 1]
                pterm[i] = pterm[i - 1]
                iterm[i] = iterm[i - 1]
                dterm[i] = dterm[i - 1]

        self.results = (t, PV, SP, CV, pterm, iterm, dterm)
        self._has_new_values = False
        return self.results


class FOPDTModel(object):
    def __init__(self, PlantParams, ModelData):
        self.CV = PlantParams
        self.Gain, self.TimeConstant, self.DeadTime, self.Bias = ModelData

    def calc(self, PV, ts):
        if (ts - self.DeadTime) <= 0:
            um = 0
        elif int(ts - self.DeadTime) >= len(self.CV):
            um = self.CV[-1]
        else:
            um = self.CV[int(ts - self.DeadTime)]
        dydt = (-(PV - self.Bias) + self.Gain * um) / self.TimeConstant
        return dydt

    def update(self, PV, ts):
        y = odeint(self.calc, PV, ts)
        return y[-1]


def calcITAE(PV, SP, startofstep):
    itae = 0
    for i in range(len(PV)):
        itae = 0 if i < startofstep else itae + \
            (i - startofstep) * abs(SP[i] - PV[i])
    if len(PV) > 0:
        return itae / len(PV)
    else:
        return 0


def timeToStable(t, PV, SP, percent_err):
    # max_err = np.max(np.abs(pterm)) * percent_err
    # for i in range(len(t)):
    #     l = min(len(t), i + back_steps)
    #     if np.abs(np.average(pterm[i:l])) < max_err:
    #         return t[i]
    # return t[-1]
    zero_crossings = np.where(np.diff(np.sign(PV - SP)))[0]
    print(zero_crossings)
    zero_crossings = [0] + zero_crossings
    print(zero_crossings)
    apexes = np.zeros(len(zero_crossings))
    for i in range(len(zero_crossings)):
        if i > 0:
            apexes[i] = np.max(
                np.abs(PV[zero_crossings[i-1]:zero_crossings[i]]))
    for i in range(len(apexes)):
        if i > 1:
            if abs(apexes[i] - apexes[i-1]) > percent_err * apexes[i]:
                return t[int(zero_crossings[i])]
    return t[-1]


def _jac(Ks, itae, prev_itae):
    global pid
    grad = np.zeros(len(Ks))
    print(Ks, pid.prev_tunings)
    for i in range(len(Ks)):
        grad[i] = (Ks[i] - pid.prev_tunings[i]) / (itae - prev_itae)
    print(grad)
    return grad


def _data(Ks, SP, model_params):
    global pid
    pid.tunings = Ks
    pid.setpoint = SP
    pid.model_params = model_params
    return pid()


def optimize():
    global buttons
    model_params = tuple([float(x.get()) for x in buttons[0:5]])
    pid_params = tuple([float(x.get()) for x in buttons[5:9]])
    gc_params = tuple([float(x.get()) for x in buttons[9:]])
    p, i, d, sp = pid_params
    max_itae, n_iter, perm_error, tolerance, back_steps, act_time, pc_err = gc_params
    igain, itau, ideadtime, ibias, startofstep = model_params
    print("Inputs: ", p, i, d, sp)
    def data(Ks): return _data(Ks, sp, model_params)

    def tts(Ks): return timeToStable(data(Ks)[0], data(Ks)[
        1], data(Ks)[2])  # ignore, old fitness function

    def itae_func(Ks): return calcITAE(
        data(Ks)[1], data(Ks)[2], startofstep) / 200  # scale output to ~magnitude 1 for fmin compatibility

    def jac(Ks): return _jac(Ks, itae_func(Ks), itae_func(
        pid.prev_tunings))  # gradient for use with minimize
    # tts_con = NonlinearConstraint(tts, 0, act_time)
    # err_con = NonlinearConstraint(itae_func, 0., max_itae / 150)
    # results = minimize(itae_func, (p, i, d), method="Newton-CG", jac=jac,
    #                    tol=tolerance, options={'maxiter': n_iter, 'disp': True}, callback=update_graphs)
    results = fmin(itae_func, (p, i, d), ftol=tolerance,
                   maxiter=n_iter, disp=True, callback=update_graphs)
    print(results)
    # print(kP_opt, kI_opt, kD_opt)
    # buttons[5].insert(10, str(kP_opt))
    # buttons[6].insert(10, str(kI_opt))
    # buttons[7].insert(10, str(kD_opt))
    # refresh(buttons)


def update_graphs(Ks):
    global buttons, pid
    tKp, tKi, tKd = buttons[5:8]
    print("Iteration #:", pid.iterations, "; inputs:", Ks,
          "; ITAE:", calcITAE(pid.results[1], pid.results[2], 10))
    tKp.delete(0, tk.END)
    tKp.insert(10, str(Ks[0]))
    tKi.delete(0, tk.END)
    tKi.insert(10, str(Ks[1]))
    tKd.delete(0, tk.END)
    tKd.insert(10, str(Ks[2]))
    refresh()


def refresh():
    global pid, buttons
    model_params = tuple([float(x.get()) for x in buttons[0:5]])
    pid_params = tuple([float(x.get()) for x in buttons[5:9]])
    # get values from tkinter
    startofstep = model_params[-1]
    ikp, iki, ikd, iSP = pid_params

    pid.tunings = (ikp, iki, ikd)
    pid.setpoint = iSP
    pid.model_params = model_params

    t, PV, SP, CV, pterm, iterm, dterm = pid()
    # Plots
    if pid.l_SP is None:
        # Display itae value
        # measure PID performance
        global itae_text
        global tts_text
        itae_text.set(round(calcITAE(PV, SP, startofstep), 2))
        pid.l_SP, = pid.ax1.plot(t, SP, color="blue", linewidth=2, label='SP')
        pid.l_CV, = pid.ax1.plot(
            t, CV, color="darkgreen", linewidth=2, label='CV')
        pid.l_PV, = pid.ax1.plot(t, PV, color="red", linewidth=2, label='PV')
        pid.ax1.set_ylabel('EU')
        pid.fig.suptitle("ITAE: %s" % round(
            calcITAE(PV, SP, startofstep), 2))
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

        error_set = (calcITAE(PV[:i], SP[:i], startofstep)
                     for i in range(len(t)))
        pid.l_itae, = pid.ax3.plot(t, tuple(error_set))

        pid.fig.show()
    else:
        # Display itae value
        # measure PID performance
        itae_text.set(round(calcITAE(PV, SP, startofstep), 2))
        pid.fig.suptitle("ITAE: %s" % round(
            calcITAE(PV, SP, startofstep), 2))
        pid.ax1.set_title("Kp:%s   Ki:%s  Kd:%s" %
                          (ikp, iki, ikd), fontsize=10)
        pid.l_SP.set_ydata(SP)
        pid.l_CV.set_ydata(CV)
        pid.l_PV.set_ydata(PV)
        pid.l_kP.set_ydata(pterm)
        pid.l_kI.set_ydata(iterm)
        pid.l_kD.set_ydata(dterm)
        error_set = (calcITAE(PV[:i], SP[:i], startofstep)
                     for i in range(len(t)))
        pid.l_itae.set_ydata(tuple(error_set))
        pid.fig.canvas.draw()
        pid.fig.canvas.flush_events()


if __name__ == "__main__":
    # Random Noise between -0.25 and 0.25, same set used for each run. Created once at runtime.
    minsize = 600
    maxsize = 18000
    noise = np.random.rand(maxsize) / 2
    noise -= 0.25

    # Gui
    root = tk.Tk()
    root.title('PID Simulator')
    root.resizable(True, True)
    root.geometry('725x150')

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
    tk.Label(root, text="Kp:").grid(row=1, column=3, sticky="E")
    tk.Label(root, text="Ki:").grid(row=2, column=3, sticky="E")
    tk.Label(root, text="Kd:").grid(row=3, column=3, sticky="E")
    tk.Label(root, text="Setpoint: ").grid(row=4, column=3, sticky="E")
    tk.Label(root, text="revs").grid(row=4, column=5, sticky="W")
    tk.Label(root, text="1/usec").grid(row=2, column=5, sticky="W")
    tk.Label(root, text="usec").grid(row=3, column=5, sticky="W")
    tk.Label(root, text="Optimization").grid(row=0, column=8)
    tk.Label(root, text="Max ITAE: ").grid(row=1, column=6, sticky="E")
    tk.Label(root, text="GC iterations: ").grid(row=2, column=6, sticky="E")
    tk.Label(root, text="Perm err: ").grid(row=3, column=6, sticky="E")
    tk.Label(root, text="Tolerance: ").grid(row=4, column=6, sticky="E")
    tk.Label(root, text="Grad steps: ").grid(row=1, column=8, sticky="E")
    tk.Label(root, text="Actuation time: ").grid(row=2, column=8, sticky="E")
    tk.Label(root, text="usec").grid(row=2, column=10, sticky="W")
    tk.Label(root, text="Stable err: ").grid(row=3, column=8, sticky="E")
    tk.Label(root, text="(*100= \% err)").grid(row=3, column=10, sticky="W")

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
    stable_err = tk.Entry(root, width=8)

    # Defaults
    tK.insert(10, "2.25")
    ttau.insert(10, "60.5")
    tdt.insert(10, "9.99")
    tSoS.insert(10, "10")
    tibias.insert(10, ".1")
    tKp.insert(10, "1.1")
    tKi.insert(10, "0.1")
    tKd.insert(10, "0.09")
    tSP.insert(10, "3.5")
    max_itae.insert(10, "100")
    n_iter.insert(10, "50")
    perm_error.insert(10, str(float(1 / 12)))
    tolerance.insert(10, ".5")
    grad_steps.insert(10, "5")
    act_time.insert(10, "800")
    stable_err.insert(10, ".05")

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
    stable_err.grid(row=3, column=9)

    # Buttons
    buttons = (tK, ttau, tdt, tibias, tSoS, tKp, tKi, tKd, tSP, max_itae,
               n_iter, perm_error, tolerance, grad_steps, act_time, stable_err)
    pid = PID(float(tKp.get()), float(tKi.get()),
              float(tKd.get()), float(tSP.get()))
    button_calc = tk.Button(root, text="Refresh",
                            command=refresh)
    button_optimize = tk.Button(
        root, text="Optimize", command=optimize)
    tk.Label(root, text="itae:").grid(row=5, column=3)
    itae_text = tk.StringVar()
    tk.Label(root, textvariable=itae_text).grid(row=5, column=4)
    button_calc.grid(row=7, column=0)
    button_optimize.grid(row=7, column=2)

    root.mainloop()
