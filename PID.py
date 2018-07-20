############################################################################################
#
# PID algorithm to take input sensor readings, and target requirements, and
# as a result feedback new rotor speeds.
#
############################################################################################
class PID:

    def __init__(self, p_gain, i_gain, d_gain, now):
        self.last_error = 0.0
        self.last_time = now

        self.p_gain = p_gain
        self.i_gain = i_gain
        self.d_gain = d_gain

        self.i_error = 0.0

    def Compute(self, input, target, now):
        dt = (now - self.last_time)

        # ---------------------------------------------------------------------------
        # Error is what the PID alogithm acts upon to derive the output
        # ---------------------------------------------------------------------------
        error = target - input

        # ---------------------------------------------------------------------------
        # The proportional term takes the distance between current input and target
        # and uses this proportially (based on Kp) to control the ESC pulse width
        # ---------------------------------------------------------------------------
        p_error = error

        # ---------------------------------------------------------------------------
        # The integral term sums the errors across many compute calls to allow for
        # external factors like wind speed and friction
        # ---------------------------------------------------------------------------
        self.i_error += (error + self.last_error) * dt
        i_error = self.i_error

        # ---------------------------------------------------------------------------
        # The differential term accounts for the fact that as error approaches 0,
        # the output needs to be reduced proportionally to ensure factors such as
        # momentum do not cause overshoot.
        # ---------------------------------------------------------------------------
        d_error = (error - self.last_error) / dt

        # ---------------------------------------------------------------------------
        # The overall output is the sum of the (P)roportional, (I)ntegral and (D)iffertial terms
        # ---------------------------------------------------------------------------
        p_output = self.p_gain * p_error
        i_output = self.i_gain * i_error
        d_output = self.d_gain * d_error

        # ---------------------------------------------------------------------------
        # Store off last input for the next differential calculation and time for next integral calculation
        # ---------------------------------------------------------------------------
        self.last_error = error
        self.last_time = now

        # ---------------------------------------------------------------------------
        # Return the output, which has been tuned to be the increment / decrement in ESC PWM
        # ---------------------------------------------------------------------------
        return p_output, i_output, d_output


############################################################################################
#
#  Class for managing each blade + motor configuration via its ESC
#
############################################################################################
class HEATER:

    def __init__(self, pin):
        # ---------------------------------------------------------------------------
        # The GPIO BCM numbered pin providing PWM signal for this ESC
        # ---------------------------------------------------------------------------
        self.bcm_pin = pin

        # ---------------------------------------------------------------------------
        # Initialize the RPIO DMA PWM for the THERMOSTAT in microseconds - full range
        # of pulse widths for 3ms carrier.
        # ---------------------------------------------------------------------------
        self.min_pulse_width = 0
        self.max_pulse_width = 2999

        # ---------------------------------------------------------------------------
        # The PWM pulse range required by this ESC
        # ---------------------------------------------------------------------------
        pulse_width = self.min_pulse_width

        # ---------------------------------------------------------------------------
        # Initialize the RPIO DMA PWM for the THERMOSTAT.
        # ---------------------------------------------------------------------------
        PWM.add_channel_pulse(RPIO_DMA_CHANNEL, self.bcm_pin, 0, pulse_width)

    def update(self, temp_out):
        pulse_width = int(self.min_pulse_width + temp_out)

        if pulse_width < self.min_pulse_width:
            pulse_width = self.min_pulse_width
        if pulse_width > self.max_pulse_width:
            pulse_width = self.max_pulse_width

        PWM.add_channel_pulse(RPIO_DMA_CHANNEL, self.bcm_pin, 0, pulse_width)


temp_pid = PID(PID_TEMP_P_GAIN, PID_TEMP_I_GAIN, PID_TEMP_D_GAIN, time_now)
heater = HEATER(RPIO_THERMOSTAT_PWM)

[p_out, i_out, d_out] = temp_pid.Compute(temp_now, MPU6050_TEMP_TARGET, time_now)
temp_out = p_out + i_out + d_out
heater.update(temp_out)

'''
import time
import logging
import logging.config

# Start logging
logging.config.fileConfig('/home/pi/EasySmoker/logging.conf')
logger = logging.getLogger(__name__)


# PID controller based on proportional band in standard PID form https://en.wikipedia.org/wiki/PID_controller#Ideal_versus_standard_PID_form
# u = Kp (e(t)+ 1/Ti INT + Td de/dt)
# PB = Proportional Band
# Ti = Goal of eliminating in Ti seconds
# Td = Predicts error value at Td in seconds

class PID:
    def __init__(self, PB, Ti, Td):
        self.CalculateGains(PB, Ti, Td)

        self.P = 0.0
        self.I = 0.0
        self.D = 0.0
        self.u = 0

        self.Derv = 0.0
        self.Inter = 0.0
        self.Inter_max = abs(0.5 / self.Ki)

        self.Last = 150

        self.setTarget(0.0)

    def CalculateGains(self, PB, Ti, Td):
        self.Kp = -1 / PB
        self.Ki = self.Kp / Ti
        self.Kd = self.Kp * Td
        logger.info('PB: %f Ti: %f Td: %f --> Kp: %f Ki: %f Kd: %f', PB, Ti, Td, self.Kp, self.Ki, self.Kd)

    def update(self, Current):
        # P
        error = Current - self.setPoint
        self.P = self.Kp * error + 0.5  # P = 1 for PB/2 under setPoint, P = 0 for PB/2 over setPoint

        # I
        dT = time.time() - self.LastUpdate
        # if self.P > 0 and self.P < 1: #Ensure we are in the PB, otherwise do not calculate I to avoid windup
        self.Inter += error * dT
        self.Inter = max(self.Inter, -self.Inter_max)
        self.Inter = min(self.Inter, self.Inter_max)

        self.I = self.Ki * self.Inter

        # D
        self.Derv = (Current - self.Last) / dT
        self.D = self.Kd * self.Derv

        # PID
        self.u = self.P + self.I + self.D

        # Update for next cycle
        self.error = error
        self.Last = Current
        self.LastUpdate = time.time()

        logger.info('Target: %d Current: %d Gains: (%f,%f,%f) Errors(%f,%f,%f) Adjustments: (%f,%f,%f) PID: %f',
                    self.setPoint, Current, self.Kp, self.Ki, self.Kd, error, self.Inter, self.Derv, self.P, self.I,
                    self.D, self.u)

        return self.u

    def setTarget(self, setPoint):
        self.setPoint = setPoint
        self.error = 0.0
        self.Inter = 0.0
        self.Derv = 0.0
        self.LastUpdate = time.time()
        logger.info('New Target: %f', setPoint)

    def setGains(self, PB, Ti, Td):
        self.CalculateGains(PB, Ti, Kd)
        self.Inter_max = abs(0.5 / self.Ki)
        logger.info('New Gains (%f,%f,%f)', self.Kp, self.Ki, self.Kd)

    def getK(self):
        return self.Kp, self.Ki, self.Kd
'''
