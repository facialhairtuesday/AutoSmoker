#!/usr/bin/python
#
# This file is part of IvPID.
# Copyright (C) 2015 Ivmech Mechatronics Ltd. <bilgi@ivmech.com>
#
# IvPID is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# IvPID is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

# title           :PID.py
# description     :python pid controller
# author          :Caner Durmusoglu
# date            :20151218
# version         :0.1
# notes           :
# python_version  :2.7
# ==============================================================================

"""Ivmech PID Controller is simple implementation of a Proportional-Integral-Derivative (PID) Controller in the Python Programming Language.
More information about PID Controller: http://en.wikipedia.org/wiki/PID_controller
"""
import time

class PID:
    """PID Controller
    """

    def __init__(self, P=0.2, I=0.0, D=0.0):

        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time

        self.clear()

    def clear(self):
        """Clears PID computations and coefficients"""
        self.SetPoint = 0.0

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output = 0.0

    def update(self, feedback_value):
        """Calculates PID value for given reference feedback
        .. math::
            u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}
        .. figure:: images/pid_1.png
           :align:   center
           Test PID with Kp=1.2, Ki=1, Kd=0.001 (test_pid.py)
        """
        error = self.SetPoint - feedback_value

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

    def setKp(self, proportional_gain):
        """Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        """Determines how aggressively the PID reacts to the current error with setting Integral Gain"""
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        """Determines how aggressively the PID reacts to the current error with setting Derivative Gain"""
        self.Kd = derivative_gain

    def setWindup(self, windup):
        """Integral windup, also known as integrator windup or reset windup,
        refers to the situation in a PID feedback controller where
        a large change in setpoint occurs (say a positive change)
        and the integral terms accumulates a significant error
        during the rise (windup), thus overshooting and continuing
        to increase as this accumulated error is unwound
        (offset by errors in the other direction).
        The specific problem is the excess overshooting.
        """
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        """PID that should be updated at a regular interval.
        Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
        """
        self.sample_time = sample_time


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