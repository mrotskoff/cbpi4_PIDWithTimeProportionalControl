
# -*- coding: utf-8 -*-
import os
from aiohttp import web
import logging
import asyncio
import time
import datetime
from cbpi.api import *

@parameters([Property.Number(label = "P", configurable = True, description="P Value of PID"),
             Property.Number(label = "I", configurable = True, description="I Value of PID"),
             Property.Number(label = "D", configurable = True, description="D Value of PID"),
             Property.Number(label = "Sample_Period", configurable = True, default_value = 30, description="How often, in seconds, kettle temp is checked against target temp and heater output is adjusted"),
             Property.Number(label = "Minimum_On_Time", configurable = True, default_value = 10, description="Minimum time in seconds that heater can be turned on (heater output below this value will be ignored)")])

class PIDWithTimeProportionalControl(CBPiKettleLogic):

    async def on_stop(self):
        await self.actor_off(self.heater)
        pass

    async def run(self):
        try:        
            self.TEMP_UNIT = self.get_config_value("TEMP_UNIT", "C")

            sample_period = int(self.props.get("Sample_Period", 30))
            min_on_time = int(self.props.get("Minimum_On_Time", 10))
            p = float(self.props.get("P", 117.0795))
            i = float(self.props.get("I", 0.2747))
            d = float(self.props.get("D", 41.58))

            self.kettle = self.get_kettle(self.id)
            self.heater = self.kettle.heater
            self.heater_actor = self.cbpi.actor.find_by_id(self.heater)
            
            pid = PIDArduino(sample_period, p, i, d, 0, 100)

            while self.running == True:
                self._logger.debug('--- entering PID loop ---')

                self._logger.debug("P input:      {0}".format(p))
                self._logger.debug("I input:      {0}".format(i))
                self._logger.debug("D input:      {0}".format(d))

                sensor_value = current_temp = self.get_sensor_value(self.kettle.sensor).get("value")
                target_temp = self.get_kettle_target_temp(self.id)
                self._logger.debug('sensor temp:  {0}'.format(sensor_value))
                self._logger.debug('target temp:  {0}'.format(target_temp))

                pid_calc = pid.calc(sensor_value, target_temp)
            
                on_time = sample_period * (pid_calc / 100)
                self._logger.debug('on_time:      {0}'.format(on_time))
                
                if on_time < min_on_time:
                    self._logger.debug('on_time below minimum {0}'.format(min_on_time))
                    on_time = 0
                
                off_time = sample_period - on_time
                self._logger.debug('off_time:     {0}'.format(off_time))
            
                if on_time > 0:
                    self._logger.debug('turning heater on for {0} seconds'.format(on_time))
                    await self.actor_on(self.heater)
                    await asyncio.sleep(on_time)
                if off_time > 0:
                    self._logger.debug('turning heater off for {0} seconds'.format(off_time))
                    await self.actor_off(self.heater)
                    await asyncio.sleep(off_time)
                                    
                self._logger.debug('--- exiting PID loop  ---')

        except asyncio.CancelledError as e:
            pass
        except Exception as e:
            self._logger.error("PIDWithTimeProportionalControl Error {}".format(e))
        finally:
            self.running = False
            await self.actor_off(self.heater)

# Based on Arduino PID Library
# See https://github.com/br3ttb/Arduino-PID-Library
class PIDArduino(object):

    def __init__(self, sampleTimeSec, kp, ki, kd, outputMin=float('-inf'),
                 outputMax=float('inf'), getTimeMs=None):
        if kp is None:
            raise ValueError('kp must be specified')
        if ki is None:
            raise ValueError('ki must be specified')
        if kd is None:
            raise ValueError('kd must be specified')
        if float(sampleTimeSec) <= float(0):
            raise ValueError('sampleTimeSec must be greater than 0')
        if outputMin >= outputMax:
            raise ValueError('outputMin must be less than outputMax')

        self._logger = logging.getLogger(type(self).__name__)
        self._Kp = kp
        self._Ki = ki * sampleTimeSec
        self._Kd = kd / sampleTimeSec
        self._sampleTime = sampleTimeSec * 1000
        self._outputMin = outputMin
        self._outputMax = outputMax
        self._iTerm = 0
        self._lastInput = None
        self._lastOutput = 0
        self._lastCalc = 0

        if getTimeMs is None:
            self._getTimeMs = self._currentTimeMs
        else:
            self._getTimeMs = getTimeMs

    def calc(self, inputValue, setpoint):
        now = self._getTimeMs()

        if (now - self._lastCalc) < self._sampleTime:
            return self._lastOutput

        # Compute all the working error variables
        error = setpoint - inputValue
        
        # In order to prevent windup, only integrate if the process is not saturated
        if self._lastOutput < self._outputMax and self._lastOutput > self._outputMin:
            self._iTerm += self._Ki * error
            self._iTerm = min(self._iTerm, self._outputMax)
            self._iTerm = max(self._iTerm, self._outputMin)

        # do not compute a derivative if this is our first input
        dInput = 0
        if self._lastInput is not None:            
            dInput = inputValue - self._lastInput

        p = self._Kp * error
        i = self._iTerm
        d = -(self._Kd * dInput)

        # Compute PID Output
        self._lastOutput = p + i + d
        self._lastOutput = min(self._lastOutput, self._outputMax)
        self._lastOutput = max(self._lastOutput, self._outputMin)

        # Log some debug info
        self._logger.debug('P output:   {0}'.format(p))
        self._logger.debug('I output:   {0}'.format(i))
        self._logger.debug('D output:   {0}'.format(d))
        self._logger.debug('PID output: {0}'.format(self._lastOutput))

        # Remember some variables for next time
        self._lastInput = inputValue
        self._lastCalc = now
        return self._lastOutput

    def _currentTimeMs(self):
        return time.time() * 1000

def setup(cbpi):
    cbpi.plugin.register("PIDWithTimeProportionalControl", PIDWithTimeProportionalControl)

'''
period = 2
p = 10
i = 1
d = 5
pid = PIDArduino(period, p, i, d, 0, 100)
for num in range(100, 160):
    print("sensor val: {}".format(num))
    print("pid calc:   {}". format(pid.calc(num, 150)))
    time.sleep(period)
for num in range(160, 140, -1):
    print("sensor val: {}".format(num))
    print("pid calc:   {}". format(pid.calc(num, 150)))
    time.sleep(period)
for num in range(140, 155):
    print("sensor val: {}".format(num))
    print("pid calc:   {}". format(pid.calc(num, 150)))
    time.sleep(period)
for num in range(155, 145, -1):
    print("sensor val: {}".format(num))
    print("pid calc:   {}". format(pid.calc(num, 150)))
    time.sleep(period)
for num in range(145, 151):
    print("sensor val: {}".format(num))
    print("pid calc:   {}". format(pid.calc(num, 150)))
    time.sleep(period)
for num in range(151, 148, -1):
    print("sensor val: {}".format(num))
    print("pid calc:   {}". format(pid.calc(num, 150)))
    time.sleep(period)
'''

