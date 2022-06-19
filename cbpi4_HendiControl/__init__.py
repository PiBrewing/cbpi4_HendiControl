
# -*- coding: utf-8 -*-
#import os
#from aiohttp import web
import logging
from unittest.mock import MagicMock, patch
import asyncio
import numpy as np
from cbpi.api import *
from cbpi.api.base import CBPiBase

logger = logging.getLogger(__name__)

try:
    import RPi.GPIO as GPIO
except Exception:
    logger.warning("Failed to load RPi.GPIO. Using Mock instead")
    MockRPi = MagicMock()
    modules = {
        "RPi": MockRPi,
        "RPi.GPIO": MockRPi.GPIO
    }
    patcher = patch.dict("sys.modules", modules)
    patcher.start()
    import RPi.GPIO as GPIO

mode = GPIO.getmode()
if (mode == None):
    GPIO.setmode(GPIO.BCM)




class PID(object):
    ek_1 = 0.0
    xk_1 = 0.0
    xk_2 = 0.0

    yk = 0.0

    GMA_HLIM = 100.0
    GMA_LLIM = 0.0

    def __init__(self, ts, kc, ti, td, Pmax=100.0):
        self.kc = kc
        self.ti = ti
        self.td = td
        self.ts = ts
        self.GMA_HLIM = Pmax
        self.k0 = 0.0
        self.k1 = 0.0
        self.pp = 0.0
        self.pi = 0.0
        self.pd = 0.0

        if (self.ti == 0.0):
            self.k0 = 0.0
        else:
            self.k0 = self.kc * self.ts / self.ti
        self.k1 = self.kc * self.td / self.ts

    def calc(self, xk, tset):

        ek = 0.0
        ek = tset - xk # calculate e[k] = SP[k] - PV[k]

        self.pp = self.kc * (self.xk_1 - xk) # y[k] = y[k-1] + Kc*(PV[k-1] - PV[k])
        self.pi = self.k0 * ek  # + Kc*Ts/Ti * e[k]
        self.pd = self.k1 * (2.0 * self.xk_1 - xk - self.xk_2)
        self.yk += self.pp + self.pi + self.pd
        print("------------")
        print(self.yk, self.pp, self.pi, self.pd)

        self.xk_2 = self.xk_1  # PV[k-2] = PV[k-1]
        self.xk_1 = xk    # PV[k-1] = PV[k]

        # limit y[k] to GMA_HLIM and GMA_LLIM
        if (self.yk > self.GMA_HLIM):
            self.yk = self.GMA_HLIM
        if (self.yk < self.GMA_LLIM):
            self.yk = self.GMA_LLIM

        return round(self.yk, 1)


@parameters([Property.Number(label = "P", configurable = True, description="P Value of PID"),
             Property.Number(label = "I", configurable = True, description="I Value of PID"),
             Property.Number(label = "D", configurable = True, description="D Value of PID"),
             Property.Number(label = "Max_Output", configurable = True, description="Power before Boil threshold is reached.")])
             
class PIDHendi(CBPiKettleLogic):
    async def on_stop(self):
        await self.actor_off(self.heater)
        pass


    async def run(self):
        try:
            sampleTime = 5
          
            p = float(self.props.get("P", 40))
            i = float(self.props.get("I", 140))
            d = float(self.props.get("D", 0))
            maxout = int(self.props.get("Max_Output", 100))

            self.kettle = self.get_kettle(self.id)
            self.heater = self.kettle.heater
            self.heater_actor = self.cbpi.actor.find_by_id(self.heater)
                       
            await self.actor_on(self.heater, maxout)

            pid = PID(sampleTime, p, i, d, maxout)

            while self.running == True:
                sensor_value = float(self.get_sensor_value(self.kettle.sensor).get("value",999))
                target_temp = int(self.get_kettle_target_temp(self.id))
                heat_percent = pid.calc(sensor_value, target_temp)
                if heat_percent == 0:
                    await self.actor_set_power(self.heater, 0)
                    await self.actor_off(self.heater)
                    logging.info("PIDHendi OFF")
                else:
                    await self.actor_set_power(self.heater,heat_percent)
                    await self.actor_on(self.heater, heat_percent)

                    logging.info("PIDHendi calling heater_on(power={})".format(heat_percent))

                await asyncio.sleep(sampleTime)

        except asyncio.CancelledError as e:
            pass
        except Exception as e:
            logging.error("PIDHendi {}".format(e))
        finally:
            self.running = False
            await self.actor_set_power(self.heater,0)
            await self.actor_off(self.heater)

@parameters([Property.Select(label="power_pin", options=[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 
                                                        16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27], description="Power control GPIO"),
             Property.Select(label="onoff_pin", description ="On/Off control GPIO",
                                options=[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
                                         20, 21, 22, 23, 24, 25, 26, 27]),
             Property.Number(label="freq", description="PWM frequency", configurable=True),
             Property.Number(label="Pmax",description="Max Power", configurable=True)])  
           
    
class HendiControl(CBPiActor):

    # Custom property which can be configured by the user
    @action("Set Power", parameters=[Property.Number(label="Power", configurable=True,description="Power Setting [0-100]")])
    async def setpower(self,Power = 100 ,**kwargs):
        self.power=int(Power)
        if self.power < 0:
            self.power = 0
        if self.power > 100:
            self.power = 100           
        await self.set_power(self.power)   

    def on_start(self):
        self.state = False
        self.power = 0
        self.pwm = None
        self.power_pin=self.props.get("power_pin",0)
        self.onoff_pin=self.props.get("onoff_pin",0)
        self.Pmax=self.props.get("Pmax",100)       
        self.freq=self.props.get("freq",100)  
        # setup pins for power control
        GPIO.setup(int(self.power_pin), GPIO.OUT)
        # setup pins for on/off control
        GPIO.setup(int(self.onoff_pin), GPIO.OUT)
        GPIO.output(int(self.onoff_pin), 0)
        self.power =  int(self.Pmax)

    async def on(self, power=None):
        if self.pwm is None:
            self.pwm = GPIO.PWM(int(self.power_pin), int(self.freq))
            self.pwm.start(int(self.power))
        if(0 == self.power):
            GPIO.output(int(self.onoff_pin), 0)
        else:
            GPIO.output(int(self.onoff_pin), 1)
            #self.pwm.start(1)
            self.pwm.ChangeDutyCycle(int(self.power))
            logging.info("ON, Set power {}".format(self.power))
            self.state = True
            #await self.cbpi.actor.actor_update(self.id,self.power)

    async def set_power(self, power):
        self.power = min(int(power), int(self.Pmax))
        if(self.power == 0):
            GPIO.output(int(self.onoff_pin), 0)
            if self.pwm:
                self.pwm.ChangeDutyCycle(0)
        if self.pwm:
            try:
                GPIO.output(int(self.onoff_pin), 1)
            except:
                pass
            self.pwm.ChangeDutyCycle(self.power)
        await self.cbpi.actor.actor_update(self.id,self.power)
        logging.info("Set power {}".format(self.power))
        pass


    async def off(self):
        try:
            self.pwm.ChangeDutyCycle(0)
            logging.info("Set Duty Cycle to 0")
            #self.pwm.stop()
        except:
            logging.info("No PWM instance for hendi actor to stop")
            pass
        GPIO.output(int(self.onoff_pin), 0)
        self.state = False
        logging.info("Hendi off")
 
    def get_state(self):
        return self.state
   
    async def run(self):
        while self.running == True:
            await asyncio.sleep(1)
        pass

def setup(cbpi):
    cbpi.plugin.register("HendiControl", HendiControl)
    cbpi.plugin.register("PIDHendi", PIDHendi)
    pass
