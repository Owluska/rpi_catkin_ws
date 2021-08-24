#! /usr/bin/env python3
# Copyright (c) 2016 Adafruit Industries
# Author: Tony DiCola
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
from __future__ import division
import logging
import time
import math
import smbus

# Registers/etc:
PCA9685_ADDRESS    = 0x70
MODE1              = 0x00
MODE2              = 0x01
SUBADR1            = 0x02
SUBADR2            = 0x03
SUBADR3            = 0x04
PRESCALE           = 0xFE
LED0_ON_L          = 0x06
LED0_ON_H          = 0x07
LED0_OFF_L         = 0x08
LED0_OFF_H         = 0x09
ALL_LED_ON_L       = 0xFA
ALL_LED_ON_H       = 0xFB
ALL_LED_OFF_L      = 0xFC
ALL_LED_OFF_H      = 0xFD

# Bits:
RESTART            = 0x80
SLEEP              = 0x10
ALLCALL            = 0x01
INVRT              = 0x10
OUTDRV             = 0x04


logger = logging.getLogger(__name__)


class PCA9685(object):
    """PCA9685 PWM LED/servo controller."""

    def __init__(self, address=PCA9685_ADDRESS, i2c=None, **kwargs):
        """Initialize the PCA9685."""
        # Setup I2C interface for the device.
        if i2c is None:
            import Adafruit_GPIO.I2C as I2C
            i2c = I2C
        self._device = i2c.get_i2c_device(address, **kwargs)
        self.set_all_pwm(0, 0)
        self._device.write8(MODE2, OUTDRV)
        self._device.write8(MODE1, ALLCALL)
        time.sleep(0.005)  # wait for oscillator
        mode1 = self._device.readU8(MODE1)
        mode1 = mode1 & ~SLEEP  # wake up (reset sleep)
        self._device.write8(MODE1, mode1)
        time.sleep(0.005)  # wait for oscillator

    def software_reset(self, i2c=None, **kwargs):
        """Sends a software reset (SWRST) command to all servo drivers on the bus."""
        # Setup I2C interface for device 0x00 to talk to all of them.
        if i2c is None:
            import Adafruit_GPIO.I2C as I2C
            i2c = I2C
        self._device = i2c.get_i2c_device(0x00, **kwargs)
        self._device.writeRaw8(0x06)  # SWRST    
    
    def set_pwm_freq(self, freq_hz):
        """Set the PWM frequency to the provided value in hertz."""
        prescaleval = 25000000.0    # 25MHz
        prescaleval /= 4096.0       # 12-bit
        prescaleval /= float(freq_hz)
        prescaleval -= 1.0
        logger.debug('Setting PWM frequency to {0} Hz'.format(freq_hz))
        logger.debug('Estimated pre-scale: {0}'.format(prescaleval))
        prescale = int(math.floor(prescaleval + 0.5))
        logger.debug('Final pre-scale: {0}'.format(prescale))
        oldmode = self._device.readU8(MODE1);
        newmode = (oldmode & 0x7F) | 0x10    # sleep
        self._device.write8(MODE1, newmode)  # go to sleep
        self._device.write8(PRESCALE, prescale)
        self._device.write8(MODE1, oldmode)
        time.sleep(0.005)
        self._device.write8(MODE1, oldmode | 0x80)

    def set_pwm(self, channel, on, off):
        """Sets a single PWM channel."""
        self._device.write8(LED0_ON_L+4*channel, on & 0xFF)
        self._device.write8(LED0_ON_H+4*channel, on >> 8)
        self._device.write8(LED0_OFF_L+4*channel, off & 0xFF)
        self._device.write8(LED0_OFF_H+4*channel, off >> 8)

    def set_all_pwm(self, on, off):
        """Sets all PWM channels."""
        self._device.write8(ALL_LED_ON_L, on & 0xFF)
        self._device.write8(ALL_LED_ON_H, on >> 8)
        self._device.write8(ALL_LED_OFF_L, off & 0xFF)
        self._device.write8(ALL_LED_OFF_H, off >> 8)

    
class car_movement_PCA9685():
    FREQUENCY = 50
    MAX_SPEED = 4095
    MIN_SPEED = 0
    
    CENTER_DEGREE = 100
    MAX_DEGREE = CENTER_DEGREE + 50
    MIN_DEGREE = CENTER_DEGREE - 50
    

    A1 = 6
    B1 = 5

    A2 = 2
    B2 = 1

    RUL = 0
    def __init__(self, pca):
        self.pwm = pca
        self.pwm.set_pwm_freq(self.FREQUENCY)
        self.stop_all()
    
    def stop(self, A, B):
        self.pwm.set_pwm(A, 1, self.MAX_SPEED)
        self.pwm.set_pwm(B, 1, self.MAX_SPEED)
    
    def stop_all(self):
        self.stop(self.A1, self.B1)
        self.stop(self.A2, self.B2)
        self.turn(self.CENTER_DEGREE)

    def move_backward(self, velocity):
        if velocity < 0 or velocity > 100:
            velocity = 100  
        velocity = int(velocity * self.MAX_SPEED/100)
        #print("Moving forward with velocity {}".format(velocity))
        self.pwm.set_pwm(self.A1, 0, velocity)
        self.pwm.set_pwm(self.A2, 0, velocity)
        
        self.pwm.set_pwm(self.B1, 0, self.MIN_SPEED)
        self.pwm.set_pwm(self.B2, 0, self.MIN_SPEED)
       
    def move_forward(self, velocity):
        self.pwm.set_pwm(self.A1, 0, self.MIN_SPEED)
        self.pwm.set_pwm(self.A2, 0, self.MIN_SPEED)
        
        if velocity < 0 or velocity > 100:
            velocity = 100  
        velocity = int(velocity * self.MAX_SPEED/100)
        #print("Moving backward with velocity {}".format(velocity))
        self.pwm.set_pwm(self.B1, 0, velocity)       
        self.pwm.set_pwm(self.B2, 0, velocity)
    
  
 
    def turn(self, degree):
        #print(degree)
        if degree < self.MIN_DEGREE or degree > self.MAX_DEGREE:
            degree = self.CENTER_DEGREE
        MIN_WIDTH = 0.02 * self.MAX_SPEED
        MAX_WIDTH = 0.12 * self.MAX_SPEED

        duty_cycle = MIN_WIDTH + (MAX_WIDTH-MIN_WIDTH) * degree/180
        duty_cycle = int(duty_cycle)
  
        # print("ON:{}".format(duty_cycle))
        self.pwm.set_pwm(self.RUL, 0, duty_cycle)




# pwm = PCA9685(0x41)
# car = car_movement_PCA9685(pwm)
# debug = "servos"

# if debug == "motors":
#     while True:
#         stop = False
#         for i in range(1, 11):
#             try:
#                 print(10 * i)
#                 car.move_forward(velocity = 10 * i)
#                 time.sleep(1)
#                 car.move_backward(velocity = 10 * i)
#                 time.sleep(1)
#             except KeyboardInterrupt:
#                 car.stop_all()
#                 stop = True
#                 break
#         if stop:
#             break

# elif debug == "servos":
#     while True:
#         stop = False
#         for i in range(95, 120, 1):
#             try:
#                 car.turn(degree = i)
#                 time.sleep(2)
#                 print(i)
#             except KeyboardInterrupt:
#                 car.stop_all()
#                 stop = True
#                 break
#         if stop:
#             break
#         time.sleep(5)





