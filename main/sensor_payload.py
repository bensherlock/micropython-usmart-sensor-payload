#! /usr/bin/env python
#
# MicroPython USMART Sensor Payload
#
# This file is part of micropython-usmart-sensor-payload.
# https://github.com/bensherlock/micropython-usmart-sensor-payload
#
#
# MIT License
#
# Copyright (c) 2020 Benjamin Sherlock <benjamin.sherlock@ncl.ac.uk>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
"""MicroPython USMART Sensor Payload."""

# Standard Interface for Sensor Payloads for Gateway and Sensor Nodes. 
# For nodes with different sensors available, implement this api and provide your data accordingly.

class SensorPayload:
    """SensorPayload standard class. Inherit form this class to provide your own sensor data."""

    def __init__(self):
        """Initialise."""
        pass

    def __call__(self):
        return self

    def get_est_acquisition_duration(self) -> float:
        """Get estimated acquisition duration in seconds."""
        return 0.0

    def start_acquisition(self):
        """Start an acquisition. Returns True if started successfully."""
        return True

    def process_acquisition(self):
        """Continue processing the acquisition. To be called periodically within the mainloop. 
        This is where the state machine keeps track on progress. """
        return None

    def is_completed(self):
        """Is acquisition completed flag."""
        return True

    def get_latest_data(self) -> bytes:
        """Get the latest data as a bytes."""
        return None

    def get_latest_data_json(self) -> str:
        """Get the latest data as a json string."""
        return None


import pybd_expansion.main.bme280 as bme280
from pybd_expansion.main.bme280 import BME280

import pybd_expansion.main.lsm303agr as lsm303agr
from pybd_expansion.main.lsm303agr import LSM303AGR
import pyb



class PebSensorPayload(SensorPayload):
    """Sensor Payload for the PYBD Expansion Board (PEB).
    Provides readings from: 3-axis accelerometer, 3-axis magnetometer,
    temperature, pressure, humidity within the housing."""

    def __init__(self):
        """Initialise."""
        SensorPayload.__init__(self)

        self._i2c = None

        self._bme280 = None
        self._bme280_awaiting_valid_measurements = False
        self._bme280_pressure = None
        self._bme280_temperature = None
        self._bme280_humidity = None

        self._lsm303agr = None
        self._lsm303agr_awaiting_valid_measurements = False
        self._lsm303agr_temperature = None
        self._lsm303agr_accel = None
        self._lsm303agr_magneto = None


        pass

    def __call__(self):
        return self

    def get_est_acquisition_duration(self) -> float:
        """Get estimated acquisition duration in seconds."""
        return 0.0

    def start_acquisition(self):
        """Start an acquisition. Returns True if started successfully."""

        # Ensure the sensors are powered and the I2C pullups are enabled.
        pyb.Pin.board.EN_3V3.on()
        pyb.Pin('PULL_SCL', pyb.Pin.OUT, value=1)  # enable 5.6kOhm X9/SCL pull-up
        pyb.Pin('PULL_SDA', pyb.Pin.OUT, value=1)  # enable 5.6kOhm X10/SDA pull-up
        # i2c = machine.I2C(1, freq=400000)  # machine.I2C
        i2c = pyb.I2C(1)  # pyb.I2C
        i2c.init(pyb.I2C.MASTER, baudrate=400000)  # pyb.I2C

        self._bme280 = BME280(i2c)
        self._lsm303agr = LSM303AGR(i2c)

        # Clear measurements
        self._bme280_pressure = None
        self._bme280_temperature = None
        self._bme280_humidity = None

        # Setup to take measurements
        self._bme280.set_ctrl_hum_reg(osrs_h=bme280.OSRS_OVERSAMPLE_X_1)
        self._bme280.set_ctrl_meas_reg(mode=bme280.MODE_NORMAL, osrs_p=bme280.OSRS_OVERSAMPLE_X_1, osrs_t=bme280.OSRS_OVERSAMPLE_X_1)
        self._bme280.set_config_reg(filter=bme280.FILTER_COEF_OFF, t_sb=bme280.STANDBY_T_1000_MS)

        self._bme280_awaiting_valid_measurements = True

        # Clear measurements
        self._lsm303agr_temperature = None
        self._lsm303agr_accel = None
        self._lsm303agr_magneto = None

        # Setup to take measurements
        # Temperature
        self._lsm303agr.set_accel_ctrl_reg4(bdu=1)
        self._lsm303agr.set_temp_cfg_reg(en=1)

        # Accelerometer
        self._lsm303agr.set_accel_ctrl_reg1(accel_odr=lsm303agr.ACCEL_ODR_1HZ)
        self._lsm303agr.set_accel_ctrl_reg4(accel_fs=lsm303agr.ACCEL_FS_2G, hr=0, bdu=1)  # bdu=1 needed for temperature

        # Magnetometer
        self._lsm303agr.set_magneto_cfg_reg_a(magneto_odr=lsm303agr.MAGNETO_ODR_10HZ, magneto_md=lsm303agr.MAGNETO_MD_CONTINUOUS, lp=0, comp_temp_en=1)
        self._lsm303agr.set_magneto_cfg_reg_b(lpf=1)
        self._lsm303agr.set_magneto_cfg_reg_c(bdu=1)

        self._lsm303agr_awaiting_valid_measurements = True


        return True

    def process_acquisition(self):
        """Continue processing the acquisition. To be called periodically within the mainloop.
        This is where the state machine keeps track on progress. """

        if self._bme280_awaiting_valid_measurements:
            reg_val, im_update, measuring = self._bme280.get_status_reg()
            if not measuring:
                pressure, temperature, humidity = self._bme280.get_sensor_values_float()

                if pressure and temperature and humidity:
                    self._bme280_awaiting_valid_measurements = False
                    self._bme280_pressure = pressure
                    self._bme280_temperature = temperature
                    self._bme280_humidity = humidity

                    # Stop acquisitions
                    self._bme280.set_ctrl_meas_reg(mode=bme280.MODE_SLEEP, osrs_p=bme280.OSRS_OVERSAMPLE_X_1,
                                                   osrs_t=bme280.OSRS_OVERSAMPLE_X_1)

        if self._lsm303agr_awaiting_valid_measurements:
            # Check the temperature status register for new data
            status_reg, tda, tor = self._lsm303agr.get_temp_status_reg_aux()
            if tda:
                temperature = self._lsm303agr.get_temp_output_float()
                self._lsm303agr_temperature = temperature

            # Check the status register for new accelerometer data
            status_reg, xda, yda, zda, zyxda, xor, yor, zor, zyxor = self._lsm303agr.get_accel_status_reg()
            if zyxda:
                x_val, y_val, z_val = self._lsm303agr.get_accel_outputs()
                self._lsm303agr_accel = (x_val, y_val, z_val)

            # Check the status register for new magnetometer data
            status_reg, xda, yda, zda, zyxda, xor, yor, zor, zyxor = self._lsm303agr.get_magneto_status_reg()
            if zyxda:
                x_val, y_val, z_val = self._lsm303agr.get_magneto_outputs()
                self._lsm303agr_magneto = (x_val, y_val, z_val)

            if self._lsm303agr_temperature and self._lsm303agr_accel and self._lsm303agr_magneto:
                self._lsm303agr_awaiting_valid_measurements = False

                # Stop acquisitions
                self._lsm303agr.set_accel_ctrl_reg1(accel_odr=lsm303agr.ACCEL_ODR_POWERDOWN)
                self._lsm303agr.set_magneto_cfg_reg_a(magneto_odr=lsm303agr.MAGNETO_ODR_10HZ,
                                                      magneto_md=lsm303agr.MAGNETO_MD_IDLE0, lp=0, comp_temp_en=1)


        return None

    def is_completed(self):
        """Is acquisition completed flag."""
        return (not self._bme280_awaiting_valid_measurements) and (not self._lsm303agr_awaiting_valid_measurements)

    def get_latest_data(self) -> bytes:
        """Get the latest data as a bytes."""
        # Format to be determined

        return None

    def get_latest_data_json(self) -> str:
        """Get the latest data as a json string."""
        

        return None


def get_sensor_payload_instance() -> SensorPayload:
    """Get instance of the SensorPayload for this module.
    Override this function in derived modules."""
    return PebSensorPayload()