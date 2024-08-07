import time
from smbus2 import SMBus


class ClickSeries:
    """Base class for Click series sensors"""

    def __init__(self, bus: SMBus, i2c_address: int, verbose=False) -> None:
        """Constructor for Click series sensors

        Parameters
        ----------
        bus : SMBus
            SMBus object
        i2c_address : int
            I2C address of the sensor
        verbose : bool, optional
            Verbose mode, by default False

        Raises
        ------
        Exception
            Sensor not found
        """
        self.bus = bus
        self.i2c_address = i2c_address
        self.verbose = verbose
        print(f"Barometer: {self.__class__.__name__}")

        if not self._check_device(self.i2c_address):
            raise Exception("Sensor not found")
        else:
            self._reset_sensor()
            time.sleep(0.2)
            self.prom_data = self._read_prom()
            print("Sensor initialized")

    def _reset_command(self) -> int:
        pass

    def _check_device(self, address: int) -> bool:
        """Helper function to check if the I2C device is connected

        Parameters
        ----------
        address : int
            I2C address of the device

        Returns
        -------
        bool
            True if the device is connected, False otherwise

        Raises
        ------
        Exception
            [description]
        """
        try:
            self.bus.read_byte(address)
            return True
        except Exception as e:
            if self.verbose:
                print(f"[ClickSeries] {e}")
            return False

    def _send_command(self, command: int) -> bool:
        """Helper function to send a command to the sensor

        Parameters
        ----------
        command : int
            Command to send

        Returns
        -------
        bool
            True if successful
        """
        if self._check_device(self.i2c_address):
            self.bus.write_byte(self.i2c_address, command)
            if self.verbose:
                print(f"Command {command} sent to sensor")
            return True
        return False

    def _reset_sensor(self) -> bool:
        """Helper function to reset the sensor

        Returns
        -------
        bool
            True if successful, False otherwise
        """
        return self._send_command(self._reset_command())

    def _read_prom(self) -> list:
        pass

    def _read_adc(self, command: int) -> int:
        pass

    def read_sensor(self) -> tuple:
        pass

    def __del__(self) -> None:
        """Destructor for Click series sensors"""
        self.bus.close()


class MS5637(ClickSeries):
    """Class for MS5637 sensor"""

    def __init__(self, bus: SMBus, i2c_address: int, verbose=False) -> None:
        """Constructor for MS5637 sensor

        Parameters
        ----------
        bus : SMBus
            SMBus object
        i2c_address : int
            I2C address of the sensor
        verbose : bool, optional
            Verbose mode, by default False
        """
        self._RESET_COMMAND = 0x1E
        self._PROM_READ_COMMAND = 0xA0
        self._CONVERT_D1_COMMAND = 0x40
        self._CONVERT_D2_COMMAND = 0x50
        self._ADC_READ_COMMAND = 0x00
        super().__init__(bus, i2c_address, verbose)

    def _reset_command(self) -> int:
        """Function to get the reset command"""
        return self._RESET_COMMAND

    def _read_prom(self) -> list:
        """Function to read the calibration data

        Returns
        -------
        list
            List containing calibration data
            [0]: Pressure sensitivity
            [1]: Pressure offset
            [2]: Temperature coefficient of pressure sensitivity
            [3]: Temperature coefficient of pressure offset
            [4]: Reference temperature
            [5]: Temperature coefficient of the temperature
        """
        prom_data = []
        for i in range(1, 7):
            prom_cmd = self._PROM_READ_COMMAND + (i * 2)
            data = self.bus.read_i2c_block_data(self.i2c_address, prom_cmd, 2)
            prom_data.append((data[0] << 8) + data[1])

        if self.verbose:
            print(f"PROM data: {prom_data}")
        return prom_data

    def _read_adc(self, command: int) -> int:
        """Function to read the ADC data

        Parameters
        ----------
        command : int
            Command to read the ADC data

        Returns
        -------
        int
            ADC data
        """
        self.bus.write_byte(self.i2c_address, command)
        time.sleep(0.01)
        data = self.bus.read_i2c_block_data(self.i2c_address, self._ADC_READ_COMMAND, 3)
        adc_data = (data[0] << 16) + (data[1] << 8) + data[2]

        if self.verbose:
            print(f"ADC data: {adc_data}")
        return adc_data

    def read_sensor(self) -> tuple:
        """Function to read sensor data

        Returns
        -------
        tuple
            Tuple containing pressure and temperature (pressure [hPa], temperature [\u00B0C])
        """
        D1 = self._read_adc(self._CONVERT_D1_COMMAND)  # Pressure
        D2 = self._read_adc(self._CONVERT_D2_COMMAND)  # Temperature

        # Calculate pressure and temperature using calibration data
        dT = D2 - self.prom_data[4] * 256
        TEMP = 2000 + dT * self.prom_data[5] / 8388608

        # Second order temperature compensation
        if TEMP < 2000:
            T2 = 3 * (dT * dT) / 8589934592
            OFF2 = 61 * ((TEMP - 2000) * (TEMP - 2000)) / 16
            SENS2 = 29 * ((TEMP - 2000) * (TEMP - 2000)) / 16

            if TEMP < -1500:
                OFF2 += 17 * ((TEMP + 1500) * (TEMP + 1500))
                SENS2 += 9 * ((TEMP + 1500) * (TEMP + 1500))
        else:
            T2 = 5 * (dT * dT) / 274877906944
            OFF2 = 0
            SENS2 = 0

        OFF = (self.prom_data[1] * 131072) + ((self.prom_data[3] * dT) / 64)
        OFF -= OFF2
        SENS = (self.prom_data[0] * 65536) + ((self.prom_data[2] * dT) / 128)
        SENS -= SENS2

        P = ((D1 * SENS / 2097152) - OFF) / 32768
        return P / 100, (TEMP - T2) / 100


class MS5611(MS5637):
    """Class for MS5611 sensor"""

    def __init__(self, bus: SMBus, i2c_address: int, verbose=False) -> None:
        """Constructor for MS5611 sensor

        Parameters
        ----------
        bus : SMBus
            SMBus object
        i2c_address : int
            I2C address of the sensor
        verbose : bool, optional
            Verbose mode, by default False

        Raises
        ------
        Exception
            Sensor not found
        """
        super().__init__(bus, i2c_address, verbose)
        self._CONVERT_D1_COMMAND = 0x48  # Highest oversampling rate
        self._CONVERT_D2_COMMAND = 0x58  # Highest oversampling rate

    def read_sensor(self) -> tuple:
        """Function to read sensor data

        Returns
        -------
        tuple
            Tuple containing pressure and temperature (pressure [hPa], temperature [\u00B0C])
        """
        D1 = self._read_adc(self._CONVERT_D1_COMMAND)
        D2 = self._read_adc(self._CONVERT_D2_COMMAND)

        # Calculate pressure and temperature using calibration data
        dT = D2 - self.prom_data[4] * 256
        TEMP = 2000 + dT * self.prom_data[5] / 8388608

        # Second order temperature compensation
        if TEMP < 2000:
            T2 = dT * dT / 2147483648
            OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2
            SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4

            if TEMP < -1500:
                OFF2 += 7 * ((TEMP + 1500) * (TEMP + 1500))
                SENS2 += 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2

        else:
            T2 = 0
            OFF2 = 0
            SENS2 = 0

        OFF = (self.prom_data[1] * 65536) + ((self.prom_data[3] * dT) / 128)
        OFF -= OFF2
        SENS = (self.prom_data[0] * 32768) + ((self.prom_data[2] * dT) / 256)
        SENS -= SENS2

        P = ((D1 * SENS / 2097152) - OFF) / 32768
        return P / 100, (TEMP - T2) / 100


def apply_fir_filter():
    pass
