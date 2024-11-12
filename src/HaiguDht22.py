import struct
import time


class HAIGU_DHT22:
    """Class to read temperature and humidity from DHT22 I2C HAIGU, much of class was
    this is based on code from https://github.com/jaques/sht21_python/blob/master/sht31.py

    https://jlcpcb.com/partdetail/Haigu-DHT22/C2880291
    
    datasheet:
    https://pajenicko.cz/index.php?route=product/product/get_file&file=2202181630_HAIGU-DHT22_C2880291%20en-GB.pdf
    """

    # control constants
    I2C_ADDRESS = 0x44

    CMD_MEASURE              = 0x2C10 # trigger measurement: temperature and hmidity at once
    CMD_MEASURE_TEMPERATURE  = 0xCC44 # trigger measurement: temperature
    CMD_MEASURE_HUMIDITY     = 0xCC66 # trigger measurement: hmidity
    CMD_READ_STATUS     = 0xF32D # read status register
    CMD_CLEAR_STATUS    = 0x3041 # clear status register
    CMD_SOFT_RESET      = 0x30A2 # soft reset    
    CMD_GET_HUMCOEFA_H  = 0xD208 # Read HumA high 8 bits  
    CMD_GET_HUMCOEFA_L  = 0xD209 # Read HumA lower 8 bits  
    CMD_GET_HUMCOEFB_H  = 0xD20A # Read HumB high 8 bits  
    CMD_GET_HUMCOEFB_L  = 0xD20B # Read HumB lower 8 bits  

    MEASUREMENT_WAIT_TIME = 0.050  

    def __init__(self, i2c, debug=False):
        self.debug = debug
        self._temperature = None
        self.i2c = i2c
        time.sleep(self.MEASUREMENT_WAIT_TIME)
        self.soft_reset()
        
        while True:            
            time.sleep(self.MEASUREMENT_WAIT_TIME)
            self.humA, self.humB = self.get_humidity_coefficients()
            if self.humA is not None and self.humB is not None:
                break
       
    def soft_reset(self):
        self._write(self.CMD_SOFT_RESET)

    def _get_byte_with_crc_check(self, command):
        self._write(command)
        raw_bytes = self._read(3)
        byte, dummy, crc_received = struct.unpack('<BBB', raw_bytes)
        crc_computed = self._calculate_checksum(byte<<8|dummy)
        if crc_computed == crc_received:
            return byte
        return None

    def _get_coefficient(self, command):
        low = self._get_byte_with_crc_check(command)
        high = self._get_byte_with_crc_check(command+1)

        if low is None or high is None:            
           return None
        return low << 8|high

    def get_humidity_coefficients(self):
        humA = self._get_coefficient(self.CMD_GET_HUMCOEFA_H)
        humB = self._get_coefficient(self.CMD_GET_HUMCOEFB_H)
        return humA, humB

    def get_temperature_and_humidity(self):
        self._write(self.CMD_MEASURE)
        time.sleep(self.MEASUREMENT_WAIT_TIME)
        data = self._read(6)
        temp_data, temp_checksum, humidity_data, humidity_checksum = struct.unpack('>HBHB', data)
        
        temp = None
        humi = None
        if self._calculate_checksum(temp_data) == temp_checksum:
           temp = self._compute_temperature(temp_data) 
        if self._calculate_checksum(humidity_data) == humidity_checksum and temp is not None:
           humi = self._compute_humidity(humidity_data, temp)
        return temp, humi

    def _read_integer_response(self):
        data = self._read(3)
        raw_int, raw_checksum = struct.unpack('>HB', data)
        
        if self._calculate_checksum(raw_int) == raw_checksum:
           return raw_int
        return None

    def get_temperature(self):    
        self._write(self.CMD_MEASURE_TEMPERATURE)
        time.sleep(self.MEASUREMENT_WAIT_TIME)
        value = self._read_integer_response()
        if value is None:
           return None 

        return self._compute_temperature(value) 
    
    def get_humidity(self):
        self._write(self.CMD_MEASURE_HUMIDITY)
        time.sleep(self.MEASUREMENT_WAIT_TIME)
        value = self._read_integer_response()
        if value is None:
           return None 
        if self._temperature is None:
           self.get_temperature()
        return self._compute_humidity(value) 

    def _write(self, value):
        self._debug_print(f"write 0x{self.I2C_ADDRESS:X}:", value)
        self.i2c.writeto(self.I2C_ADDRESS, struct.pack(">H", value), False)

    def _read(self, count):
        data = bytearray(count)
        while True:
            # While busy, the sensor doesn't respond to reads.
            try:
                self.i2c.readfrom_into(self.I2C_ADDRESS, data)
                self._debug_print(f"read 0x{self.I2C_ADDRESS:X}: ", data)
                if data[0] != 0xFF:  # Check if read succeeded.
                    break
                time.sleep_ms(1)
            except OSError as e:
                print (e)
                pass      
            except Exception as e:
                print (e)
        return data

    def __enter__(self):
        return self

    def __exit__(self, *exc_info):
        return

    def _calculate_checksum(self, value):
        polynomial = 0x131  # //P(x)=x^8+x^5+x^4+1 = 100110001
        crc = 0xFF
        data = struct.pack(">H", value)
        for byteCtr in range(len(data)):
            crc ^= data[byteCtr]
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ polynomial
                else:
                    crc = crc << 1
                crc &= 0xFF  # Keep CRC to 8-bit

        self._debug_print('Computed CRC', crc, 'for', value)
        return crc

    def _compute_temperature(self, value):
        # strip last 4 junky bits
        value &= 0xFFE0
        if (value>0x7FF):
            value -= 0xFFFE

        self._temperature = round(40.0 + value/256.0,1)
        return self._temperature
    
    def _compute_humidity(self, value, temperature = None):
        rh = 30+(((value-self.humB)*60)/(self.humA-self.humB))
        rel = rh + 0.25 * (self._temperature - 25.0)
        if rel > 100.0:
           rel = 100.0 
        else:
            if rel < 0.0:
               rel = 0.0 
        return int(round(rel,0))

    def _debug_print(self, *args):
        if not self.debug:
            return
        formatted_args = []
        for arg in args:
                if isinstance(arg, str):
                    # Process strings directly
                    formatted_args.append(arg)
                elif isinstance(arg, int):
                    # Format integers (e.g., as hex for debugging)
                    formatted_args.append(f"{arg} 0x{arg:X} 0b{bin(arg)[2:]}")
                elif isinstance(arg, (bytes, bytearray)):
                    # Format bytes as a hex string
                    num = int.from_bytes(arg, 'big')
                    formatted_args.append(f"0x{arg.hex().upper()} {num} 0b{bin(num)[2:]}")
                else:
                    # Fallback for other types
                    formatted_args.append(str(arg))
        
        message = " ".join(formatted_args)
        
        # Print the formatted message
        print(" [DEBUG]", message)

import machine
import sys
if __name__ == "__main__":
    try:
        # Adjust pins and frequency if needed
        i2c = machine.I2C(1, scl=machine.Pin(11), sda=machine.Pin(10), freq=1000)  
        #i2c = machine.SoftI2C(scl=machine.Pin(11), sda=machine.Pin(10), freq=1000)

        print('i2c scanning ...')
        devices = i2c.scan()

        if len(devices) == 0:
          print("No i2c device !")
          machine.soft_reset()
        else:
          print('i2c devices found:', len(devices))
          for device in devices:
            print("i2c hexadecimal address: ", hex(device))    
        
        with HAIGU_DHT22(i2c, False) as sensor:
            while True:
                print('\n')
                print (f"Temperature solo:   {sensor.get_temperature()}°C")
                print (f"Humidity solo:      {sensor.get_humidity()}%")
                temperature, humidity = sensor.get_temperature_and_humidity()
                print (f"Temperature & Humidity: {temperature}°C  &  {humidity}%")
                time.sleep(10)
    except Exception as e:
        print (e)