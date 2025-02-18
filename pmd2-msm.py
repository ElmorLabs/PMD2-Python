import serial
import serial.tools.list_ports
from ctypes import *
from enum import IntEnum
import numpy as np
import matplotlib.pyplot as plt
import time

# Constants
PRODUCT_NAME = "PMD2"
PRODUCT_STRING = "ElmorLabs PMD2"
VENDOR_ID = 0xEE
PRODUCT_ID = 0x15
FIRMWARE_VERSION = 0x00 # Pre-rel firmware
SENSOR_POWER_NUM = 10

# Structures
class VendorDataStruct(Structure):
    _pack_ = 1
    _fields_ = [
        ('VendorId', c_uint8),
        ('ProductId', c_uint8),
        ('FwVersion', c_uint8)
    ]

    def __str__(self):
        return f'VendorId: {hex(self.VendorId)}, ProductId: {hex(self.ProductId)}, FwVersion: {hex(self.FwVersion)}'


class PowerSensor(Structure):
    _pack_ = 1
    _fields_ = [
        ('Voltage', c_int16),
        ('Current', c_int32),
        ('Power', c_int32)
    ]

    def __str__(self):
        return f'Voltage: {self.Voltage}, Current: {self.Current}, Power: {self.Power}'


class SensorStruct(Structure):
    _pack_ = 1
    _fields_ = [
        ('Vdd', c_uint16),
        ('Tchip', c_int16),
        ('PowerReadings', PowerSensor * SENSOR_POWER_NUM),
        ('EpsPower', c_uint16),
        ('PciePower', c_uint16),
        ('MbPower', c_uint16),
        ('TotalPower', c_uint16),
        ('Ocp', c_uint8 * SENSOR_POWER_NUM)
    ]

    def __str__(self):
        vin = ', '.join(str(v) for v in self.Vin)
        ts = ', '.join(str(t) for t in self.Ts)
        power_readings = ', '.join(str(p) for p in self.PowerReadings)
        fans = ', '.join(str(f) for f in self.Fans)
        return f'Vdd:{self.Vref}\nTchip: {self.FanExt}\nPowerReadings: {power_readings}'

# Enums
class UART_CMD(IntEnum):
    CMD_WELCOME = 0
    CMD_READ_VENDOR_DATA = 1
    CMD_READ_UID = 2
    CMD_READ_DEVICE_DATA = 3
    CMD_READ_SENSOR_VALUES = 4
    CMD_WRITE_CONT_TX = 5
    CMD_READ_CALIBRATION = 6
    CMD_WRITE_CALIBRATION = 7
    CMD_LOAD_CALIBRATION = 8
    CMD_STORE_CALIBRATION = 9
    CMD_RESET = 0xF0
    CMD_BOOTLOADER = 0xF1
    CMD_NVM_CONFIG = 0xF2
    CMD_NOP = 0xFF

    # Function to get byte value
    def toByte(self):
        return self.value.to_bytes(1, byteorder='little')


if __name__ == '__main__':
    # Find device Serial Port
    print(f"\nFinding potential {PRODUCT_NAME} Serial Ports...\n")
    ports = []
    available_ports = serial.tools.list_ports.comports()

    for port, desc, hwid in sorted(available_ports):
            # Check for VID=0483 and PID=5740
            if hwid.startswith('USB VID:PID=0483:5740'):
                print("{} - {}: {} [{}]".format(len(ports), port, desc, hwid))
                ports.append(port)
    print("\n")

    device_port = 0
    if len(ports) == 0:
        print(f"No {PRODUCT_NAME} Serial Ports found.")
        exit()

    if len(ports) > 1:
        print(f"Multiple {PRODUCT_NAME} Serial Ports found. Input which port to use:")
        device_port = int(input())
        if device_port >= len(ports):
            print("Invalid port index.")
            exit()

    print(f"Using port index {device_port} ({ports[device_port]})\n")

    ser = serial.Serial(ports[device_port], 115200, timeout=1, rtscts=False, dsrdtr=False)

    # Read product string
    print("Reading welcome string...")
    ser.dtr = True
    #ser.write(UART_CMD.CMD_WELCOME.toByte())
    buffer = ser.read(len(PRODUCT_STRING)+1)
    print(str(buffer))
    assert buffer == PRODUCT_STRING.encode('ascii') + b'\x00'
    print(f"Result: {buffer.decode('ascii')}\n")

    # Check if firmware version is compatible
    print("Reading vendor data...")
    ser.write(UART_CMD.CMD_READ_VENDOR_DATA.toByte())
    buffer = ser.read(3)
    assert len(buffer) == 3
    vendor_data = VendorDataStruct.from_buffer_copy(buffer)
    print(f"Result: {vendor_data}\n")
    assert vendor_data.VendorId == VENDOR_ID and vendor_data.ProductId == PRODUCT_ID

    if(vendor_data.FwVersion != FIRMWARE_VERSION):
        print(f"Warning: Firmware version {hex(vendor_data.FwVersion)} may not be compatible with this script (for version {hex(FIRMWARE_VERSION)}).\n")
        # Wait for user input
        input("Press Enter to continue...")

    ser.read_all()

    num_samples = 500

    while(True):
        
        #print("Reading sensor values...")
        
        sensor_struct = SensorStruct()

        voltage_array = np.zeros((SENSOR_POWER_NUM, num_samples))
        current_array = np.zeros((SENSOR_POWER_NUM, num_samples))
        power_array = np.zeros((SENSOR_POWER_NUM, num_samples))

        for i in range(num_samples):
            
            success = False
            while not success:
                try:
                    ser.write(UART_CMD.CMD_READ_SENSOR_VALUES.toByte())
                    buffer = ser.read(sizeof(SensorStruct))
                    if len(buffer) == sizeof(SensorStruct):
                        success = True
                except:
                    time.sleep(0.1)

            sensor_struct_single = SensorStruct.from_buffer_copy(buffer)
            for j in range(SENSOR_POWER_NUM):
                voltage_array[j][i] = sensor_struct_single.PowerReadings[j].Voltage
                current_array[j][i] = sensor_struct_single.PowerReadings[j].Current
                power_array[j][i] = sensor_struct_single.PowerReadings[j].Power
            

        for j in range(SENSOR_POWER_NUM):
            sensor_struct.PowerReadings[j].Voltage = int(np.average(voltage_array[j]))
            sensor_struct.PowerReadings[j].Current = int(np.average(current_array[j]))
            sensor_struct.PowerReadings[j].Power = int(np.average(power_array[j]))

        power_readings = sensor_struct.PowerReadings
        print("ATX24\t\t12V\t5V\t5VSB\t3.3V")
        print(f'{"Voltage"}\t\t{power_readings[0].Voltage/1000:.3f}\t{power_readings[1].Voltage/1000:.3f}\t{power_readings[2].Voltage/1000:.3f}\t{power_readings[3].Voltage/1000:.3f}')
        print(f'{"Current"}\t\t{power_readings[0].Current/1000:.3f}\t{power_readings[1].Current/1000:.3f}\t{power_readings[2].Current/1000:.3f}\t{power_readings[3].Current/1000:.3f}')
        print(f'{"Power"}\t\t{power_readings[0].Power/1000:.3f}\t{power_readings[1].Power/1000:.3f}\t{power_readings[2].Power/1000:.3f}\t{power_readings[3].Power/1000:.3f}')
        print("\n")

        print("\t\tEPS1\tEPS2")
        print(f'{"Voltage"}\t\t{power_readings[5].Voltage/1000:.3f}\t{power_readings[6].Voltage/1000:.3f}')
        print(f'{"Current"}\t\t{power_readings[5].Current/1000:.3f}\t{power_readings[6].Current/1000:.3f}')
        print(f'{"Power"}\t\t{power_readings[5].Power/1000:.3f}\t{power_readings[6].Power/1000:.3f}')
        print("\n")

        print("\t\tPCIE1\tPCIE2\tPCIE3\tHPWR1")
        print(f'{"Voltage"}\t\t{power_readings[7].Voltage/1000:.3f}\t{power_readings[8].Voltage/1000:.3f}\t{power_readings[9].Voltage/1000:.3f}\t{power_readings[4].Voltage/1000:.3f}')
        print(f'{"Current"}\t\t{power_readings[7].Current/1000:.3f}\t{power_readings[8].Current/1000:.3f}\t{power_readings[9].Current/1000:.3f}\t{power_readings[4].Current/1000:.3f}')
        print(f'{"Power"}\t\t{power_readings[7].Power/1000:.3f}\t{power_readings[8].Power/1000:.3f}\t{power_readings[9].Power/1000:.3f}\t{power_readings[4].Power/1000:.3f}')
        print("\n\n")

        #power_cpu = (power_readings[0].Power + power_readings[1].Power)/1000
        #power_gpu = (power_readings[6].Power + power_readings[7].Power + power_readings[8].Power + power_readings[9].Power + power_readings[10].Power)/1000
        #power_mb = (power_readings[2].Power + power_readings[3].Power + power_readings[4].Power + power_readings[5].Power)/1000
        #power_system = power_cpu + power_gpu + power_mb

        time.sleep(0.5)


