import time
import smbus
import lsm303

i2c_channel = 1
bus = smbus.SMBus(i2c_channel)

# Will raise OSError if device is not connected
device = lsm303.LSM303(bus)

max_x = max_y = max_z = 0
min_x = min_y = min_z = 0

while True:
    mag_data = device.read_mag()
    x, y, z = mag_data
    max_x = max(x, max_x)
    max_y = max(y, max_y)
    max_z = max(z, max_z)
    min_x = min(x, min_x)
    min_y = min(y, min_y)
    min_z = min(z, min_z)

    print(
            f"avg_x = ({min_x:.2f} + {max_x:.2f}) / 2\
            avg_y = ({min_y:.2f} + {max_y:.2f}) / 2\
            avg_z = ({min_z:.2f} + {max_z:.2f}) / 2"
    )

    time.sleep(0.1)

