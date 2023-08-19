###
### This script will point the boat towards a target location and drive towards it
###

import time
import math
import board
import adafruit_lsm303dlh_mag
from geopy.geocoders import Nominatim
from gpiozero import Servo, Motor
import gpsd  # the gpsd interface module
import pyproj
import smbus
import lsm303
import argparse

## Config
DEFAULT_TARGET_LAT = 53.034139
DEFAULT_TARGET_LON = 13.307036

SERVO_PIN = 24

MOTOR_SPEED = 1  # 0-1
MOTOR_BRIDGE_PIN_A = 16
MOTOR_BRIDGE_PIN_B = 19

## GPS Maths
geodesic = pyproj.Geod(ellps="WGS84")
loc = Nominatim(user_agent="GetLoc")

## GPSd location
gpsd.connect()

## Magnetometer
i2c_channel = 1
bus = smbus.SMBus(i2c_channel)
device = lsm303.LSM303(bus)  # Will raise OSError if device is not connected

# Finger-in-the-air calibration
#avg_x = (50 + -20) / 2
#avg_y = (60 + -25) / 2
#avg_z = (2 + -5) / 2

# Actual calibration from calibrate.py
avg_x = (-56.18 + 49.64) / 2
avg_y = (-37.91 + 68.18) / 2
avg_z = (-4.80 + 2.24) / 2

## Motor
motor = Motor(MOTOR_BRIDGE_PIN_A, MOTOR_BRIDGE_PIN_B)

## Servo
servo = Servo(SERVO_PIN)

left = -0.5
centre = 0.25
right = 0.8

servo.value = centre

############


def get_target_heading(current_lat, current_lon):
    # Read the target heading
    fwd_azimuth, back_azimuth, distance = geodesic.inv(
        TARGET_LON, TARGET_LAT, current_lon, current_lat
    )
    target_heading = fwd_azimuth

    print("Target heading: {0:10.3f}".format(target_heading))

    return target_heading


def get_current_location():
    gps_packet = gpsd.get_current()

    if gps_packet.mode < 2:
        reset_boat()
        print("No GPS lock, waiting for better signal")
        return None, None

    precision_x_y, precision_z = gps_packet.position_precision()
    if precision_x_y > 10:
        reset_boat()
        print("Poor GPS lock, waiting for better signal (%s)" % precision_x_y)
        return None, None

    if math.isfinite(gps_packet.lat) and math.isfinite(gps_packet.lat):
        print("Lat %.6f Lon %.6f" % (gps_packet.lat, gps_packet.lon))
    else:
        print("Lat n/a Lon n/a")

    return gps_packet.lat, gps_packet.lon

def get_current_heading()
    orientation = [0, -1, 0]  # the board is y backwards, x right according to the ship
    acc_data = device.read_accel()
    mag_data = device.read_mag()

    # Based on https://github.com/pololu/lsm303-arduino/blob/be88750/LSM303.h#L228
    mag_x, mag_y, mag_z = mag_data

    temp = [mag_x - avg_x, mag_y - avg_y, mag_z - avg_z]

    e = np.cross(temp, acc_data)
    e /= np.linalg.norm(e)
    n = np.cross(acc_data, e)
    n /= np.linalg.norm(n)

    #print(f"east: {e}, north: {n}")

    heading = math.degrees(math.atan2(np.dot(e, orientation), np.dot(n, orientation)))
    if heading < 0:
        heading += 360

    return heading

def reset_boat():
    servo.value = centre
    motor.stop()

    print("Resetting boat")


def turn_boat(degrees_to_change_by):
    # 0 = straight
    # 90 = left
    # -90 = right

    # Turning left
    if degrees_to_change_by > 0:
        print("Turning left by {0:10.3f} degrees".format(degrees_to_change_by))

        # Percentage left
        if degrees_to_change_by < 90:
            percentage_difference_in_heading = degrees_to_change_by / 90

            servo_position = centre - left + left * percentage_difference_in_heading

            servo.value = servo_position

            print(
                "Angling servo to {0:10.3f} percent left".format(
                    percentage_difference_in_heading
                )
            )

        # Full lock left
        else:
            servo.value = left

            print("Full lock left")

    # Turning right
    elif degrees_to_change_by < 0:
        print("Turning right by {0:10.3f} degrees".format(degrees_to_change_by))

        # Percentage right
        if degrees_to_change_by > -90:
            percentage_difference_in_heading = degrees_to_change_by / 90

            servo_position = (
                centre + (right - centre) * percentage_difference_in_heading
            )

            servo.value = servo_position

            print(
                "Angling servo to {0:10.3f} percent right".format(
                    percentage_difference_in_heading
                )
            )

        # Full lock right
        else:
            servo.value = right

            print("Full lock right")

    # Straight
    else:
        servo.value = centre

        print("Forwards")


############ Main Loop

parser = argparse.ArgumentParser()
parser.add_argument("-lat", "--latitude", default=DEFAULT_TARGET_LAT)
parser.add_argument("-lon", "--longitude", default=DEFAULT_TARGET_LON)
args = parser.parse_args()

TARGET_LAT = args.latitude
TARGET_LON = args.longitude

print("Target location is {0:10.3f}, {0:10.3f}".format(TARGET_LAT, TARGET_LON))

recent_target_headings = []
recent_current_headings = []
print("Waking boat")
reset_boat()
while True:
    # Target heading
    current_lat, current_lon = get_current_location()
    if not current_lat or not current_lon:
        print("No GPS signal")
        reset_boat()
        continue

    current_target_heading = get_target_heading(current_lat, current_lon)
    if not current_target_heading:
        print("No heading from magnetrometer")
        reset_boat()
        continue

    recent_target_headings.append(current_target_heading)
    recent_target_headings = recent_target_headings[-10:]

    average_target_heading = sum(recent_target_headings) / len(
        recent_target_headings
    )

    # Current heading
    recent_current_headings.append(get_current_heading())
    recent_current_headings = recent_current_headings[-10:]

    average_current_heading = sum(recent_current_headings) / len(
        recent_current_headings
    )

    ## Point Boat
    difference_in_heading = average_current_heading - average_target_heading
    turn_boat(difference_in_heading)

    ## Set speed
    fwd_azimuth, back_azimuth, distance = geodesic.inv(
        current_lon, current_lat, TARGET_LON, TARGET_LAT
    )
    print("Distance to target: {0:10.3f}".format(distance))
    if distance < 1:
        motor.stop()
        print("Within 1m of target, stopping")

    else:
        max_speed_after = 5
        speed_percentage = min([distance / max_speed_after, 1])

        motor.backward(MOTOR_SPEED * speed_percentage)

        print("Settings speed to: {0:10.3f}".format(speed_percentage))

    time.sleep(0.5)

