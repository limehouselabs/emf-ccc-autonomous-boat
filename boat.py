###
### This script will point the boat towards a target location and drive towards it
###

import time
import math
import board
import adafruit_lsm303dlh_mag
import numpy as np
from geopy.geocoders import Nominatim
from gpiozero import Servo, Motor
import gpsd  # the gpsd interface module
import pyproj
import smbus
import lsm303
import argparse
import requests

## Config
DEFAULT_TARGET_LAT = 0.0
DEFAULT_TARGET_LON = 0.0

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
centre = 0.2
right = 0.7

servo.value = centre

############


def get_target_heading(current_lat, current_lon):
    # Read the target heading
    fwd_azimuth, back_azimuth, distance = geodesic.inv(
        TARGET_LON, TARGET_LAT, current_lon, current_lat
    )
    target_heading = fwd_azimuth

    fixed_target_heading = target_heading + 180
    if fixed_target_heading < 180:
        fixed_target_heading += 360
    elif fixed_target_heading > 180:
        fixed_target_heading -= 360
    print("Target heading: {0:10.3f}, calling it {1:10.3f}".format(target_heading, fixed_target_heading))

    return fixed_target_heading


def get_current_location():
    gpsd.connect()
    gps_packet = gpsd.get_current()
    print(gps_packet.get_time())

    if gps_packet.mode < 2:
        reset_boat()
        print("No GPS lock, waiting for better signal")
        return None, None

    precision_x_y, precision_z = gps_packet.position_precision()
    if precision_x_y > 15:
        reset_boat()
        print("Poor GPS lock, waiting for better signal (%s)" % precision_x_y)
        return None, None

    if math.isfinite(gps_packet.lat) and math.isfinite(gps_packet.lat):
        print("Lat %.6f Lon %.6f" % (gps_packet.lat, gps_packet.lon))
    else:
        print("Lat n/a Lon n/a")

    return gps_packet.lat, gps_packet.lon

def get_current_heading():
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
    if heading < -180:
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
parser.add_argument("-lat", "--latitude", default=DEFAULT_TARGET_LAT, type=float)
parser.add_argument("-lon", "--longitude", default=DEFAULT_TARGET_LON, type=float)
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
    current_heading = get_current_heading()
    recent_current_headings.append(current_heading)
    recent_current_headings = recent_current_headings[-10:]

    average_current_heading = sum(recent_current_headings) / len(
        recent_current_headings
    )

    print(f"Average current heading: {average_current_heading:.2f}")

    ## Point Boat
    difference_in_heading = average_current_heading - average_target_heading
    turn_boat(difference_in_heading)

    ## Set speed
    fwd_azimuth, back_azimuth, distance = geodesic.inv(
        current_lon, current_lat, TARGET_LON, TARGET_LAT
    )
    print("Distance to target: {0:10.3f}".format(distance))
    if distance < 1 or (TARGET_LAT == 0 and TARGET_LON == 0):
        motor.stop()
        print("Within 1m of target, stopping")

        response = requests.post("http://127.0.0.1:8000/boat/remove_target", json={
            "lat": TARGET_LAT,
            "lon": TARGET_LON,
        })

    else:
        max_speed_after = 5
        speed_percentage = min([distance / max_speed_after, 1])

        motor.backward(MOTOR_SPEED * speed_percentage)

        print("Settings speed to: {0:10.3f}".format(speed_percentage))

    response = requests.post("http://127.0.0.1:8000/boat/update", json={
        "lat": current_lat,
        "lon": current_lon,
        "heading": current_heading,
    })

    targets = response.json()["targets"]
    if targets:
        TARGET_LAT, TARGET_LON = targets[0]
    else:
        TARGET_LAT, TARGET_LON = 0.0, 0.0
        motor.stop()

    time.sleep(0.5)

