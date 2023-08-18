###
### This script will point the boat towards a target location and drive towards it
###

import time
import board
import adafruit_lsm303dlh_mag
from geopy.geocoders import Nominatim
import robohat
import RPi.GPIO as gpio
import gpsd  # the gpsd interface module
import pyproj

## Config
TARGET_LAT = 53.032986
TARGET_LON = 13.301234

SERVO_PIN = 22
MOTOR_SPEED = 10

## GPS Maths
geodesic = pyproj.Geod(ellps="WGS84")
loc = Nominatim(user_agent="GetLoc")

## GPSd location
gpsd.connect()

## Magnetometer
i2c = board.I2C()  # uses board.SCL and board.SDA
sensor = adafruit_lsm303dlh_mag.LSM303DLH_Mag(i2c)

## Motor
robohat.init()

## Servo
gpio.setmode(gpio.BOARD)
gpio.setup(SERVO_PIN, gpio.OUT)
p = gpio.PWM(SERVO_PIN, 200)

left = 50 / 5  # Frequency is 500Hz, so each pulse is 5ms wide
centre = (
    150 / 5
)  # Servos will be fully left at 0.5ms, centred at 1.5ms and fully right at 2.5ms
right = 250 / 5

p.start(centre)  # start it at 50% - should be centre of servo

############


def get_target_heading(current_lat, current_lon):
    # Read the target heading
    fwd_azimuth, back_azimuth, distance = geodesic.inv(
        TARGET_LON, TARGET_LAT, current_lon, current_lat
    )
    target_heading = fwd_azimuth

    print("Target heading: {0:10.3f}".format(target_heading))


def get_current_location():
    gps_packet = gpsd.get_current()

    if (gps_packet.mode < 2):
      reset_boat()
      print("No GPS lock, waiting for better signal")
      return null, null

    if (gps_packet.position_precision < 10):
      reset_boat()
      print("Poor GPS lock, waiting for better signal")
      return null, null

    if gps.isfinite(gps_packet.lat) and gps.isfinite(gps_packet.lat):
        print(" Lat %.6f Lon %.6f" % (gps_packet.lat, gps_packet.lon))
    else:
        print(" Lat n/a Lon n/a")

    return gps_packet.lat, gps_packet.lon


def vector_2_degrees(x, y):
    angle = degrees(atan2(y, x))
    if angle < 0:
        angle += 360
    return angle


def get_current_heading():
    magnet_x, magnet_y, _ = sensor.magnetic
    print(
        "Magnetometer (gauss): ({0:10.3f}, {1:10.3f}, {2:10.3f})".format(
            mag_x, mag_y, mag_z
        )
    )
    return vector_2_degrees(magnet_x, magnet_y)


def reset_boat():
    p.changeDutyCycle(centre)  # Centre the servo
    robohat.stop()  # Stop the motors


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

            p.ChangeDutyCycle(servo_position)

            print(
                "Angling servo to {0:10.3f} percent left".format(
                    percentage_difference_in_heading
                )
            )

        # Full lock left
        else:
            p.ChangeDutyCycle(left)

            print("Full lock left")

    # Turning right
    elif degrees_to_change_by < 0:
        print("Turning right by {0:10.3f} degrees".format(degrees_to_change_by))

        # Percentage right
        if degrees_to_change_by > -90:
            percentage_difference_in_heading = degrees_to_change_by / 90

            servo_position = (
                center + (right - center) * percentage_difference_in_heading
            )

            p.ChangeDutyCycle(servo_position)

            print(
                "Angling servo to {0:10.3f} percent right".format(
                    percentage_difference_in_heading
                )
            )

        # Full lock right
        else:
            p.ChangeDutyCycle(right)

            print("Full lock right")

    # Straight
    else:
        p.ChangeDutyCycle(centre)

        print("Forwards")


############ Main Loop
recent_target_headings = []
recent_current_headings = []
try:
    reset_boat()
    while True:
        # Target heading
        current_lat, current_lon = get_current_location()
        if not current_lat or not current_lon:
            print("No GPS signal")
            reset_boat()
            continue

        recent_target_headings.append(get_target_heading(current_lat, current_lon))
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
        if distance < 1:
            robohat.forward(stop)
            print("Within 1m of target, stopping")

        else:
            max_speed_after = 5
            speed_percentage = min([distance / max_speed_after, 1])

            robohat.forward(MOTOR_SPEED * speed_percentage)

            print("Settings speed to: {0:10.3f}".format(speed_percentage))

        time.sleep(0.5)

except KeyboardInterrupt:
    print("Closing")

finally:
    robohat.cleanup()
    session.close()
    exit(0)
