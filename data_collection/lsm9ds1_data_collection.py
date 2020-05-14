# read the sensor and dump the contents into a CSV file for analysis
import csv
import time
import RPi.GPIO as GPIO
import board
import busio
import adafruit_lsm9ds1
 
# I2C connection:
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

global collecting
collecting = False

# set up hardware button to quit
GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)
# call back for exit
def leave(pin):
    GPIO.cleanup()
    quit()
def start(pin):
    global collecting
    if collecting:
        collecting = False
    else:
        collecting = True
    
GPIO.add_event_detect(27, GPIO.FALLING, callback=leave)
GPIO.add_event_detect(17, GPIO.FALLING, callback=start)

# set accerometer range, it should default to 2G
sensor.accel_range = adafruit_lsm9ds1.ACCELRANGE_2G

print("Starting IMU collect (press 17 to start/stop)")
# a bad way to do this
while not collecting:
    time.sleep(0.2)

# Main loop will read data values
data = [0,0,0,0,0,0,0]
# wait a second to stablalize
time.sleep(0.5)
# make a file name
now = time.localtime()
file_name = str(now.tm_mon) + "-" + str(now.tm_mday) + "-" + str(now.tm_hour) + "-" + str(now.tm_min) + "-" + str(now.tm_sec)+ "-" + "imu-data.csv"

print('IMU started.')
with open(file_name, 'w') as f:
    writer = csv.writer(f)
    print("don't move")
    for i in range(12000):
        if i == 10:
            print("start moving")
        data[0] = time.clock_gettime(time.CLOCK_MONOTONIC_RAW)
        data[1:4] = sensor.acceleration
        data[4:] = sensor.gyro
        #mag_x, mag_y, mag_z = sensor.magnetic
        writer.writerow(data)
        # I don't want to quit from the call back because I'm scared about
        # quiting while writing to a file. If I was smart I would use
        # locks and the file close methode, but this is good enough for
        # goverment work.
        if collecting == False:
            break
        time.sleep(0.05)
GPIO.cleanup()
print('IMU done.')