# read the sensor and dump the contents into a CSV file for analysis
import csv
import time
import board
import busio
import adafruit_lsm9ds1
 
# I2C connection:
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)
 
# Main loop will read the acceleration, magnetometer, gyroscope, Temperature
# values every second and print them out.
# Read acceleration, magnetometer, gyroscope, temperature.
data = [0,0,0,0,0,0]
# wait a second to get away from the device
time.sleep(3)
print('start')
with open('test-data.csv', 'w') as f:
    writer = csv.writer(f)
    for i in range(10000):
        data[0:3] = sensor.acceleration
        data[3:] = sensor.gyro
        #mag_x, mag_y, mag_z = sensor.magnetic
        writer.writerow(data)
print('done')
