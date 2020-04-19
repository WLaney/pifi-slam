# figure out the time per read of LSM9DS1 circuit python library
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
start = time.time()
for i in range(1000):
    accel_x, accel_y, accel_z = sensor.acceleration
    gyro_x, gyro_y, gyro_z = sensor.gyro
    #mag_x, mag_y, mag_z = sensor.magnetic
print(time.time() - start)
