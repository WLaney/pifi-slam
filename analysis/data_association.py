#!/usr/bin/env python3
import analysis.dead_reckoning as dead_reckoning
import numpy as np

def data_association(imu_file, wifi_file):
    """Take IMU and WIFI data files and prepare them for use in WiFi SLAM
    
    Inputs:
        imu_file: String of a file path to the IMU data CSV
        wifi_file: String of a file path to the WIFI data CSV
    
    Outputs:
        measurment_data: Nx2 matrix where N is the number of wifi samples.
                        The first collum is distance and the second is gyro measurs
        wifi_data: NxM matrix where N is the number of WiFi samples, and M is the total number
                    of wireless acces points seen
    """
    dr = dead_reckoning.deadReckoning(imu_file)
    distance = dr.displacment_from_axis(dr.ax)
    gyro = dr.used_data(dr.wz)
    imu_times = dr.used_data(dr.time)

    wifi_data = np.genfromtxt(wifi_file, delimiter=',')
    wifi_times = wifi_data[:,0]

    # get rid of data from after one of the two proccess ended
    if wifi_times[-1] > imu_times[-1]:
        out_of_time_index = wifi_times <= imu_times[-1]
        wifi_times = wifi_times[out_of_time_index]
        wifi_data = wifi_data[out_of_time_index, :]
    else:
        out_of_time_index = imu_times <= wifi_times[-1]
        imu_times = imu_times[out_of_time_index]
        distance = distance[out_of_time_index]
        gyro = gyro[out_of_time_index]

    # We imu data before the first wifi measurment
    # The other option is to have the first wifi measurment assosated with a 0 distance and 0 gyro
    # reading. I don't really know what option is better
    while wifi_times[0] < imu_times[0]:
        # delte the first wifi time stamp
        wifi_times = np.delete(wifi_times, 0)
        # delte the first row of wifi data
        wifi_data = np.delete(wifi_data, 0, 0)
    
        if wifi_times.size == 0:
            raise("Can not assoate WiFi times with IMU times")

    # convert dBm to mW
    wifi_data_dbm   = wifi_data[:,1:]
    wifi_data_mw    = np.power(10, wifi_data_dbm/10.0)

    # create measurment data vector, it needs to be the same size as the wifi times
    measurment_data = np.empty([wifi_times.size, 2])

    # fill the first entries of measurment data with values from before the first wifi scan
    measurment_data[0,0] = np.sum(distance[imu_times <= wifi_times[0]])
    measurment_data[0,1] = np.mean(gyro[imu_times <= wifi_times[0]])

    # now fill in the rest of the matrix
    for i in range(1, wifi_times.size):
        time_slices = (imu_times > wifi_times[i-1]) & (imu_times <= wifi_times[i])
        measurment_data[i,0] = np.sum(distance[time_slices])
        measurment_data[i,1] = np.mean(gyro[time_slices])

    return measurment_data, wifi_data_mw
    
if __name__ == "__main__":
    # this is for testing, the funtion is this file should be called by the controling script
    imu_file = 'imu_and_wifi/5-14-12-32-15-imu-data.csv'
    wifi_file = 'imu_and_wifi/wifi_data.csv'
    md, wd = data_assosation(imu_file, wifi_file)
    print(md)
    print(wd)
