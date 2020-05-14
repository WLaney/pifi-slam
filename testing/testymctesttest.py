#!/usr/bin/env python3
import wifislam
import numpy as np
from analysis.data_assosation import data_assosation
import analysis.dead_reckoning as dead_reckoning

drmath = dead_reckoning.deadReckoningMath()
imu_file = 'analysis/imu_and_wifi/5-14-12-32-15-imu-data.csv'
wifi_file = 'analysis/imu_and_wifi/wifi_data.csv'

movement_data, wifi_data = data_assosation(imu_file, wifi_file)

drmath.plot_trajectory(movement_data)

slam = wifislam.Slam(movement_data, wifi_data)
solution = slam.solve_slam()

cords = drmath.rd2xy(solution[:,0], solution[:,1])
drmath.plot_trajectory(cords)
drmath.show_plots()