#!/usr/bin/env python3

import os
import time
from multiprocessing import Pool

from analysis.data_association import data_association
import analysis.wifislam as wifislam
import analysis.dead_reckoning as dr
from analysis.tft_plot import TFTplotting

now = time.localtime()
now_string = str(now.tm_mon) + "-" + str(now.tm_mday) + "-" + str(now.tm_hour) + "-" + str(now.tm_min) + "-" + str(now.tm_sec)
dir_name = "/home/pi/ece5725project/data/" + now_string + "/"
os.makedirs(dir_name)
scripts = ('data_collection/wifi_data_collection.py ' + dir_name,
        'data_collection/lsm9ds1_data_collection.py ' + dir_name)

def run_script(script):
    os.system('python3 {}'.format(script))

pool = Pool(2)
pool.map(run_script, scripts)

print("FINISHED BOTH")
print("Data written to " + dir_name)

wifi_data_path = dir_name + "wifi_data.csv"
imu_data_path = dir_name + "imu_data.csv"

measurment_data, wifi_data = data_association(imu_data_path, wifi_data_path)

print("Solving Slam...")
slam = wifislam.Slam(measurment_data, wifi_data)
slam_positions = slam.solve_slam()

drmath = dr.deadReckoningMath()
slamXY = drmath.rd2xy(slam_positions[:,0], slam_positions[:,1])
slam_plot_path = dir_name + "wifi_slam_traj.png"
drmath.plot_trajectory(slamXY, save=slam_plot_path, title="SLAM Trajectory")

print("Preforming Dead Reckoning")
dr_solver = dr.deadReckoning(imu_data_path)
dr_displacment = dr_solver.displacment_from_axis(dr_solver.ax)
dr_angle = dr_solver.rotation_from_wz()
drXY = drmath.rd2xy(dr_displacment, dr_angle)
dr_plot_path = dir_name + "dr_traj.png"
drmath.plot_trajectory(drXY, save=dr_plot_path, title="Dead Reckoning Trajectory")

tft_plotter = TFTplotting([slam_plot_path, dr_plot_path])
tft_plotter.show_plots()
print("DONE!!")
