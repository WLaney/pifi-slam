#!/usr/bin/env python3

import os
import time
from multiprocessing import Pool

from analysis.data_association import data_association

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
