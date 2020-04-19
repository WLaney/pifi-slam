#!/usr/bin/env python3
# calcuate statistics on IMU dumped data
import numpy as np

# file path to data file
path = "../../final_project/data/luke-10000-imu-test-data.csv"

# import csv straight to a numpy array
data = np.genfromtxt(path, delimiter=',')
collum_names  = ['ax', 'ay', 'az', 'wx', 'wy', 'wz']
var = np.var(data, 0)
mean = np.mean(data, 0)
print(collum_names)
print("varience")
print(var)
print("mean")
print(mean)
