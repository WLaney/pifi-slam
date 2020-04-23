#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt

class deadReckoning:
    def __init__(self, data_file):
        data = np.genfromtxt(data_file, delimiter=',')
        # csv format: time, ax, ay, az, wx, wy, wz
        self.time = data[:,0]
        self.ax = data[:,1]
        self.ay = data[:,2]
        self.az = data[:,3]
        self.wx = data[:,4]
        self.wy = data[:,5]
        self.wz = data[:,6]
        del data  # superstion is fun
        
    def get_positions(self):
        pass
        
    def plot_positions(self):
        pass
        
    def plot_raw_acceration(self):
        plt.plot(self.time, self.ax, self.time, self.ay, self.time, self.az)
        plt.xlabel('Time')
        plt.ylabel('Acceration (m/s^2)')
        plt.legend(['ax', 'ay', 'az'])
        plt.title('Input Acceration vs Time')
        plt.show()
    
    def plot_raw_omega(self):
        plt.plot(self.time, self.wx, self.time, self.wy, self.time, self.wz)
        plt.xlabel('Time')
        plt.ylabel('Angular Velocity (deg/s)')
        plt.legend(['wx', 'wy', 'wz'])
        plt.title('Input Angular Velocity vs Time')
        plt.show()