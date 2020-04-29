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
        self.wx = np.deg2rad(data[:,4])
        self.wy =  np.deg2rad(data[:,5])
        self.wz =  np.deg2rad(data[:,6])
        del data  # superstion is fun
        
    def data_stats(self):
        print("Number of data reads: " + str(self.time.size))
        print("Time accounted for: " + str(self.time[-1] - self.time[1]))
        delta_t = self.time[1:] - self.time[:-1]
        print("Max time between readings: " + str(max(delta_t)))
        mean_delta_t = np.mean(delta_t)
        print("Average time between readings: " + str(mean_delta_t))
        print("Meadian time between readings: " + str(np.median(delta_t)))
        #print("Average Reading Frequency: " + str(1/mean_delta_t))
        
    def displacment_from_axis(self, axis):
        # we do a window moving average, this is the number of samples in each direction from the 
        # point of intest
        win_size = 3
        # we start taking data before we start moving for callibration
        start_offset = 10
        #combine all the indexing offsets
        total_offset = win_size + start_offset
        # we start not moving
        v_prev = 0
        
        # we need the first item in ax for intial time, so we always have ax-1 measurments
        # in this contexted the displacment is the distance in each time step, not the total distance traveld
        displacment = np.zeros(axis.size - total_offset)
        self.velocity = np.zeros(axis.size - total_offset)
        self.total_displacment = np.zeros(axis.size - total_offset - 3)
        #axis -= np.mean(axis[:10]) # this needs to be improved
        for i in range(start_offset, axis.size-win_size):
            #delta_t = 0.05
            delta_t = self.time[i] - self.time[i-1]
            # this is a simple rolling average
            accel = np.mean(axis[i-win_size:i+win_size+1])
            displacment[i - total_offset] = (v_prev * delta_t) #+ (0.5 * accel * delta_t**2)
            v_prev += (accel * delta_t)
            self.velocity[i-total_offset] = v_prev
            self.total_displacment[i - total_offset] = displacment[i - total_offset] + self.total_displacment[i - (total_offset + 1)]
            
        return displacment
        
    def rotation_from_wz(self):
        # in this contexted the displacment is the distance in each time step, not the total distance traveld
        angle = np.zeros(self.wz.size - 1)
        for i in range(1, self.wz.size):
            delta_t = self.time[i] - self.time[i-1]
            angle[i - 1] = angle[i - 2] + (self.wz[i] * delta_t)
                
        return angle
            
        
    def plot_trajectory(self):
        pass
        
    def plot_accleration(self, axis='all'):
        plt.figure(1) # there is almost deffiently a better way to have multiple plots
        if axis=='all':
            plt.plot(self.time, self.ax, '.', self.time, self.ay, '.', self.time, self.az, '.')
            plt.legend(['ax', 'ay', 'az'])
        elif axis=='x':
            plt.plot(self.time, self.ax, '-')
            plt.legend(['ax'])
        elif axis=='y':
            plt.plot(self.time, self.ay, '-')
            plt.legend(['ay'])
        elif axis=='z':
            plt.plot(self.time, self.az, '-')
            plt.legend(['az'])
        else:
            print("Please spefic 'all', 'x', 'y', or 'z'")
        plt.xlabel('Time')
        plt.ylabel('Acceration (m/s^2)')
        plt.title('Input Acceration vs Time')
        plt.show(block=False)
    
    def plot_omega(self):
        plt.figure(2)
        plt.plot(self.time, self.wx, self.time, self.wy, self.time, self.wz)
        plt.xlabel('Time')
        plt.ylabel('Angular Velocity (deg/s)')
        plt.legend(['wx', 'wy', 'wz'])
        plt.title('Input Angular Velocity vs Time')
        plt.show(block=False)
        
    def plot_velocity(self):
        plt.figure(3)
        plt.plot(self.velocity, '-')
        plt.xlabel('Samples')
        plt.ylabel('Velocity (m/s)')
        plt.title('Velcocoty vs Samples')
        plt.show(block=False)
        
    def plot_distance(self):
        plt.figure(4)
        plt.plot(self.total_displacment, '*')
        plt.xlabel('Samples')
        plt.ylabel('Total Displacment (m)')
        plt.title('Displacment vs Samples')
        plt.show(block=False)
    def show_plots(self):
        plt.show()