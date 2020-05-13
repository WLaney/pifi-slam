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
        
        # set up some constants
        # TODO make these input variables
        # we do a window moving average, this is the number of samples in each direction from the 
        # point of intest
        self.win_size = 3
        # we start taking data before we start moving for callibration
        # we need to offset our loop counter by this amount when writng to new variables
        self.start_offset = 10
        #combine all the indexing offsets
        # ths determines the total size of our new arrays
        self.total_offset = self.win_size + self.start_offset
        
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
        # we start not moving
        v_prev = 0
        
        # we need the first item in ax for intial time, so we always have ax-1 measurments
        # in this contexted the displacment is the distance in each time step, not the total distance traveld
        
        # we add an extra zero at the end end to make indexing match up with the gyro/angle stuff
        # I don't love it but I think it's what we need to do to make everything work
        displacment = np.zeros(axis.size - self.total_offset + 1)
        self.velocity = np.zeros(axis.size - self.total_offset)
        self.total_displacment = np.zeros(axis.size - self.total_offset + 1)
        axis -= np.mean(axis[0:self.start_offset]) # this needs to be improved
        for i in range(self.start_offset, axis.size-self.win_size):
            #delta_t = 0.05
            delta_t = self.time[i] - self.time[i-1]
            # this is a simple rolling average
            accel = np.mean(axis[i-self.win_size:i+self.win_size+1])
            displacment[i - self.start_offset] = (v_prev * delta_t) #+ (0.5 * accel * delta_t**2)
            v_prev += (accel * delta_t)
            self.velocity[i-self.start_offset] = v_prev
            self.total_displacment[i - self.start_offset] = \
                displacment[i - self.start_offset] + self.total_displacment[i - self.start_offset - 1]
            # manufaly add the last point in to total displacment so plots look good
            self.total_displacment[-1] = self.total_displacment[-2]
        return displacment
        
    def rotation_from_wz(self):
        # in this contexted the displacment is the distance in each time step, not the total distance traveld
        # the intial angle must be zero, and we should have one more angle than we do gyro measurments
        angle = np.zeros(self.wz.size - self.total_offset + 1)
        for i in range(self.start_offset + 1, self.wz.size-self.win_size):
            delta_t = self.time[i] - self.time[i-1]
            angle[i - self.start_offset] = angle[i - self.start_offset - 1] + (self.wz[i] * delta_t)
                
        return angle
        
    def plot_accleration(self, axis='all'):
        plt.figure() # there is almost deffiently a better way to have multiple plots
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
        plt.figure()
        plt.plot(self.time, self.wx, self.time, self.wy, self.time, self.wz)
        plt.xlabel('Time')
        plt.ylabel('Angular Velocity (rad/s)')
        plt.legend(['wx', 'wy', 'wz'])
        plt.title('Input Angular Velocity vs Time')
        plt.show(block=False)
        
    def plot_velocity(self):
        plt.figure()
        plt.plot(self.velocity, '.')
        plt.xlabel('Samples')
        plt.ylabel('Velocity (m/s)')
        plt.title('Velcocoty vs Samples')
        plt.show(block=False)
        
    def plot_distance(self):
        plt.figure()
        plt.plot(self.total_displacment, '*')
        plt.xlabel('Samples')
        plt.ylabel('Total Displacment (m)')
        plt.title('Displacment vs Samples')
        plt.show(block=False)
        
    def show_plots(self):
        # this feels stupid but I don't know a better way to get the behavior I want
        plt.show()
        
class deadReckoningMath:
    """A class for math function related to dead reckonging that we will need in other modules"""
    def rd2xy(self, displacment, angle):
        """Change dispacment and roation into xy cordinates
        
        Inputs:
            displcament: distances Nx1
            angels: nx1
        Output:
            X-Y postions, N-1x2"""
        
        assert displacment.size == angle.size
        xy_cords = np.zeros([displacment.size - 1, 2])
        xy_cords[0, 0] = (displacment[0] * np.cos(angle[0]))
        xy_cords[0, 1] = (displacment[0] * np.sin(angle[0]))
        for i in range(1, displacment.size - 1):
            xy_cords[i, 0] = xy_cords[i-1, 0] + (displacment[i] * np.cos(angle[i]))
            xy_cords[i, 1] = xy_cords[i-1, 1] + (displacment[i] * np.sin(angle[i]))
        return xy_cords
        
    def plot_trajectory(self, xy_cords):
        plt.figure()
        plt.plot(xy_cords[:,0], xy_cords[:,1], '-.')
        plt.title("Trajectory")
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.show(block=False)
    