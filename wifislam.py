#!/usr/bin/env python3

from analysis import dead_reckoning
import numpy as np

class Slam:

    def __init__(self, movement_data, wifi_data):
        
        # input data
        # movment data is a Nx2 matrix where N is the number of time steps
        # the first collum is distnace and the second is angular velcoties
        self.movement_measurments = movement_data
        # wifi data is a NxM matrix where N is the number of time steps and M is the number of unquia WAPs
        # If we do not have a reading from a spefic WAP at a point in time we put NaN in it's cell
        self.wifi_measurments = wifi_data
        
        # get the dead reckong math functions, these allow us to go between XY and D-Phi space
        self.drm = dead_reckoning.deadReckonginMath()
        
        # variances
         # TODO look up the values I experemntly found for these
        self.var_dis = 1
        self.var_gyro = 1
        self.var_wifi = 1
        
        # other variables
        self.tau = 2 # scale parameter repesting the distance between walls
        
        # get the total number of measuremnts of each type we have
        self.num_dist_meas = self.movement_data.size[0]
        self.num_gyro_meas = self.movement_data.size[0]
        self.num_wifi_meas = np.sum(~np.isnan(self.wifi_measurments))
        # get the total number of measurments, this is the size of the state vecotr
        self.total_meas = self.num_dist_meas + self.num_gyro_meas + self.num_wifi_meas
        
    def solve_slam(self):
        
        convered = False
        converg_threshold = 0.1 # I made this up
        trys = 0
        max_trys = 5
        while convered is False and trys < max_trys:
            # get the XY repsentation of the robot pose
            self.positon_xy = self.drm.rd2xy(robot_position)
            
            # calulate predicted WiFi measurments
            h_wifi = self.predict_h_wifi()
            
            # calculate predicted gyro measurments
            h_gyro = self.predict_h_gyro(robot_position[:,1])
            
            # do not need to calculate predicted distance meaurments because we "measure that directly"
            # if the paper cheats we get to cheat too
            
            # calculate the diffrence between predicted and real measurments
            diff_wifi = self.wifi_measurments - h_wifi
            diff_dist = self.movement_measurments[:,0] - robot_position[:,0]
            diff_gyro = self.movement_measurments[:,1] - h_gyro
            
            # multipy in the inverse variance of the sensors into the diffrences
            diff_wifi *= (1/self.var_wifi)
            diff_dist *= (1/self.var_dist)
            diff_gyro *= (1/self.var_gyro)
            
            # calcuate the jacobian
            jac = self.calc_jacobian()
            
            # formulat diffrences into a giant vector
            
            # solve for correction and adjust the state vector
            
            # check if convergence critera is met
            if np.mean(correction) < converg_threshold:
                # if on average nothing is changing by more than 0.1 say we're convered
                # I have no idea if this a good way to check for convergence
                # at the very least we should probably have a diffrent threshold for each meausrment type
                convered = True
                
            # reformulat new state vecotr back into a sane repsentatoin


    # def_h_wifi
    # predicts the vector h of all predicted wifi measurements for a single access point
    # h[i] = a single prediction
    def predict_h_wifi(self):
        assert self.position.shape[0] == self.wifi_data.shape[0]
        
        # we should be returning predictions in an identical form as the input
        pred_wifi_matrix = np.empty(wifi_data.shape)
        # making identical means lots of NaNs
        pred_wifi_matrix[:] = np.NaN
        
        # loop over the diffrent access points
        for wap in range(wifi_data.shape[1])
            
            # get the real wifi measurment and the corosponding movment measurments
            # we can use the real_wif_index to put the NaNs back at the end
            real_wifi_index = ~np.isnan(wifi_data[:,wap])
            real_wifi = wifi_data[real_wifi_index:,wap]
            wifi_move_data = self.move_data_xy[real_wifi_index]
        
            # to get z_wifi
            # 1. take column from wifi_data (for a single AP)
            # 2. remove zeros from the column, and remove the corresponding move_data measurements (recall assertion)
            # 3. loop 
            pred_wifi = np.zeros(real_wifi.shape)
            for i in range(real_wifi.shape[0]):
                beta = self.calc_beta(i, wifi_move_data)
                pred_wifi[i] = np.transpose(beta) * real_wifi
                
            # put the prediced WiFi data back into a vecor with the NaN
            # if preformace becomes an issue we should do this without the temp variabel.
            pred_wifi_full_size = pred_wifi_matrix[:wap]
            pred_wifi_full_size[real_wifi_ind] = pred_wifi
            pred_wifi_matrix[:wap] = pred_wifi_full_size
            
        return pred_wifi_matrix
            
    def calc_beta(self, index, wifi_move_data):
        """Calculate the weighting vecory Beta for the ith wifi measurment
    
        In the paper notation this function caluclates omega and then does the subtraction
    
        Inputs:
            index: integrer for wifi mesurmement this weight vector corrosponds to
            wifi_move_data: Nx2 vecor of XY cords corrosponding to WiFi data
    
        Outputs:
            Collum vecor of weights, the element corrosponding to the input index will be zero"""
        # we need the x and y element of wifi_move_data a lot so just find it once
        xi = wifi_move_data[index,0]
        yi = wifi_move_data[index,1]
        
        # calcuate the scaling factor
        scale = -1/(2*self.tau**2)
        
        beta = np.zeros([wifi_move_data.shape[0]])
        for j in range(wifi_move_data.shape[0]):
            # euqclidan disntace squared
            equ_dist = ((xi -  wifi_move_data[j,0])**2) + ((yi -  wifi_move_data[j,1])**2)
            # get the weight value
            beta[j] = np.exp(scale * equ_dist)
            
        # zero out the term corrosonding to the measurment we're looking for
        beta[index] = 0
        
        return beta
        
    def predict_h_gyro(self, angles):
        """"Use angels to predit gyroscope measurments
        
        Inputs:
            angles: N length vector of headings
        Outputs:
            N-1 length vector of gyroscope readings
        """
        # pre allocate output
        gyro = np.zeros(angels.size -1)
        for i = range(angles.size-1):
            gyro(i) = angles(i+1) - angles(i)
            
        return gyro
        
    def calc_jacobian(self):
        """Find the jacobian of the state space and measurment predictions"""
        jac = np.zeros([total_meas, total_meas])