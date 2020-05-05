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
        self.var_dis = 1
        self.var_gyro = 1
        #self.var_wifi = 
        
        # other variables
        self.tau = 2


    # def_h_wifi
    # predicts the vector h of all predicted wifi measurements for a single access point
    # h[i] = a single prediction
    def predict_h_wifi(self, wifi_data_ap):
        assert self.move_data.shape[0] == wifi_data_ap.shape[0]
        
        # convert move_data to x/y 
        move_data_xy = self.drm.rd2xy(good_move_data)
        
        # get the real wifi measurment and the corosponding movment measurments
        # we can use the real_wif_index to put the NaNs back at the end
        real_wifi_index = ~np.isnan(wifi_data_ap)
        real_wifi = wifi_data_ap[real_wifi_index]
        wifi_move_data = self.move_data_xy[real_wifi_index]
        
        # to get z_wifi
        # 1. take column from wifi_data (for a single AP)
        # 2. remove zeros from the column, and remove the corresponding move_data measurements (recall assertion)
        # 3. loop 
        pred_wifi = np.zeros(real_wifi.shape)
        for i in range(real_wifi.shape[0]):
            beta = self.calc_beta(i, wifi_move_data)
            pred_wifi[i] = np.transpose(beta) * real_wifi
            
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
        