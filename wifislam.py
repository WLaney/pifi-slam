#!/usr/bin/env python3

import analysis/dead_reckoning.py
import numpy

class Slam:

    def __init__(self, movement_data, wifi_data):
        
        # input data
        self.movement_data = movement_data
        self.wifi_data = wifi_data
        
        # variances
        #self.var_imu = 
        #self.var_wifi = 
        
        # other variables
        self.tau = 2


    # def_h_wifi
    # predicts the vector h of all predicted wifi measurements for a single access point
    # h[i] = a single prediction
    def predict_h_wifi(self, move_data, wifi_data):
        assert move_data.shape[0] == wifi_data.shape[0]
        
        # convert move_data to x/y 
        move_data_xy = deadReckoning.rd2xy(move_data)
        
        # to get z_wifi
        # 1. take column from wifi_data (for a single AP)
        # 2. remove zeros from the column, and remove the corresponding move_data measurements (recall assertion)
        # 3. loop 