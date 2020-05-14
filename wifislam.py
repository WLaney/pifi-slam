#!/usr/bin/env python3

from analysis import dead_reckoning
import numpy as np

class Slam:

    def __init__(self, movement_data, wifi_data):
        """Set up class for solving the WIFI slam problem. Call solve_slam to get a result
        
        Inputs:
            movement_data: Nx2 matrix corrosponding to distance and gyro measurments.
                            N is the number os time steps we have measurments for. Collum 1 is distance,
                            collum 2 is gyro
            wifi_data: NxM matric corrosponding to WiFi measurments.
                      N is the number os time steps we have measurments for.
                      M is the number of access points we saw in all the data. If we did not get data from
                      a particular access point at a time step that entry should be NaN
        """
        
        # input data
        # movment data is a Nx2 matrix where N is the number of time steps
        # the first collum is distnace and the second is angular velcoties
        self.movement_measurments = movement_data
        # wifi data is a NxM matrix where N is the number of time steps and M is the number of unquia WAPs
        # If we do not have a reading from a spefic WAP at a point in time we put NaN in it's cell
        self.wifi_measurments = wifi_data
        
        # get the dead reckong math functions, these allow us to go between XY and D-Phi space
        self.drm = dead_reckoning.deadReckoningMath()
        
        # variances
         # very rough axpromemations from experements
         # error progegation is hard and confusing
        self.var_dist = 0.003
        self.var_gyro = 0.2
        self.var_wifi = 3
        
        # other variables
        self.tau = 2 # scale parameter repesting the distance between walls
        
        # get the total number of measuremnts of each type we have
        #self.num_dist = self.movement_measurments.shape[0] + 1
        #self.num_angle = self.movement_measurments.shape[0] + 1
        self.num_dist = self.movement_measurments.shape[0]
        self.num_gyro = self.movement_measurments.shape[0]
        # we get one more angle than we do gyro measurment
        self.num_angle = self.movement_measurments.shape[0] + 1
        self.num_wifi_meas = np.sum(~np.isnan(self.wifi_measurments))
        # get the total number of measurments, this is the size of the state vecotr
        self.total_meas = self.num_dist + self.num_gyro + self.num_wifi_meas
        
    def solve_slam(self):
        """Solve the slam problem. This is the  main class fuction, and the only one that needs called
        
        Outputs:
            robot_position: N+1 by 2 matrix corrosponding to distance and angles, N is the number of measurmenst.
            This is the slam solution to where the robot is. We don't return anything to do with WiFi because
            we don't care
        """
        # inizial the state, we should probably not actual use all zeros for this
         # we use the number of angles so we can generate gyro predictions correctly
         # see section VB in the paper
        robot_position = np.zeros([self.num_angle, 2])
        wifi_state = np.zeros(self.num_wifi_meas)
        
        convered = False
        converg_threshold = 0.1 # I made this up
        trys = 0
        max_trys = 5
        while convered is False and trys < max_trys:
            # get the XY repsentation of the robot pose
            self.positon_xy = self.drm.rd2xy(robot_position[:,0], robot_position[:,1])
            
            # calulate predicted WiFi measurments
            h_wifi = self.predict_h_wifi()
            
            # calculate predicted gyro measurments
            h_gyro = self.predict_h_gyro(robot_position[:,1])
            
            # do not need to calculate predicted distance meaurments because we "measure that directly"
            # if the paper cheats we get to cheat too
            
            # calculate the diffrence between predicted and real measurments
            diff_wifi = self.wifi_measurments - h_wifi
            # the state vector distances are longer than the measured distnaces by one
            # this is because of gyro calcuations
            diff_dist = self.movement_measurments[:,0] - robot_position[:-1,0]
            diff_gyro = self.movement_measurments[:,1] - h_gyro
            
            # multipy in the inverse variance of the sensors into the diffrences
            diff_wifi *= (1/self.var_wifi)
            diff_dist *= (1/self.var_dist)
            diff_gyro *= (1/self.var_gyro)
            
            # calcuate the jacobian
            jac = self.calc_jacobian(h_wifi, robot_position)
            
            # formulat diffrences into a giant vector
            # first we need to put the wifi data into a vector, we flattern and then get rid of the naps
            diff_wifi_vector = diff_wifi.flatten('F')
            diff_wifi_vector = diff_wifi_vector[~np.isnan(diff_wifi_vector)]
            # concatinate everything
            diff_vector = np.concatenate((diff_dist, diff_gyro, diff_wifi_vector))
            # so the same for the state
            state_vecotr = np.concatenate((robot_position.flatten('F'), wifi_state))
            # solve for correction and adjust the state vector
            correction = np.linalg.lstsq(jac, diff_vector, rcond=None)
            # check if convergence critera is met
            if np.mean(correction[0]) < converg_threshold:
                # if on average nothing is changing by more than 0.1 say we're convered
                # I have no idea if this a good way to check for convergence
                # at the very least we should probably have a diffrent threshold for each meausrment type
                convered = True
                
            # reformulat new state vecotr back into a sane repsentatoin
            state_vecotr = state_vecotr + correction[0]
            robot_position[:,0] = state_vecotr[0:self.num_angle]
            robot_position[:,1] = state_vecotr[self.num_angle:self.num_angle + self.num_angle]
            wifi_state = state_vecotr[-self.num_wifi_meas:]
            # incremtn try counter
            trys += 1
         
        if convered is False:
            print("Slam Failed to converge. Returning latest position state vector")
        else:
            print("Slam convereged succesfuly!")
            
        return robot_position


    # def_h_wifi
    def predict_h_wifi(self):
        """Predicts the vector h of all predicted wifi measurements for all access points
        
        h[i] = a single prediction
        
        Outputs:
            pred_wifi_matrix: a TxW matrix where T is the numbor of measuremnt time stampts and W is the
            number of WIFI measurments. Any measurment that is NaN in the WiFi measurments will be NaN in
            the predicted measurments.
        """
        assert self.positon_xy.shape[0] == self.wifi_measurments.shape[0]
        
        # we should be returning predictions in an identical form as the input
        pred_wifi_matrix = np.empty(self.wifi_measurments.shape)
        # making identical means lots of NaNs
        pred_wifi_matrix[:] = np.NaN
        
        # loop over the diffrent access points
        for wap in range(self.wifi_measurments.shape[1]):
            
            # get the real wifi measurment and the corosponding movment measurments
            # we can use the real_wif_index to put the NaNs back at the end
            real_wifi_index = ~np.isnan(self.wifi_measurments[:,wap])
            real_wifi = self.wifi_measurments[real_wifi_index,wap]
            wifi_move_data = self.positon_xy[real_wifi_index,:]
        
            # to get z_wifi
            # 1. take column from wifi_data (for a single AP)
            # 2. remove zeros from the column, and remove the corresponding move_data measurements (recall assertion)
            # 3. loop 
            pred_wifi = np.zeros(real_wifi.shape)
            for i in range(real_wifi.shape[0]):
                beta = self.calc_beta(i, wifi_move_data)
                pred_wifi[i] = np.matmul(np.transpose(beta),real_wifi)
                
            # put the prediced WiFi data back into a vecor with the NaN
            # if preformace becomes an issue we should do this without the temp variabel.
            pred_wifi_full_size = pred_wifi_matrix[:,wap]
            pred_wifi_full_size[real_wifi_index] = pred_wifi
            pred_wifi_matrix[:,wap] = pred_wifi_full_size
            
        return pred_wifi_matrix
            
    def calc_beta(self, index, wifi_move_data):
        """Calculate the weighting vecory Beta for the ith wifi measurment
    
        In the paper notation this function caluclates omega and then does the subtraction
    
        Inputs:
            index: integrer for wifi mesurmement this weight vector corrosponds to
            wifi_move_data: Nx2 vecor of XY cords corrosponding to WiFi data
    
        Outputs:
            beta: Collum vecor of weights, the element corrosponding to the input index will be zero"""
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
        gyro = np.zeros(angles.size -1)
        for i in range(angles.size-1):
            gyro[i] = angles[i+1] - angles[i]
            
        return gyro
        
    def calc_jacobian(self, h_wifi, robot_position):
        """Find the jacobian of the state space and measurment predictions
        
        Inputs:
            h_wifi: matrix of predicted WiFi values. Geneared from predict_h_wifi
            robot_position: Mx2 matrix of the current belived state of robot positions. M is the number of
            distance/angles
        
        Outputs:
            jac: Jacobian matrix for the state. NxM matirx where N is the number of measurments, and M is
                the size of the state space. M=N+2
        """
        
        # the jacbobian should be a square matrix and repesent all the state vairables
        # the order of the variables is all the distance measurments, then all the gyro measurments
        # then all the wifi measurments in order of access point as defined in the measurment matrix
        # J = [[hd_1/dd1,...,hd_1/ddtheta1,..., hd_1/ddwifi]
        #      [.                .                       .]
        #      [.                .                       .]
        #      [.                .                       .]
        #      [hgyro_1/dd1,...,hgyro_1/ddtheta1,...,hgyro_1/ddwifi]
        #      [.                .                       .]
        #      [.                .                       .]
        #      [.                .                       .]
        #      [hwifi_1/dd1,...,hwifi_1/ddtheta1,...hwifi_1/ddwifi]
        
        jac = np.zeros([self.total_meas, self.total_meas+2])
        
        # sice the distance measurmetnts predictions are based only on the distance the derivates are 1
        # for the measurment in questoin and zero everywhere else
        # ie hd_1/dd1 = 1 and hd_1/dd2 = 0 hd_1/ddthetaN = 0, hd_1/ddwifiN = 0
        # this means the secion of the jacobian concerned with the derivates of h_distance is a idently matrix
        jac[:self.num_dist,:self.num_dist] = np.eye(self.num_dist)
        
        # the gyro measurments are based on angle theta_i and theta_i+1
        # this means for each gyro measurment prediction there are non zero derivates for theta_i and theta_i+1
        # these derivates will be -1 and 1 repspectfuly
        for i in range(self.num_dist+1, self.num_dist+1+self.num_angle-1):
            jac[i-1,i] = -1
            jac[i-1, i+1] = 1
            
        # now we need to fill in the h_wifi derivates, this is more complicated so the actual math is done
        # in helper functions
        
        # all of the h_wifi derivates need the jacobain of the x state space with respect to the state
        # variables, that's a constan so we will find it here, out of the loops
        jac_xy_dp = self.jacobian_xy_dp(robot_position)
        
        # set up counter so we know what row of the jacobian we should write the wifi derivates to
        jac_wifi_row = self.num_dist+self.num_gyro
        
        # Now we loop through all the wifi real (real meaning not NaN) measurments
        for wap in range(self.wifi_measurments.shape[1]):
            
            # get the real wifi measurment and the corosponding movment measurments
            real_wifi_indecies = ~np.isnan(self.wifi_measurments[:,wap])
            real_wifi = self.wifi_measurments[real_wifi_indecies,wap]
            real_predict_wifi = h_wifi[real_wifi_indecies,wap]
        
            for i in range(real_wifi.shape[0]):
                # get the predicted value for this wifi measurment
                h_wifi_i = real_predict_wifi[i]
                wifi_dervivative = self.calc_wifi_derivative(i, real_wifi, h_wifi_i, real_wifi_indecies)
                wifi_derivate_dphi_state = np.matmul(wifi_dervivative, jac_xy_dp)
                jac[jac_wifi_row,:] = wifi_derivate_dphi_state
                jac_wifi_row += 1
                
        return jac
        
    def calc_wifi_derivative(self, index, real_wifi, h_wifi_i, real_wifi_indecies):
        """Calculate the derivates of the WiFi function for a spefic predition.
        
        These derivates are in the X-Y state space. The results of this function must by multipled by the
        results of jacobian_xy_dp to get values sutable for the full state space jacobian
        
        Inputs:
            index: Index corrosponding to a wifi meausrment for the access point
                   This is where the derivate is
            real_wifi: list of all the real wifi measurments at the access point
            h_wifi_i: predicted wifi measurment at index
            real_wifi_indecies: boolean list of where the real wifi measurments are at this access points
                                We need this to get the robots XY postions at the measurments
        Returns:
            Vector of the wifi derivate. This vector is 1xM where M is the number of X and Y postions
        """
        
        # pre allocate the derivate vecotr
        wifi_dervivative = np.zeros(2*self.positon_xy.shape[0])
        
        # get the xy postion corrosponding the value we're taking the derivate for
        position_i = self.positon_xy[index,:]
        
        # get the xy postions of the real wifi measurments
        wifi_move_data = self.positon_xy[real_wifi_indecies,:]
        
        # get where the derivates values should go in the derivate vector
        # the vecotr goes [x1, y1, x2, y2,...]
        # this is the x postion, we multiply by two to skip the ys
        x_incds = 2 * np.nonzero(real_wifi_indecies)[0]  # I hate numpy somtimes
        
        # get index for the i derivative
        x_i_inc = x_incds[index]
        
        # get beta outside the loop
        beta = self.calc_beta(index, self.positon_xy[real_wifi_indecies,:])
        
        # loop through all the wifi measumrents values from the access point h_wifi_i is from
        # and get the derivate infromation for that point. Each measurment gives four derivates values to 
        # put in the derivate vecotr: d/dxi, d/dxj, d/dyi, and d/dyj
        # at each iteration of the loop these values add to the final derivate vector
        for j in range(real_wifi.size):
            # calculate the sclar portion
            scaler = beta[j] * (real_wifi[j] - h_wifi_i) * (-1/(2*self.tau**2))
            # get the diffrences between the x and y values
            x_diff = wifi_move_data[j,0] - position_i[0]
            y_diff = wifi_move_data[j,1] - position_i[1]
            # get the actual derivate values
            # the two values come from the expoentnts in the equlidean distance squard formula
            # the sign is what is being subtracted from in the diffrence
            d_dxi = scaler * -2 * x_diff
            d_dxj = scaler * 2 * x_diff
            d_dyi = scaler * -2 * y_diff
            d_dyj = scaler * 2 * y_diff
            
            # now we have to add these derivate values to the proper places in derivate matrix
            x_j_inc = x_incds[j]
            wifi_dervivative[x_i_inc] += d_dxi
            wifi_dervivative[x_i_inc + 1] += d_dyi
            wifi_dervivative[x_j_inc] += d_dxj
            wifi_dervivative[x_j_inc +1] += d_dyj
            
        return wifi_dervivative
        
            
    def jacobian_xy_dp(self, robot_position):
        """Get jacobian for going between the XY state space and distance/angle state space
        
        This combined with the results of calc_wifi_derivative gives results for the full jacobian
        
        Inputs:
            robot_positoin: current belived robot state in distnace and angles
        
        Retruns:
            jac_xy_dp: jacobian of functions used to go from distance/angle to XY state space
            The resulting jaconian is MxN where M is the number of X cords the number of Y cords,
            N is the length of the state space vecotr
        """
        
        # the resulting jaconian needs to be MxN where M is the number of X cords the number of Y cords
        # N is the length of the state space vecotr
        #
        # to create this matrix we are going to create three smaller matricies and concatnate them into the
        # full matrix.
        # The first matrix will be the distance derivates for each cordinate
        # the second matrix will be the angle derivates for each cordinate
        # the third matrix will be the wifi derivates for each cordinate, this matrix will be all zeros
        
        # we multisply the number of distnace by 2 for the rows because each distance is giving us an x and a
        # y corridinate
        distance_derv = np.zeros([(self.num_dist * 2), self.num_dist + 1])
        
        # i represent the X-Y pair we're on
        # meas_num is a second counter to keep track of what distnace we're on, we could derive this
        # from the loop counter but I think a second counter cuts down on confustin
        # every distance leads to an 1 X positoin and 1 Y position
        meas_num = 0
        for i in range(0,self.num_dist*2, 2):
            distance_derv[i::2, meas_num] = np.cos(robot_position[meas_num,1])
            distance_derv[i+1::2, meas_num] = np.sin(robot_position[meas_num,1])
            meas_num += 1
        #print(distance_derv)
        # we're now going to creata the angle derivatives with the same logic
        angle_derv = np.zeros([self.num_gyro * 2, self.num_gyro+1])
        meas_num = 0
        for i in range(0,self.num_gyro*2, 2):
            angle_derv[i::2, meas_num] = -1 * robot_position[meas_num,0] * np.sin(robot_position[meas_num,1])
            angle_derv[i+1::2, meas_num] = robot_position[meas_num,0] * np.cos(robot_position[meas_num,1])
            meas_num += 1
        #print(angle_derv)
            
        # finaly create a zero matrix representing the derivates with respect to the wifi stengths
        # add one to the number of wifi measurments for the last angle dosent get used
        wifi_derv = np.zeros([self.num_dist * 2, self.num_wifi_meas])
        
        # combine everything together into the final jacobian matrix
        jac_xy_dp = np.concatenate((distance_derv, angle_derv, wifi_derv), axis=1)
        
        return jac_xy_dp
            
        