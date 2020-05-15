#!/usr/bin/env python3
from analysis import wifislam
import numpy as np
import unittest

class TestWIFIslamZeros(unittest.TestCase):
    # simple test that all functions work
    def setUp(self):
        # the robot sits in place for two time stamps and get's zero readings from one access point
        self.movement_data = np.zeros([2,2])
        self.wifi_data = np.zeros([2,1])
        self.slam = wifislam.Slam(self.movement_data, self.wifi_data)
        
        # set up some class vairables that are needed. These are normally created and maintained in
        # solve slam
        self.robot_position = np.zeros([self.slam.num_angle, 2])
        self.slam.positon_xy = self.slam.drm.rd2xy(self.robot_position[:,0], self.robot_position[:,1])
        
    def test_beta(self):
        beta = self.slam.calc_beta(0, self.movement_data)
        # im not really sure you're suppose to start usin np testing in unittest like this, but
        # it basicaly works so I'm not going to think to hard about this
        np.testing.assert_array_equal(beta, np.array([0,1]))
        
    def test_predict_h_wifi(self):
        h_wifi = self.slam.predict_h_wifi()
        np.testing.assert_array_equal(self.wifi_data, h_wifi)
        
    def test_precit_h_gyro(self):
        h_gyro = self.slam.predict_h_gyro(self.robot_position[:,0])
        np.testing.assert_array_equal(self.movement_data[:,1], h_gyro)
        
    def test_jacobian_xy_dp(self):
        jac_xy = np.zeros([4,8])
        jac_xy[0,0] = 1
        jac_xy[2,0] = 1
        jac_xy[2,1] = 1
        calc_jac_xy = self.slam.jacobian_xy_dp(self.robot_position)
        np.testing.assert_array_equal(calc_jac_xy, jac_xy)
        
    def test_calc_wifi_derivative(self):
        real_wifi_indecies = np.array([True, True])
        derivative = self.slam.calc_wifi_derivative(0, self.wifi_data, 0, real_wifi_indecies)
        np.testing.assert_array_equal(derivative, np.zeros(4))
        
    def test_calc_jacobian(self):
        # calculated by hand
        jac = np.array([[1, 0, 0, 0, 0, 0, 0, 0],
                        [0, 1, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, -1, 1, 0, 0, 0],
                        [0, 0, 0, 0, -1, 1, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0]])
        calc_jac = self.slam.calc_jacobian(self.wifi_data, self.robot_position)
        np.testing.assert_array_equal(jac, calc_jac)
        
    def test_solve_slam(self):
        slam_positions = self.slam.solve_slam()
        np.testing.assert_array_equal(slam_positions, self.robot_position)
        
        
class TestWIFIslamNaNs(unittest.TestCase):
    # test functions that deal with WiFi data handle NaNs correctly
    def setUp(self):
        # the robot sits in place for two time stamps and get's zero readings from one access point
        self.movement_data = np.zeros([2,2])
        self.wifi_data = np.array([[0,0],[0, np.nan]])
        self.slam = wifislam.Slam(self.movement_data, self.wifi_data)
        
        # set up some class vairables that are needed. These are normally created and maintained in
        # solve slam
        self.robot_position = np.zeros([self.slam.num_angle, 2])
        self.slam.positon_xy = self.slam.drm.rd2xy(self.robot_position[:,0], self.robot_position[:,1])

    def test_predict_h_wifi(self):
        h_wifi = self.slam.predict_h_wifi()
        np.testing.assert_array_equal(self.wifi_data, h_wifi)
        
    def test_jacobian_xy_dp(self):
        jac_xy = np.zeros([4,9])
        jac_xy[0,0] = 1
        jac_xy[2,0] = 1
        jac_xy[2,1] = 1
        calc_jac_xy = self.slam.jacobian_xy_dp(self.robot_position)
        np.testing.assert_array_equal(calc_jac_xy, jac_xy)
        
    def test_calc_wifi_derivative(self):
        real_wifi_indecies = np.array([True, False])
        real_wifi = self.wifi_data[real_wifi_indecies, 1]
        derivative = self.slam.calc_wifi_derivative(0, real_wifi, 0, real_wifi_indecies)
        np.testing.assert_array_equal(derivative, np.zeros(4))
    
    def test_calc_jacobian(self):
        # calculated by hand
        jac = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 1, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, -1, 1, 0, 0, 0, 0],
                        [0, 0, 0, 0, -1, 1, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0]])
        calc_jac = self.slam.calc_jacobian(self.wifi_data, self.robot_position)
        np.testing.assert_array_equal(jac, calc_jac)

    def test_solve_slam(self):
        slam_positions = self.slam.solve_slam()
        np.testing.assert_array_equal(slam_positions, self.robot_position)
        
if __name__ == '__main__':
    unittest.main()