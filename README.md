# Pi-Fi SLAM
WiFi SLAM on a Raspberry Pi. This is William and Luke's final project for ECE 5725. Our full project report can be found [here](https://courses.ece.cornell.edu/ece5990/ECE5725_Spring2020_Projects/May_19_Demo/PiFi%20Slam/ece5725-website/index.html).

The work here was based heavly on the paper [Efficient, generalized indoor WiFi GraphSLAM](https://ieeexplore.ieee.org/document/5979643)

## Requirments
```
pip3 install adafruit-blinka
pip3 install adafruit-circuitpython-lsm9ds1
pip3 install matplotlib
```

## Testing

To run the unit tests run the following command from the main repository directory.

```
python3 -m unittest testing.test_wifislam
```

## Orginization

* **analysis**
  * *data_association.py*: Take in raw measurments and prepare them for the WiFi SLAM code
  * *dead_reckoning.py*: Preform the dead reckoning functions nessary for WiFi SLAM along with full dead reckoning
  * *tft_plots.py*: Display the plots that result from WiFi SLAM and Dead Reckoning on the Pi TFT
  * *wifislam.py*: Does what it says on the tin
* **data_collection**
  * *lsm9ds1_data_collection.py*: Collect IMU data and write it to a CSV file
  * *wifi_data_collection.py*: Collect WiFi data and write it to a CSV file
* **images**
  * Directory of images that are helpful to the project
* **testing**
  * *imu_stats.py*: Calculate IMU sample statstics
  * *lsm9ds1_test.py*: Test that the IMU is connected and working
  * *lsm9ds_time_test.py*: Script used to charterize max IMU sampling rate
  * *sample_imu_stats.txt*: IMU stats from our system
  * *sample_wifi_stats.txt*: WiFi stats from our system
  * *test_wifislam.py*: WiFi SLAM unit tests
* *control.py*: Main control function. Run this to collect data, solve SLAM, and display the plots on the piTFT.

