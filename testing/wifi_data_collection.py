#!/usr/bin/env python3
import subprocess
import time
import RPi.GPIO as GPIO
import json


class Collection:

    def __init__(self):

        # collection boolean
        self.collecting = False

        # dictionary (to be outputted as json file)
        self.wifi_data = {}

        # set up GPIO
        GPIO.setmode(GPIO.BCM)  # Set for broadcom numbering, not board numbers
        GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(17, GPIO.FALLING, callback=self.button_press)
        GPIO.add_event_detect(27, GPIO.FALLING, callback=self.leave)
        
        # WiFi data collection commands
        self.cmd0 = 'sudo iwlist wlan0 scan | egrep "Cell|ESSID|Signal"'
        self.cmd1 = 'sudo iwlist wlan1 scan | egrep "Cell|ESSID|Signal"'
        self.cmd2 = 'sudo iwlist wlan0 scan'
        self.cmd3 = 'sudo iw dev wlan0 scan'

    def button_press(self, num):
        if self.collecting:
            self.collecting = False
        else:
            self.collecting = True

    def leave(self, num):
        GPIO.cleanup()
        quit()

    def run(self):
        count = 0

        # Wait for user to push 'start' button
        print("Push button 17 to start and stop data collection, 27 to quit.")
        while not self.collecting:
            time.sleep(0.1)
            # Todo: make this a threading.Event()

        # User has pushed 'start'

        while self.collecting:
            try:
                p = subprocess.check_output(self.cmd0, shell=True)
                t = time.clock_gettime(time.CLOCK_MONOTONIC_RAW)
                self.wifi_data[t] = p.decode()
            except subprocess.CalledProcessError:
                print("wlan0 was busy")
                time.sleep(0.1)
            finally:
                print("count = " + str(count))
                count += 1
        
        # User has pushed button 17 again (stop).
        
        # Create a file for each WiFi adapter
        print(json.dumps(self.wifi_data))

        #f = open('wifi.txt', 'w' )
        GPIO.cleanup()
        print("Done")

if __name__ == "__main__":
    a = Collection()
    a.run()
