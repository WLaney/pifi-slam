#!/usr/bin/env python3
import subprocess
import time
import RPi.GPIO as GPIO



class Collection:

    def __init__(self):

        # collection boolean
        self.collecting = False

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
        # Create a file for each WiFi adapter
        f = open('wifi.txt', 'w' )

        # Create a count variable
        count = 0

        # Wait for user to push 'start' button
        print("Push button 17 to start, 27 to quit.")
        while not self.collecting:
            time.sleep(0.05)
            # Todo: make this a threading.Event()

        # User has pushed 'start'
        # Here, we're sampling at 0.25 Hz
        starttime = time.time()

        while self.collecting:
            t1 = time.time()
            print(t1)
            p = subprocess.check_output(self.cmd3, shell=True)
            # To track how long the command takes:
            #print(time.time() - t1)
            f.write("Iteration " + str(count) + '\n\n')
            f.write(p.decode())
            count += 1
            time.sleep(float('%.6f'%(4 - (time.time() - t1))))

        f.close()
        GPIO.cleanup()
        print("Done")

if __name__ == "__main__":
    a = Collection()
    a.run()
