#!/usr/bin/env python3
import subprocess
import time
import RPi.GPIO as GPIO
import re
import csv
import sys

class Collection:

    def __init__(self):

        # collection boolean
        self.collecting = False

        # list of timestamps
        self.timestamps = []

        # list of wifi data
        self.wifi_raw = []

        # extracted wifi data: list of [ [list of addresses], [list of signals] ]
        self.wifi_data = []

        # total number of access points detected throughout data collection
        # this will be known as "M"
        self.mac_list_len = 0

        # list to look up recognized MAC addresses
        self.mac_list = []

        # set up GPIO
        GPIO.setmode(GPIO.BCM)  # Set for broadcom numbering, not board numbers
        GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(17, GPIO.FALLING, callback=self.button_press, bouncetime=200)
        GPIO.add_event_detect(27, GPIO.FALLING, callback=self.leave)
        
        # WiFi data collection commands
        self.cmd0 = 'sudo iwlist wlan0 scan | egrep "Cell|ESSID|Signal"'
        self.cmd1 = 'sudo iwlist wlan1 scan | egrep "Cell|ESSID|Signal"'
        self.cmd2 = 'sudo iwlist wlan0 scan | egrep "Cell|Signal"'
        self.cmd3 = 'sudo iw dev wlan0 scan'

    def button_press(self, num):
        if self.collecting:
            self.collecting = False
        else:
            self.collecting = True

    def leave(self, num):
        GPIO.cleanup()
        quit()

    def collect(self):
        count = 0

        # Wait for user to push 'start' button
        print("\tWiFi ready to collect.")
        while not self.collecting:
            time.sleep(0.2)

        # User has pushed 'start'
        print('WiFi started.')

        while self.collecting:
            try:
                p = subprocess.check_output(self.cmd2, shell=True)
                t = time.clock_gettime(time.CLOCK_MONOTONIC_RAW)
                self.timestamps.append(t)
                self.wifi_raw.append(p.decode())
                #print(p.decode())
            except subprocess.CalledProcessError:
                print("\twlan0 was busy")
                time.sleep(0.1)
            finally:
                print("\tcount = " + str(count))
                count += 1
        
        # User has pushed button 17 again (stop).

        assert len(self.timestamps) == len(self.wifi_raw)

        GPIO.cleanup()
        print("WiFi data collection complete.")
    
    def extract_data(self):
        for item in self.wifi_raw:
            # Build list of MAC addresses sensed in this sample (item).
            # Note: re.findall() returns list of found re's in the order of occurrence in the source string.
            adr = re.compile(r'\w\w:\w\w:\w\w:\w\w:\w\w:\w\w')
            addresses = adr.findall(item)

            # Build list of signal strengths.
            sig = re.compile(r'level=.* dBm')
            sigs = sig.findall(item)

            # Convert sigs from list of strings to list of ints
            for i in range(len(sigs)):
                sa      = sigs[i]
                sb      = sa.split()
                sc      = sb[0][6:]
                sigs[i] = int(sc)

            # Make an entry to self.wifi_data that includes addresses and sigs
            entry = [addresses, sigs]

            # Add entry to self.wifi_data
            self.wifi_data.append(entry)

            # Add unknown MAC addresses to self.mac_list
            for a in addresses:
                # If the MAC address is not already in self.mac_list, add it
                if a not in self.mac_list:
                    self.mac_list.append(a)
                    self.mac_list_len += 1

        assert self.mac_list_len == len(self.mac_list)

    def make_csv(self, file_path):
        #TODO make the csv filename like the imu csv one (with the date and all)
        line_len = self.mac_list_len + 1
        file_name = file_path + 'wifi_data.csv'
        with open(file_name, 'w') as f:
            writer = csv.writer(f)
            for current_idx, entry in enumerate(self.wifi_data):
                # Create an empty list of length "len(self.mac_list) + 1".
                # + 1 because column 0 is the timestamp of the measurement.
                # The rest of the columns represent signal strengths from a given access point.
                line = []
                for i in range(line_len):
                    # numpy reads this back in from the CSV file through magic
                    line.append(float("NaN"))

                # Add timestamp to the line
                line[0] = self.timestamps[current_idx]

                addresses   = entry[0]
                sigs        = entry[1]

                # Now, loop back through the self.mac_list.
                # MUST use an index i in order to append to correct column in line.
                for i in range(self.mac_list_len):
                    # If the address in self.mac_list is also in the list of addresses recorded in this entry, write it's signal strength in the corresponding column in line
                    mac = self.mac_list[i]
                    if mac in addresses:
                        # Get the index of the address in "addresses"
                        idx = addresses.index(mac)
                        # Add signal value into correct column
                        line[1 + i] = sigs[idx]

                # line is written. Now, just add line to the csv file
                writer.writerow(line)


if __name__ == "__main__":
    file_path = str(sys.argv[1])
    a = Collection()
    a.collect()
    a.extract_data()
    a.make_csv(file_path)
    GPIO.cleanup()
