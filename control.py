#!/usr/bin/env python3

import os
from multiprocessing import Pool

scripts = ('wifi_data_collection.py', 'lsm9ds1_data_collection.py')

def run_script(script):
    os.system('python3 {}'.format(script))

pool = Pool(2)
pool.map(run_script, scripts)

print("FINISHED BOTH")
