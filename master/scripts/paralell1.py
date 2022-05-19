#! /usr/bin/python3

import time

time1 = time.time()
counter = 0
while(time.time()-time1 < 5):
 counter +=1
 time.sleep(0.5)
 print(counter)

