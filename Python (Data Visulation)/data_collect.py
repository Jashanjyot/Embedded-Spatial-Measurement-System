# Student name: Jashan
# Student number: 400337963
# Pythin version: 3.8.6

import serial
import math

ser = serial.Serial('COM4',115200)
ser.open
print("Opening: " + ser.name)

file = open("2dx3data.xyz", "a") 
step = 16
x = 0 
counter = 0;
incr = 200 
reading = False

while (1):  
    s = ser.readline()
    a = s.decode("utf-8")  
    a = a[0:-2] 
    if (a.isdigit() == True):
        file = open("2dx3data.xyz", "a")
        reading = True
        angle = (step/512)*2*math.pi 
        d = int(a)
        y = d*math.sin(angle)
        z = d*math.cos(angle) 
        file.write('{} {} {}\n'.format(x,z,y)) 
        step += 16
        counter += 1
    if (a.isdigit() == False and reading == True):
        file.close()
        file = open("2dx3data.xyz", "a")
    if counter == 32:
        step = 16
        x = x + incr
        counter = 0
    print(a)
