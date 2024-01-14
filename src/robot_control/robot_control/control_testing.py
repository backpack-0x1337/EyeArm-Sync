#!/usr/bin/python3 
import serial
import json

ser = serial.Serial('/dev/ttyUSB0', 115200)

ALL_TORQUE_OFF = {"T":8,"P1":0}
ALL_TORQUE_ON = {"T":8,"P1":1}
GET_ANGTOR_INFO = {"T":5}
GET_DEV_INFO = {"T":4}
HELP = {'T': 10}
EMERGENCY_STOP = {'T': 0}
COORD_CTRL = {"T":2,
              "P1":300.51,"P2":-13.75,"P3":276.5822754,"P4":90,"P5":280,
              "S1":10,"S5":200}
ST_POS_CTRL_INIT = {'T': 13}
CORD_CTRL_INIT = {'T': 12}
POS_CTRL = {"T":3,"P1":2047,"P2":0,"P3":3500,"P4":1500,"P5":2047,
              "S1":200,"S2":200,"S3":200,"S4":200,"S5":200,
              "A1":60,"A2":60,"A3":60,"A4":60,"A5":60}

json_data = json.dumps(POS_CTRL)
ser.write(json_data.encode())

json_data = json.dumps(EMERGENCY_STOP)
ser.write(json_data.encode())

response = ser.readline().decode()
print(f'response: {response}')

response = ser.readline().decode()
print(f'response: {response}')


ser.close()


