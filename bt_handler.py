import time
import serial
import json

def send_value(command):
    try:
        j = str(command)
        b = j.encode()
        bluetoothSerial.write(b)
        time.sleep(0.1)
    except:
        print("couldn't send value to bluetooth device")

try:
    bluetoothSerial = serial.Serial( "/dev/rfcomm{bt}".format(bt=0), baudrate=115200 )
    print('sucess')
except NameError:
    print("Please connect to the Bluetooth")
    
    
while(True):
    try:
        
        with open('commands.txt','r') as json_file:
            data = json.load(json_file)
            print(data)
            
            time.sleep(0.4)
            
        if data["RIGHT"] == 1:
            send_value('R')

        if data["LEFT"] == 1:
            send_value('L')

        if data["BACKWARD"] == 1:
            send_value('B')
            
        if data["FORWARD"] == 1:
            send_value('F')
            
        if data["NOTHING"] == 1:
            send_value('N')
                        
    except:
        print("file in use")
    