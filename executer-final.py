
from time import time
import serial
import os

sample_counter_x = 0
sample_counter_y = 0
sample_counter_z = 0


ser = serial.Serial(port='/dev/tty.usbmodem1403', baudrate=115200)

print("Ligado a: " + ser.portstr)



while 1:

        ser.flushInput()

        print("À espera dos valores")
        
        serialString = ser.readline()
        serialString = serialString.split(b' ')
        
        
        #   Conversões para float
        float_x = str( float(serialString[0].decode('ASCII')) / 100 )
        float_y = str( float(serialString[1].decode('ASCII')) / 100 )
        float_z = str( float(serialString[2].decode('ASCII')) / 100 )

        #   Impressão dos valores do acel.
        print('X = ' + float_x)
        print('Y = ' + float_y)
        print('Z = ' + float_z)


        #   EXECUTER
        if (float(serialString[2].decode('ASCII')) < 1  and  float(serialString[1].decode('ASCII')) < (-9) ):
                sample_counter_x = sample_counter_x + 1

                if(sample_counter_x == 50):
                    os.system("open /Applications/zoom.us.app")
                    sample_counter_x = 0
                    ser.write(b'0')

        #   Inclinar para trás (Eixos X e Z)
        if (float(serialString[2].decode('ASCII')) < 5 and float(serialString[0].decode('ASCII')) > 9 ):
            sample_counter_y = sample_counter_y + 1

            if(sample_counter_y == 50):
                os.system("open /Applications/SofaScore.app")
                sample_counter_y = 0
                ser.write(b'0')


        #   Inclinar para a direita (Eixos Y e Z)
        if (float(serialString[2].decode('ASCII')) < 3  and  float(serialString[1].decode('ASCII')) > 9 ):
            sample_counter_z = sample_counter_z + 1

            if(sample_counter_z == 50):
                os.system("open /Applications/Google\ Chrome.app")
                sample_counter_z = 0
                ser.write(b'0')

