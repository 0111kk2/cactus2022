import numpy as np
import os
import threading
import time
import serial
import pyproj
import micropyGPS
import csv
import math
import pigpio
from smbus import SMBus

pi=pigpio.pi()
#lat2,lon2=35.7525063,139.7380564 #oujieki , kagurazakakaramite makita
#lat2,lon2=51.2840000,-0.0000564 #gurinijjitenmondai tesutoyou
lat2,lon2=51.2840000,0.0000564 #gurinijjitenmondai tesutoyou 
gps = micropyGPS.MicropyGPS()
ACCL_ADDR = 0x19
ACCL_R_ADDR = 0x02
GYRO_ADDR = 0x69
GYRO_R_ADDR = 0x02
MAG_ADDR = 0x13
MAG_R_ADDR = 0x42
i2c = SMBus(1)
SERVO_R=16
SERVO_L=20
# 出力のフォーマットは度数とする
gps.coord_format = 'dd'

def SETUP():
    bmx_setup()
    
def bmx_setup():
   # acc_data_setup : 加速度の値をセットアップ
   i2c.write_byte_data(ACCL_ADDR, 0x0F, 0x03)
   i2c.write_byte_data(ACCL_ADDR, 0x10, 0x08)
   i2c.write_byte_data(ACCL_ADDR, 0x11, 0x00)
   time.sleep(0.5)
   # gyr_data_setup : ジャイロ値をセットアップ
   i2c.write_byte_data(GYRO_ADDR, 0x0F, 0x04)
   i2c.write_byte_data(GYRO_ADDR, 0x10, 0x07)
   i2c.write_byte_data(GYRO_ADDR, 0x11, 0x00)
   time.sleep(0.5)
   # mag_data_setup : 地磁気値をセットアップ
   data = i2c.read_byte_data(MAG_ADDR, 0x4B)
   if(data == 0):
       i2c.write_byte_data(MAG_ADDR, 0x4B, 0x83)
       time.sleep(0.5)
   i2c.write_byte_data(MAG_ADDR, 0x4B, 0x01)
   i2c.write_byte_data(MAG_ADDR, 0x4C, 0x00)
   i2c.write_byte_data(MAG_ADDR, 0x4E, 0x84)
   i2c.write_byte_data(MAG_ADDR, 0x51, 0x04)
   i2c.write_byte_data(MAG_ADDR, 0x52, 0x16)
   time.sleep(0.5)

def SERVO(a,b):#サーボモーターの回転の1500-2300を0-800に変換，左右を無視して速度だけで書けるようにした
    pi.set_servo_pulsewidth(SERVO_L,1485+a)
    pi.set_servo_pulsewidth(SERVO_R,1480-b)
    
def PID():
    theta1=theta()+96
    if theta1<360:
        theta1=theta1
    elif theta1>360:
        theta1=theta1-360
    print("theta1->",end='')
    print(theta1)
    theta2=getGPS()[2]
    if theta2>0:
        theta2=theta2
    elif theta2<0:
        theta2=theta2+360
    print("theta2->",end='')
    print(theta2)
    thetadash=theta2-theta1
    print("thetadash->",end='')
    print(thetadash)
    
    if 360>thetadash>180:
        thetaLast=thetadash
        if 360>thetaLast>270:
            thetaLast=360-thetaLast
            print("mortar->",end='')
            print(thetaLast)
            M=1*thetaLast
            SERVO(300,300+M)
            print("Turn_Left week")
        elif 270>thetaLast>180:
            thetaLast=360-thetaLast
            print("mortar->",end='')
            print(thetaLast)
            M=1*thetaLast
            SERVO(300,300+M)
            print("Turn_Left strong")
    elif 0<thetadash<180:
        thetaLast=thetadash
        if 0<thetaLast<90:
            print("mortar->",end='')
            print(thetaLast)
            M=1*thetaLast
            SERVO(300+M,300)
            print("Turn_Right week")
        elif 180>thetaLast>90:
            print("mortar->",end='')
            print(thetaLast)
            M=1*thetaLast
            SERVO(300+M,300)
            print("Turn_Right strong")
    elif -180<thetadash<0:
        thetaLast=thetadash+360
        if thetaLast>270:
            thetaLast=360-thetaLast
            print("mortar->",end='')
            print(thetaLast)
            M=1*thetaLast
            SERVO(300,300+M)
            print("Turn_Left week")
        elif thetaLast<270:
            thetaLast=360-thetaLast
            print("mortar->",end='')
            print(thetaLast)
            M=1*thetaLast
            SERVO(300,300+M)
            print("Turn_Left strong")
    elif -360<thetadash<-180:
        thetaLast=thetadash+360
        if thetaLast<90:
            print("mortar->",end='')
            print(thetaLast)
            M=1*thetaLast
            SERVO(300+M,300)
            print("Turn_Right week")
        elif thetaLast>90:
            print("mortar->",end='')
            print(thetaLast)
            M=1*thetaLast
            SERVO(300+M,300)
            print("Turn_Right strong")
    else:
        thetadash=thetadash
        print("E")

def run_gps(): 
    """
    GPSモジュールを読み、GPSオブジェクトを更新する
    :return: None
    """
    s = serial.Serial('/dev/serial0', 9600, timeout=10)

    # 最初の1行は中途半端なデーターが読めることがあるので、捨てる
    s.readline()
       #  GPSデーターを読み、文字列に変換する
    sentence = s.readline().decode('sjis')
    #print(sentence)
        
        # 先頭が'$'でなければ捨てる
    #if sentence[0] == '$GPGGA': 
        #continue
        # 読んだ文字列を解析してGPSオブジェクトにデーターを追加、更新する
    for x in sentence: 
        gps.update(x)
    lat1=gps.latitude[0]
    lon1=gps.longitude[0]
    grs80=pyproj.Geod(ellps='GRS80')
    azimuth,bkw_azimuth,distance=grs80.inv(lon1,lat1,lon2,lat2)
    return (lat1,lon1,azimuth,bkw_azimuth,distance)
    
def getGPS():
    gps_thread = threading.Thread(target=run_gps, args=())
    #gps_thread.daemon = True
    gps_thread.start()
    gps_thread.join()
    x=run_gps()
    lat1=x[0]
    lon1=x[1]
    azimuth=x[2]
    distance=x[4]
    print(lat1,lon1,azimuth,distance)
    return(lat1,lon1,azimuth,distance)
   
def acc_value():
   data = [0, 0, 0, 0, 0, 0]
   acc_data = [0.0, 0.0, 0.0]
   try:
       for i in range(6):
           data[i] = i2c.read_byte_data(ACCL_ADDR, ACCL_R_ADDR + i)
       for i in range(3):
           acc_data[i] = ((data[2*i + 1] * 256) + int(data[2*i] & 0xF0)) / 16
           if acc_data[i] > 2047:
               acc_data[i] -= 4096
           acc_data[i] *= 0.0098
   except IOError as e:
       print("I/O error({0}): {1}".format(e.errno, e.strerror))
   return acc_data

def gyro_value():
   data = [0, 0, 0, 0, 0, 0]
   gyro_data = [0.0, 0.0, 0.0]
   try:
       for i in range(6):
           data[i] = i2c.read_byte_data(GYRO_ADDR, GYRO_R_ADDR + i)
       for i in range(3):
           gyro_data[i] = (data[2*i + 1] * 256) + data[2*i]
           if gyro_data[i] > 32767:
               gyro_data[i] -= 65536
           gyro_data[i] *= 0.0038
   except IOError as e:
       print("I/O error({0}): {1}".format(e.errno, e.strerror))
   return gyro_data

def mag_value():
   data = [0, 0, 0, 0, 0, 0, 0, 0]
   mag_data = [0.0, 0.0, 0.0]
   try:
       for i in range(8):
           data[i] = i2c.read_byte_data(MAG_ADDR, MAG_R_ADDR + i)
       for i in range(3):
           if i != 2:
               mag_data[i] = ((data[2*i + 1] * 256) + (data[2*i] & 0xF8)) / 8
               if mag_data[i] > 4095:
                   mag_data[i] -= 8192
           else:
               mag_data[i] = ((data[2*i + 1] * 256) + (data[2*i] & 0xFE)) / 2
               if mag_data[i] > 16383:
                   mag_data[i] -= 32768
   except IOError as e:
       print("I/O error({0}): {1}".format(e.errno, e.strerror))
   return mag_data

def theta():
    magx=mag_value()[0]+30
    magy=mag_value()[1]-165
    print("magx,magy->",end='')
    print(magx,magy)
    theta=math.atan2(magx,magy)
    theta1=theta*180/math.pi
    if theta1<0:
        theta1=360+theta1
        theta1=math.fabs(360-theta1)
        theta1=theta1
        if theta1 > 360:
            theta1=theta1-360
        elif theta1 <= 360:
            theta1=theta1
    elif theta1>0:
        theta1=theta1
        theta1=math.fabs(360-theta1)
        theta1=theta1
        if theta1 > 360:
            theta1=theta1-360
        elif theta1 <= 360:
            theta1=theta1
    return (theta1)



def getofset():
    data=np.zeros((1000,13))#####################
    for i in range(0,999):#########################
        data[i,0]=mag_value()[0]#x
        data[i,1]=mag_value()[1]#y
        data[i,2]=mag_value()[2]#z
        data[i,3]=(mag_value()[0])*(mag_value()[0])#xx
        data[i,4]=(mag_value()[1])*(mag_value()[1])#yy
        data[i,5]=(mag_value()[2])*(mag_value()[2])#zz
        data[i,6]=mag_value()[0]*mag_value()[1]#xy
        data[i,7]=mag_value()[1]*mag_value()[2]#yz
        data[i,8]=mag_value()[2]*mag_value()[0]#zx
        data[i,9]=(mag_value()[0])*(mag_value()[0])+(mag_value()[1])*(mag_value()[1])+(mag_value()[2])*(mag_value()[2])#xx+yy+zz
        data[i,10]=((mag_value()[0])*(mag_value()[0])+(mag_value()[1])*(mag_value()[1])+(mag_value()[2])*(mag_value()[2]))*mag_value()[0]#x(xx+yy+zz)
        data[i,11]=((mag_value()[0])*(mag_value()[0])+(mag_value()[1])*(mag_value()[1])+(mag_value()[2])*(mag_value()[2]))*mag_value()[1]#y(xx+yy+zz)
        data[i,12]=((mag_value()[0])*(mag_value()[0])+(mag_value()[1])*(mag_value()[1])+(mag_value()[2])*(mag_value()[2]))*mag_value()[2]#z(xx+yy+zz)

    sumdata=np.sum(data,axis=0)

    print(sumdata)

    array1=np.zeros((4,1))
    array1[0,0]=sumdata[9]
    array1[1,0]=sumdata[10]
    array1[2,0]=sumdata[11]
    array1[3,0]=sumdata[12]

    print(array1)

    array2=np.zeros((4,4))
    array2[0,0]=1000########################
    array2[0,1]=sumdata[0]
    array2[0,2]=sumdata[1]
    array2[0,3]=sumdata[2]
    array2[1,0]=sumdata[0]
    array2[1,1]=sumdata[3]
    array2[1,2]=sumdata[6]
    array2[1,3]=sumdata[8]
    array2[2,0]=sumdata[1]
    array2[2,1]=sumdata[6]
    array2[2,2]=sumdata[4]
    array2[2,3]=sumdata[7]
    array2[3,0]=sumdata[2]
    array2[3,1]=sumdata[8]
    array2[3,2]=sumdata[7]
    array2[3,3]=sumdata[5]

    print(array2)

    inv_array2=np.linalg.inv(array2)

    print(inv_array2)

    final=np.dot(inv_array2,array1)

    print(final)

    xofset=final[1]/2
    yofset=final[2]/2
    zofset=final[3]/2
    print(xofset,yofset,zofset)
    return(xofset,yofset,zofset)


SETUP()
while True:
    #getGPS()
    PID()
    time.sleep(0.1)
    if acc_value()[2]<-1:
        stuck=1
        print("stuck ok!!")
        SERVO(-800,-800)
        time.sleep(5)
        SERVO(0,0)
        time.sleep(1)
        if acc_value()[2]<-1:
            stuck=1
            print("mou dame")
        else:
            stuck=0
            print("kaettekita")
    else:
        stuck=
