from re import A
from unittest.mock import MagicProxy
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
SERVO_R=17
SERVO_L=18
# 出力のフーマットは度数とする
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
    theta1=theta()
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
    magx=mag_value()[0]+50#b
    magy=mag_value()[1]-165#c
    #print(mag_value()[0])
    print("magx,magy->",end='')
    print(magx,magy)
    theta=math.atan2(magx,magy)
    print(theta)
    #theta1=theta*180/math.pi
    theta1=math.degrees(theta)
    theta1=theta1-10
    if theta1<0:
        theta1=360+theta1
        theta1=math.fabs(360-theta1)
        theta1=theta1
        if theta1 > 360:
            #theta1=theta1-360
            print("erai")
        elif theta1 <= 360:
            #theta1=theta1
            print("ikemen")
    elif theta1>0:
        theta1=theta1
        theta1=math.fabs(360-theta1)
        theta1=theta1
        if theta1 > 360:
            #theta1=theta1-360
            print("hasimotokannna")
        elif theta1 <= 360:
            theta1=theta1
            #print("imadamiou")
    print('theta1')
    print(theta1)
    '''
    print("magx,magy->",end='')
    print(magx,magy)
    theta=math.atan2(magx,magy)
    print(theta)
    #theta1=theta*180/math.pi
    theta1=math.degrees(theta)
    if theta1<0:
        theta1=360+theta1
        theta1=math.fabs(360-theta1)
        theta1=theta1
        if theta1 > 360:
            #theta1=theta1-360
            print("erai")
        elif theta1 <= 360:
            #theta1=theta1
            print("ikemen")
    elif theta1>0:
        theta1=theta1
        theta1=math.fabs(360-theta1)
        theta1=theta1
        if theta1 > 360:
            #theta1=theta1-360
            print("hasimotokannna")
        elif theta1 <= 360:
            theta1=theta1
            #print("imadamiou")
    print(theta1)
    theta1=theta1+75
    if theta1<360:
        theta1=theta1
    elif theta1>360:
        theta1=theta1-360
    '''
    return (theta1)

class Euler:
    def __init__(self,xoffset,yoffset,zoffset):
        self.xoffset = xoffset
        self.yoffset = yoffset
        self.zoffset = zoffset
        
    def euler(self):#9軸センサの値からオイラー角を出力する関数（ジャイロ不使用）
        #センサ値を取得
        accel = acc_value()
        magnet = mag_value()
        #yの値の符号反転は，センサが右手座標系であるため，これをx軸を北に合わせた左手座標系（グローバル座標系と同義）に直している．
        ax = accel[0]
        ay = -accel[1]
        az = accel[2]
        mx = magnet[0]-self.xoffset
        my = -(magnet[1]-self.yoffset)
        mz = magnet[2]-self.zoffset 
        #オイラー角を算出（参考：http://watako-lab.com/2019/02/20/3axis_cmps/)
        #軸の取り方はちゃんと調節すること！初期設定では磁北とx軸正の方向が一致した時にyawが0となる．
        roll = math.atan2(az,ay)
        pitch = math.atan2(math.sqrt(ay**2+az**2),-ax)
        #=============================================#
        #yaw角が欲しければこの区画内のコメントアウトを解除し，その後をすべてコメントアウト．
        #yaw = math.atan2((math.sin(roll)*my-math.cos(roll)*mz)/(math.cos(pitch)*mx+math.sin(pitch)*math.sin(roll)*my+math.sin(pitch)*math.cos(roll)*mz))
        #return(roll,pitch,yaw)
        #=============================================#

        #今回はyaw角よりも重力によって補正したMagx,Magyの値の方を使いたい．
        Magx = math.cos(pitch)*mx+math.sin(pitch)*math.sin(roll)*my+math.sin(pitch)*math.cos(roll)*mz
        Magy = math.sin(roll)*my-math.cos(roll)*mz
        return(roll,pitch,Magx,Magy)

class P_control:
    def __init__(self,MagNorth,xoffset,yoffset,zoffset):
        #P制御の計算で定数化できるものを最初に算出．
        self.CosMagNorth = math.cos(-MagNorth*math.pi/180)
        self.SinMagNorth = math.sin(-MagNorth*math.pi/180)
        self.Euler = Euler(xoffset,yoffset,zoffset)

    def P_control(self,theta_goal):
        GoalX = math.sin(theta_goal*math.pi/180)
        GoalY = math.cos(theta_goal*math.pi/180)
        #1秒間のP制御
        for i in range(0,999):
            eulers = self.Euler.euler()
            #重力補正をした方位センサ値
            Magx = eulers[2]
            Magy = eulers[3]
            #グローバル直交座標系に座標変換を行った後の方位ベクトル
            DirectX = Magx*self.CosMagNorth-Magy*self.SinMagNorth
            DirectY = Magx*self.SinMagNorth-Magy*self.CosMagNorth
            #目標方位ベクトルとの内積から，なす角を算出
            Theta = math.acos((DirectX*GoalX*DirectY*GoalY)/math.sqrt(DirectX**2+DirectY**2))
            #なす角は正負がないため，右旋回すべきか左旋回すべきかの判断が必要．
            NowTheta = math.atan2(DirectX,DirectY)#まずは真北からのなす角を算出
            Judge = GoalX*math.cos(-NowTheta)-GoalY*math.sin(-NowTheta)#ゴール方向が現在向いている方向より右か左かの判断．ゴール方向をなす角だけ逆回転させてそのx成分の正負を確認する．
            NowThetaDegree = NowTheta*180/math.pi
            if Judge<0:
                SERVO(-NowThetaDegree,NowThetaDegree)
                time.sleep(0,1)
                SERVO(0,0)
            if Judge>=0:
                SERVO(NowThetaDegree,-NowThetaDegree)
                time.sleep(0,1)
                SERVO(0,0)

def getofset():
    
    #mag_value()を一度だけ呼び出して，値を格納したものを下の計算式で使う．
    data=np.zeros((1000,13))#####################
    magnet = mag_value()
    for i in range(0,2999):#######################
        mag_x=magnet[0]
        mag_y=magnet[1]
        mag_z=magnet[2]
        data[i,0]=mag_x#x
        data[i,1]=mag_y#y
        data[i,2]=mag_z#z
        data[i,3]=mag_x**2#xx
        data[i,4]=mag_y**2#yy
        data[i,5]=mag_z**2#zz
        data[i,6]=mag_x*mag_y#xy
        data[i,7]=mag_y*mag_z#yz
        data[i,8]=mag_z*mag_x#zx
        data[i,9]=data[i,3]*data[i,4]*data[i,5]#xx+yy+zz
        data[i,10]=data[i,9]*mag_x#x(xx+yy+zz)
        data[i,11]=data[i,9]*mag_y#y(xx+yy+zz)
        data[i,12]=data[i,9]*mag_z#z(xx+yy+zz)
        time.sleep(0.1)

    sumdata=np.sum(data,axis=0)

    array1=np.zeros((4,1))
    array1[0,0]=sumdata[9]
    array1[1,0]=sumdata[10]
    array1[2,0]=sumdata[11]
    array1[3,0]=sumdata[12]

    #print(array1)

    array2=np.zeros((4,4))
    array2[0,0]=1000
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

    #print(array2)

    inv_array2=np.linalg.inv(array2)

    #print(inv_array2)

    final=np.dot(inv_array2,array1)

    #print(final)

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

