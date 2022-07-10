from tkinter import OFF
import cgsensor
import RPi.GPIO as GPIO
import datetime
from re import T
import numpy as np
import os
import time
import serial
import pyproj
import micropyGPS
import csv
import math
import pigpio
from smbus import SMBus
from cmath import pi
import cv2
from matplotlib import pyplot as plt
import time
pi=pigpio.pi()
#lat2,lon2=35.7525063,139.7380564 #王子駅 , 神楽坂から見た真北
lat2,lon2=51.2840000,0.0000564 #グリニッジ天文台 テスト用 
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
def SERVO(a,b):#サーボモーターの回転の1500-2300を0-800に変換，左右を無視して速度だけで書けるようにした
    pi.set_servo_pulsewidth(SERVO_L,1485+a)
    pi.set_servo_pulsewidth(SERVO_R,1480-b)
    
def PID(theta_goal):
    for i in range(100):
        theta1=theta()
        theta_diff = math.fabs(theta_goal-theta1)
        if theta_goal>theta1:
            SERVO(500+theta_diff,500)
        else:
            SERVO(500,500+theta_diff)
        time.sleep(0.1)
        
        
def run_gps(): 
    """
    GPSモジュールを読み、GPSオブジェクトを更新する
    :return: None
    """
    s = serial.Serial('/dev/serial0', 9600, timeout=10)
    #print(s)
    # 最初の1行は中途半端なデーターが読めることがあるので、捨てる
    s.readline()
    #print(s.readline())
       # print(s.readline())
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
def theta(b,c):
    magx=mag_value()[0]-b
    magy=mag_value()[1]-c
    print("magx,magy->",end='')
    print(magx,magy)
    theta=math.atan2(magx,magy)
    theta1=theta*180/math.pi
    theta1=math.fabs(theta1)
    return theta1
def getofset():
    SERVO(300,-300)
    data=np.zeros((1000,13))
    for i in range(0,999):
        Input_data = mag_value()
        data[i][0]=Input_data[0]#x
        data[i][1]=Input_data[1]#y
        data[i][2]=Input_data[2]#z
        data[i][3]=Input_data[0]**2#xx
        data[i][4]=Input_data[1]**2#yy
        data[i][5]=Input_data[2]**2#zz
        data[i][6]=Input_data[0]*Input_data[1]#xy
        data[i][7]=Input_data[1]*Input_data[2]#yz
        data[i][8]=Input_data[2]*Input_data[0]#zx
        data[i][9]=Input_data[0]**2+Input_data[1]**2+Input_data[2]**2#xx+yy+zz
        data[i][10]=data[i][9]*Input_data[0]#x(xx+yy+zz)
        data[i][11]=data[i][9]*Input_data[1]#y(xx+yy+zz)
        data[i][12]=data[i][9]*Input_data[2]#z(xx+yy+zz)
    sumdata=np.sum(data,axis=0)
    #print(sumdata)
    array1=np.zeros((4,1))
    array1[0][0]=sumdata[9]
    array1[1][0]=sumdata[10]
    array1[2][0]=sumdata[11]
    array1[3][0]=sumdata[12]
    #print(array1)
    array2=np.zeros((4,4))
    array2[0][0]=1000########################
    array2[0][1]=sumdata[0]
    array2[0][2]=sumdata[1]
    array2[0][3]=sumdata[2]
    array2[1][0]=sumdata[0]
    array2[1][1]=sumdata[3]
    array2[1][2]=sumdata[6]
    array2[1][3]=sumdata[8]
    array2[2][0]=sumdata[1]
    array2[2][1]=sumdata[6]
    array2[2][2]=sumdata[4]
    array2[2][3]=sumdata[7]
    array2[3][0]=sumdata[2]
    array2[3][1]=sumdata[8]
    array2[3][2]=sumdata[7]
    array2[3][3]=sumdata[5]
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
def camera_recognition():
    now = str(time.time())
    fn = now+'pi_pic.jpg'
    ffn = now+'syorigo.jpg'
    # カメラ初期化
    with picamera.PiCamera() as camera:
        # 解像度の設定
        camera.resolution = (512, 384)
        # 撮影の準備
        camera.start_preview()
        # 準備している間、少し待機する
        time.sleep(0.1)
        # 撮影して指定したファイル名で保存する
        camera.capture(fn)
        
        cv2.destroyAllWindows()
    im = cv2.imread(fn) #画像の読み込み
    hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV) #HSV変換
    im2=np.copy(hsv)#行列の作成
    im3=np.copy(hsv)
    im4=np.copy(hsv)
    im5=np.copy(hsv)
    im6=np.copy(hsv)
    imS=np.copy(hsv)
    imA=np.copy(hsv)
    #H:0~179,(R:0~30,150,179),S:64~255,V:0~255
    im3[:,:,0]=np.where(im2[:,:,0]>150,1 , 0)
    im4[:,:,0]=np.where(0>im2[:,:,0], 1, 0)
    imS[:,:,0]=np.where(150>im2[:,:,1],1,0)
    im5=im3+im4-imS
    imA[:,:,0]=np.where(im5[:,:,0]==1,1,0)
    print(im5)
    im6[:,:,0]=np.where(imA[:,:,0]==1,50,60)
    im6[:,:,1]=np.where(imA[:,:,0]==1,50,64)
    im6[:,:,2]=np.where(imA[:,:,0]==1,255,0)
    print(im6)
    height, width, channels = im6.shape[:3]
    arrow1=np.ones((1,height))
    imA_1=imA[:,:,0]
    print(width)
    print("im5 is")
    print(imA_1)
    arrow2=np.dot(arrow1,imA_1)
    print(arrow2)
    #a = np.arange(512).reshape((1, 512))
    #plt.plot(a,arrow2,marker='o')
    #plt.show()
    #cv2.imwrite(ffn, im6)
    #cv2.imshow(ffn,im6)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    return arrow2
def write_data(f):
    acc = acc_value()
    print("Accl -> x:{}, y:{}, z: {}".format(acc[0], acc[1], acc[2]))
    print("\n")
    time.sleep(0.1)
    bme280 = cgsensor.BME280(i2c_addr=0x76)  # BME280制御クラスのインスタンス, i2c_addrは0x76/0x77から選択
    bme280.forced()  # Forcedモードで測定を行い, 結果をtemperature, pressure, humidityに入れる
    print('気温 {}°C'.format(bme280.temperature))  # 気温を取得して表示
    print('湿度 {}%'.format(bme280.humidity))  # 湿度を取得して表示
    print('気圧 {}hPa'.format(bme280.pressure))  # 気圧を取得して表示
    writer = csv.writer(f)
    writer.writerow([acc[0], acc[1], acc[2],bme280.temperature,bme280.humidity,bme280.pressure])
    ax=acc[0]
    ay=acc[1]
    az=acc[2]
    acc_abs=np.sqrt(ax**2+ay**2+az**2)
    return acc_abs
if __name__ == '__main__':
    #bmxの初期化
    bmx_setup()
    #着地判定
    led = 16
    i = 1
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(led,GPIO.OUT)
    GPIO.output(led, False)
    time.sleep(0.1)
    now_time = datetime.datetime.now()
    filename = 'BMX055' + now_time.strftime('%Y%m%d_%H%M%S') + '.csv'
    # ファイル，1行目(カラム)の作成
    with open(filename, 'a') as f:
        writer = csv.writer(f)
        writer.writerow(['Acc_x', 'Acc_y', 'Acc_z','temp','humid','press'])
    
    f=open(filename, 'a', newline="")
    while i:
        acc_abs=write_data(f)
        # 落下判定
        if acc_abs<9.5: # 実証実験で正規分布を取得し、値を決める
            start_time=time.time()
            bme280 = cgsensor.BME280(i2c_addr=0x76)  # BME280制御クラスのインスタンス, i2c_addrは0x76/0x77から選択
            bme280.forced()  # Forcedモードで測定を行い, 結果をtemperature, pressure, humidityに入れる
            first_pressure = bme280.pressure
            judge_pressure = first_pressure + 3
            while acc_abs>9.5:
                now_time=time.time()
                count=now_time-start_time
                acc_abs=write_data(f)                
                bme280 = cgsensor.BME280(i2c_addr=0x76)  # BME280制御クラスのインスタンス, i2c_addrは0x76/0x77から選択
                bme280.forced()  # Forcedモードで測定を行い, 結果をtemperature, pressure, humidityに入れる
                now_pressure = bme280.pressure
                GPIO.output(led,False)
                if count>=60: # 60秒経過後パラシュート溶断
                    GPIO.output(led, True) # 回路班の上げた回路図より
                    f.close()
                    i=0
                elif now_pressure > judge_pressure:
                    GPIO.output(led, True)
                    f.close()
                    i=0
    GPIO.cleanup()# 着地判定
        #駆動班のやったこと
    i=1
    while i:
        #GPSの情報取得
        GPS_data = run_gps()
        if GPS_data[4]<10:
            i=0
        else:
        #PID制御
            PID(GPS_data[2])
    #距離が一定以下なら画像処理に移行
    while True:
        i=1
        SERVO(300,-300)
        time.sleep(0.1)
        arrow = camera_recognition()
        if np.argmax(arrow)>10:
            while i:
                arrow = camera_recognition()
                if np.argmax(arrow)<10:
                    i=0
                elif np.argmax(arrow)>=384:
                    SERVO(0,0)
                    break
                else:
                    print(arrow)
                    max_index = np.argmax(arrow)
                    if max_index>=275:
                        #左回転
                        SERVO(-600,600)
                        time.sleep(0.1)
                        SERVO(0,0)
                    elif max_index<=247:
                        #右回転
                        SERVO(600,-600)
                        time.sleep(0.1)
                        SERVO(0,0)
                    else:#前進
                        SERVO(600,600)
                        time.sleep(1)
                        SERVO(0,0)
折りたたむ