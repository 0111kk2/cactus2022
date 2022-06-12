# -*- coding: utf-8 -*-
from tkinter import OFF
from smbus import SMBus
import cgsensor
import pigpio as GPIO
import numpy as np
import time
import math
import datetime
import csv



# I2C
ACCL_ADDR = 0x19
ACCL_R_ADDR = 0x02
GYRO_ADDR = 0x69
GYRO_R_ADDR = 0x02
MAG_ADDR = 0x13
MAG_R_ADDR = 0x42
i2c = SMBus(1)

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
        for i in range(6):#各軸に関して2byteずつ存在している
            data[i] = i2c.read_byte_data(ACCL_ADDR, ACCL_R_ADDR + i) #1byteよんだら1byte隣に追加
        for i in range(3): #3軸
            acc_data[i] = ((data[2*i + 1] * 256) + int(data[2*i] & 0xF0)) / 16
            if acc_data[i] > 2047: #+-
                acc_data[i] -= 4096
            acc_data[i] *= 0.0098
    except IOError as e: #例外処理
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

bme280 = cgsensor.BME280(i2c_addr=0x76)  # BME280制御クラスのインスタンス, i2c_addrは0x76/0x77から選択

bme280.forced()  # Forcedモードで測定を行い, 結果をtemperature, pressure, humidityに入れる
print('気温 {}°C'.format(bme280.temperature))  # 気温を取得して表示
print('湿度 {}%'.format(bme280.humidity))  # 湿度を取得して表示
print('気圧 {}hPa'.format(bme280.pressure))  # 気圧を取得して表示

def write_data(f):
    acc = acc_value()
    gyro= gyro_value()
    mag = mag_value()
    print("Accl -> x:{}, y:{}, z: {}".format(acc[0], acc[1], acc[2]))
    print("Gyro -> x:{}, y:{}, z: {}".format(gyro[0], gyro[1], gyro[2]))
    print("Mag -> x:{}, y:{}, z: {}".format(mag[0], mag[1], mag[2]))
    print("\n")
    time.sleep(0.1)
    writer = csv.writer(f)
    writer.writerow([mag[0], mag[1], mag[2]])
    ax=acc[0]
    ay=acc[1]
    az=acc[2]
    acc_abs=np.sqrt(ax**2+ay**2+az**2)
    return acc_abs

if __name__ == "__main__": #ターミナルから実行した場合
    bmx_setup()
    GPIO.output(16, 0)
    time.sleep(0.1)
    now_time = datetime.datetime.now()
    filename = 'BMX055' + now_time.strftime('%Y%m%d_%H%M%S') + '.csv'
    # ファイル，1行目(カラム)の作成
    with open(filename, 'a') as f:
        writer = csv.writer(f)
        writer.writerow(['Mag_x', 'Mag_y', 'Mag_z'])
    while True:
        f=open(filename, 'a', newline="")
        acc_abs=write_data(f)
        # 落下判定
        if acc_abs<9.8:
            start_time=time.time()
            while True:
                now_time=time.time()
                count=now_time-start_time
                write_data(f)
                if count>=60: # 60秒経過後パラシュート溶断
                    GPIO.output(16, 0) # 回路班の上げた回路図より
                    f.close()
                    break

        # 着地判定