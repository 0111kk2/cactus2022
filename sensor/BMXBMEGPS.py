# -*- coding: utf-8 -*-
from smbus import SMBus
import cgsensor
import time
import math
import datetime
import csv
import pigpio
import numpy as np
import traceback



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

RX = 20
pi = pigpio.pi()

ELLIPSOID_GRS80 = 1  # GRS80
ELLIPSOID_WGS84 = 2  # WGS84

# Long Axis Radius and Flat Rate
GEODETIC_DATUM = {
    ELLIPSOID_GRS80: [
        6378137.0,         # [GRS80] Long Axis Radius
        1 / 298.257222101,  # [GRS80] Flat Rate
    ],
    ELLIPSOID_WGS84: [
        6378137.0,         # [WGS84] Long Axis Radius
        1 / 298.257223563,  # [WGS84] Flat Rate
    ],
}

# Limited times of Itereation
ITERATION_LIMIT = 1000


def open_gps():
    try:
        pi.set_mode(RX, pigpio.INPUT)
        pi.bb_serial_read_open(RX, 9600, 8)
    except pigpio.error as e:
        #print("Open gps Error")
        pi.set_mode(RX, pigpio.INPUT)
        pi.bb_serial_read_close(RX)
        pi.bb_serial_read_open(RX, 9600, 8)


def read_gps():
    utc = -1.0
    Lat = -1.0
    Lon = 0.0
    sHeight = 0.0
    gHeight = 0.0
    value = [0.0, 0.0, 0.0, 0.0, 0.0]

    (count, data) = pi.bb_serial_read(RX)
    if count:
        # print(data)
        # print(type(data))
        if isinstance(data, bytearray):
            gpsData = data.decode('utf-8', 'replace')

        # print(gpsData)
        # print()
        # print()
        gga = gpsData.find('$GPGGA,')
        rmc = gpsData.find('$GPRMC,')
        gll = gpsData.find('$GPGLL,')

        # print(gpsData[rmc:rmc+20])
        # print(gpsData[gll:gll+40])
        if gpsData[gga:gga+20].find(",0,") != -1 or gpsData[rmc:rmc+20].find("V") != -1 or gpsData[gll:gll+60].find("V") != -1:
            utc = -1.0
            Lat = 0.0
            Lon = 0.0
        else:
            utc = -2.0
            if gpsData[gga:gga+60].find(",N,") != -1 or gpsData[gga:gga+60].find(",S,") != -1:
                gpgga = gpsData[gga:gga+72].split(",")
                # print(gpgga)
                if len(gpgga) >= 6:
                    utc = gpgga[1]
                    lat = gpgga[2]
                    lon = gpgga[4]
                    try:
                        utc = float(utc)
                        Lat = round(float(lat[:2]) + float(lat[2:]) / 60.0, 6)
                        Lon = round(float(lon[:3]) + float(lon[3:]) / 60.0, 6)
                    except:
                        utc = -2.0
                        Lat = 0.0
                        Lon = 0.0
                    if gpgga[3] == "S":
                        Lat = Lat * -1
                    if gpgga[5] == "W":
                        Lon = Lon * -1
                else:
                    utc = -2.0
                if len(gpgga) >= 12:
                    try:
                        sHeight = float(gpgga[9])
                        gHeight = float(gpgga[11])
                    except:
                        pass
                    #print(sHeight, gHeight)
            # print(gpsData[gll:gll+60].find("A"))
            if gpsData[gll:gll+40].find("N") != -1 and utc == -2.0:
                gpgll = gpsData[gll:gll+72].split(",")
                # print(gpgll)
                # print("a")
                if len(gpgll) >= 6:
                    utc = gpgll[5]
                    lat = gpgll[1]
                    lon = gpgll[3]
                    try:
                        utc = float(utc)
                        Lat = round(float(lat[:2]) + float(lat[2:]) / 60.0, 6)
                        Lon = round(float(lon[:3]) + float(lon[3:]) / 60.0, 6)
                    except:
                        utc = -2.0
                    if gpgll[2] == "S":
                        Lat = Lat * -1
                    if gpgll[4] == "W":
                        Lon = Lon * -1
                else:
                    utc = -2.0
            if gpsData[rmc:rmc+20].find("A") != -1 and utc == -2.0:
                gprmc = gpsData[rmc:rmc+72].split(",")
                # print(gprmc)
                # print("b")
                if len(gprmc) >= 7:
                    utc = gprmc[1]
                    lat = gprmc[3]
                    lon = gprmc[5]
                    try:
                        utc = float(utc)
                        Lat = round(float(lat[:2]) + float(lat[2:]) / 60.0, 6)
                        Lon = round(float(lon[:3]) + float(lon[3:]) / 60.0, 6)
                    except:
                        utc = -1.0
                        Lat = 0.0
                        Lon = 0.0
                    if(gprmc[4] == "S"):
                        Lat = Lat * -1
                    if(gprmc[6] == "W"):
                        Lon = Lon * -1
                else:
                    utc = -1.0
                    Lat = -1.0
                    Lon = 0.0
            if utc == -2.0:
                utc = -1.0
                Lat = -1.0
                Lon = 0.0

    value = [utc, Lat, Lon, sHeight, gHeight]
    for i in range(len(value)):
        if not (isinstance(value[i], int) or isinstance(value[i], float)):
            value[i] = 0
    return value


def close_gps():
    pi.bb_serial_read_close(RX)
    pi.stop()


def cal_rhoang(lat_a, lon_a, lat_b, lon_b):
    if(lat_a == lat_b and lon_a == lon_b):
        return 0.0, 0.0
    ra = 6378.140  # equatorial radius (km)
    rb = 6356.755  # polar radius (km)
    F = (ra-rb)/ra  # flattening of the earth
    rad_lat_a = np.radians(lat_a)
    rad_lon_a = np.radians(lon_a)
    rad_lat_b = np.radians(lat_b)
    rad_lon_b = np.radians(lon_b)
    pa = np.arctan(rb/ra*np.tan(rad_lat_a))
    pb = np.arctan(rb/ra*np.tan(rad_lat_b))
    xx = np.arccos(np.sin(pa)*np.sin(pb) + np.cos(pa) *
                    np.cos(pb)*np.cos(rad_lon_a-rad_lon_b))
    c1 = (np.sin(xx)-xx)*(np.sin(pa) + np.sin(pb))**2 / np.cos(xx/2)**2
    c2 = (np.sin(xx)+xx)*(np.sin(pa) - np.sin(pb))**2 / np.sin(xx/2)**2
    dr = F/8*(c1-c2)
    rho = ra*(xx + dr) * 1000  # Convert To [m]
    angle = math.atan2(lon_a-lon_b,  lat_b-lat_a) * 180 / math.pi  # [deg]
    return rho, angle


def vincenty_inverse(lat1, lon1, lat2, lon2, ellipsoid=None):
    if lat1 == lat2 and lon1 == lon2:
        return 0.0, 0.0

    # Calculate Short Axis Radius
    # if Ellipsoid is not specified, it uses GRS80
    a, f = GEODETIC_DATUM.get(ellipsoid, GEODETIC_DATUM.get(ELLIPSOID_GRS80))
    b = (1 - f) * a

    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    lambda1 = math.radians(lon1)
    lambda2 = math.radians(lon2)

    # Corrected Latitude
    U1 = math.atan((1 - f) * math.tan(phi1))
    U2 = math.atan((1 - f) * math.tan(phi2))

    sinU1 = math.sin(U1)
    sinU2 = math.sin(U2)
    cosU1 = math.cos(U1)
    cosU2 = math.cos(U2)

    # Diffrence of Longtitude between 2 points
    L = lambda2 - lambda1

    # Reset lamb to L
    lamb = L

    # Calculate lambda untill it converges
    # if it doesn't converge, returns None
    for i in range(ITERATION_LIMIT):
        sinLambda = math.sin(lamb)
        cosLambda = math.cos(lamb)
        sinSigma = math.sqrt((cosU2 * sinLambda) ** 2 +
                             (cosU1 * sinU2 - sinU1 * cosU2 * cosLambda) ** 2)
        cosSigma = sinU1 * sinU2 + cosU1 * cosU2 * cosLambda
        sigma = math.atan2(sinSigma, cosSigma)
        sinAlpha = cosU1 * cosU2 * sinLambda / sinSigma
        cos2Alpha = 1 - sinAlpha ** 2
        cos2Sigmam = cosSigma - 2 * sinU1 * sinU2 / cos2Alpha
        C = f / 16 * cos2Alpha * (4 + f * (4 - 3 * cos2Alpha))
        lambdaʹ = lamb
        lamb = L + (1 - C) * f * sinAlpha * (sigma + C * sinSigma *
                                             (cos2Sigmam + C * cosSigma * (-1 + 2 * cos2Sigmam ** 2)))

        # Deviation is udner 1e-12, break
        if abs(lamb - lambdaʹ) <= 1e-12:
            break
    else:
        return None

    # if it converges, calculates distance and angle
    u2 = cos2Alpha * (a ** 2 - b ** 2) / (b ** 2)
    A = 1 + u2 / 16384 * (4096 + u2 * (-768 + u2 * (320 - 175 * u2)))
    B = u2 / 1024 * (256 + u2 * (-128 + u2 * (74 - 47 * u2)))
    dSigma = B * sinSigma * (cos2Sigmam + B / 4 * (cosSigma * (-1 + 2 * cos2Sigmam ** 2) -
                                                   B / 6 * cos2Sigmam * (-3 + 4 * sinSigma ** 2) * (-3 + 4 * cos2Sigmam ** 2)))

    s = b * A * (sigma - dSigma)  # Distance between 2 points
    alpha = -1 * math.atan2(cosU2 * sinLambda, cosU1 * sinU2 -
                            sinU1 * cosU2 * cosLambda)  # Angle between 2 points

    # return s(distance), and alpha(angle)
    return s, math.degrees(alpha)


def gps_data_read():
    '''
    GPSを読み込むまでデータをとり続ける関数
    '''
    try:
        while True:
            utc, lat, lon, sHeight, gHeight = read_gps()
            print('gps reading')
            if utc != -1.0 and lat != -1.0:
                break
            time.sleep(1)
        return utc, lat, lon, sHeight, gHeight
    except KeyboardInterrupt:
        close_gps()
        print("\r\nKeyboard Intruppted, Serial Closed")


def location():
    try:
        while True:
            utc, lat, lon, sHeight, gHeight = read_gps()
            if utc != -1.0 and lat != -1.0:
                break
            time.sleep(1)
        return lat, lon
    except KeyboardInterrupt:
        close_gps()
        print("\r\nKeyboard Intruppted, Serial Closed")



if __name__ == "__main__": #ターミナルから実行した場合
    bmx_setup()
    time.sleep(0.1)
    now_time = datetime.datetime.now()
    filename = 'BMX055' + now_time.strftime('%Y%m%d_%H%M%S') + '.csv'
    # ファイル，1行目(カラム)の作成
    with open(filename, 'a') as f:
        writer = csv.writer(f)
        writer.writerow(['Mag_x', 'Mag_y', 'Mag_z'])
    while True:
        acc = acc_value()
        gyro= gyro_value()
        mag = mag_value()
        print("Accl -> x:{}, y:{}, z: {}".format(acc[0], acc[1], acc[2]))
        print("Gyro -> x:{}, y:{}, z: {}".format(gyro[0], gyro[1], gyro[2]))
        print("Mag -> x:{}, y:{}, z: {}".format(mag[0], mag[1], mag[2]))
        print("\n")
        time.sleep(0.1)
        with open(filename, 'a', newline="") as f:
            writer = csv.writer(f)
            writer.writerow([mag[0], mag[1], mag[2]])
        try:
            open_gps()
            t_start = time.time()
            while True:
                utc, lat, lon, sHeight, gHeight = read_gps()
                if utc == -1.0:
                    if lat == -1.0:
                        print("Reading gps Error")
                        # pass
                    else:
                        # pass
                        print("Status V")
                else:
                    # pass
                    print(utc, lat, lon, sHeight, gHeight)
                time.sleep(1)
        except KeyboardInterrupt:
            close_gps()
            print("\r\nKeyboard Intruppted, Serial Closed")
        except:
            close_gps()
            print(traceback.format_exc())
        
        bme280.forced()  # Forcedモードで測定を行い, 結果をtemperature, pressure, humidityに入れる
        print('気温 {}°C'.format(bme280.temperature))  # 気温を取得して表示
        print('湿度 {}%'.format(bme280.humidity))  # 湿度を取得して表示
        print('気圧 {}hPa'.format(bme280.pressure))  # 気圧を取得して表示