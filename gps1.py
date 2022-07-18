import os
import threading
import time

import serial

from micropy_gps import micropyGPS
from output_csv import write_position

gps = micropyGPS.MicropyGPS()

# 出力のフォーマットは度数とする
gps.coord_format = 'dd'

def run_gps(): 
    """
    GPSモジュールを読み、GPSオブジェクトを更新する
    :return: None
    """ 

    s = serial.Serial('/dev/serial0/ttyS0', 9600, timeout=10)

    # 最初の1行は中途半端なデーターが読めることがあるので、捨てる
    s.readline()

    while True:

        # GPSデーターを読み、文字列に変換する
        sentence = s.readline().decode('utf-8')  

        # 先頭が'$'でなければ捨てる
        if sentence[0] != '$': 
            continue

        # 読んだ文字列を解析してGPSオブジェクトにデーターを追加、更新する
        for x in sentence: 
            gps.update(x)


# 上の関数を実行するスレッドを生成
gps_thread = threading.Thread(target=run_gps, args=())

gps_thread.daemon = True

# スレッドを起動
gps_thread.start()  

while True:

    # ちゃんとしたデーターがある程度たまったら出力する
    if gps.clean_sentences > 20:
        h = gps.timestamp[0] if gps.timestamp[0] < 24 else gps.timestamp[0] - 24
        print('%2d:%02d:%04.1f' % (h, gps.timestamp[1], gps.timestamp[2]))
        print('緯度経度: %2.8f, %2.8f' % (gps.latitude[0], gps.longitude[0]))
        print('海抜: %f' % gps.altitude)
        print('スピード: %f' % gps.speed[2])
        print('測位利用衛星: %s' % gps.satellites_used)
        print('衛星番号: (仰角, 方位角, SN比)')
        for k, v in gps.satellite_data.items():
            print('%d: %s' % (k, v))
        print('')

    # 1秒に1回実行するように
    time.sleep(1.0)