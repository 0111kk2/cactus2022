import cgsensor  # インポート
import time
import math
import datetime
import csv

bme280 = cgsensor.BME280(i2c_addr=0x76)  # BME280制御クラスのインスタンス, i2c_addrは0x76/0x77から選択

bme280.forced()  # Forcedモードで測定を行い, 結果をtemperature, pressure, humidityに入れる
print('気温 {}°C'.format(bme280.temperature))  # 気温を取得して表示
print('湿度 {}%'.format(bme280.humidity))  # 湿度を取得して表示
print('気圧 {}hPa'.format(bme280.pressure))  # 気圧を取得して表示


if __name__ == "__main__": #ターミナルから実行した場合
    time.sleep(0.1)  
    now_time = datetime.datetime.now()
    filename = 'BME280' + now_time.strftime('%Y%m%d_%H%M%S') + '.csv'
    # ファイル，1行目(カラム)の作成
    with open(filename, 'a') as f:
        writer = csv.writer(f)
        writer.writerow(['temperature', 'humidity', 'pressure'])
    while True:
        bme280 = cgsensor.BME280(i2c_addr=0x76)
        bme280.forced() 
        print("temperature:{}, humidity:{}, pressure:{}".format(bme280.temperature,bme280.humidity, bme280.pressure))
        print("")
        print("\n")
        time.sleep(0.1)
        with open(filename, 'a', newline="") as f:
            writer = csv.writer(f)
            writer.writerow([bme280.temperature,bme280.humidity, bme280.pressure])