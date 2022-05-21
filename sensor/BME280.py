import cgsensor  # インポート

bme280 = cgsensor.BME280(i2c_addr=0x76)  # BME280制御クラスのインスタンス, i2c_addrは0x76/0x77から選択

bme280.forced()  # Forcedモードで測定を行い, 結果をtemperature, pressure, humidityに入れる
print('気温 {}°C'.format(bme280.temperature))  # 気温を取得して表示
print('湿度 {}%'.format(bme280.humidity))  # 湿度を取得して表示
print('気圧 {}hPa'.format(bme280.pressure))  # 気圧を取得して表示