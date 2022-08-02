from unicodedata import name
from sensor.communication import xbee
from sensor.GPS import gps
import time

if __name__ == "__main__":
    gps.open_gps()
    try:
        while 1:
            utc, lat, lon, sHeight, gHeight = gps.read_gps()
            if utc == -1.0:
                if lat == -1.0:
                    print("Reading gps Error")
                    xbee.str_trans("Reading gps Error")
                    # pass
                else:
                    # pass
                    print("Status V")
                    xbee.str_trans("Status V")
            else:
                #うまくGPS情報がとれた場合
                print(utc, lat, lon, sHeight, gHeight)
                
                #xbeeに送るには変数をstr変換する必要がある。↓
                xbee.str_trans(f'{utc} {lat} {lon} {sHeight} {gHeight}')
            time.sleep(1)
    except:
        gps.close_gps()