import serial
import traceback
# from time import time
import time
def str_trans(string):
    """
    文字列を送る関数
    データを送る際は必ずstr型に直して引数に入れる。
    """
    string = str(string)
    ser = serial. Serial(
        # port="/dev/ttyAMA0",
        port="/dev/ttyS0",
        baudrate=9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=5
    )
    try:
        string = string + '\n'

        for cmd in string:
            ser.write(cmd.encode())
        # moji = string.encode()
        # commands = [moji]
        # for cmd in commands:
        #     ser.write(cmd)
    except:
        ser.flush()
        ser.close()
        print("error programam finish")
        traceback.print_exc()
        exit()
    ser.flush()
    ser.close()

if __name__ =="__main__":
    str1 = "aaaa"
    str_trans(str1)
    print(str1)
    time.sleep(1)
    str2 = 123
    str2 = str(str2)
    str_trans(str2)
    print(str2)