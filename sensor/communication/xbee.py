import serial
# from time import time
import time
def str_trans(string):
    """
    文字列を送る関数
    データを送る際は必ずstr型に直して引数に入れる。
    """
    string = str(string)
    ser = serial. Serial(
        port="/dev/serial0",
        baudrate=9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=5
    )
    string = string + '\n'
    moji = string.encode()
    commands = [moji]
    for cmd in commands:
        ser.write(cmd)
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