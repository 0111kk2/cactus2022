
import pigpio
import time

SERVO_PIN_R = 16
SERVO_PIN_L = 18

pi=pigpio.pi()

while True:
    #　速く前進
    pi.set_servo_pulsewidth( SERVO_PIN_R, 1000 )
    pi.set_servo_pulsewidth( SERVO_PIN_L, 2000 )
    time.sleep( 2 )
    #　遅く前進
    pi.set_servo_pulsewidth( SERVO_PIN_R, 1300 )
    pi.set_servo_pulsewidth( SERVO_PIN_L, 1700 )
    time.sleep( 2 )
    #　速く後退
    pi.set_servo_pulsewidth( SERVO_PIN_R, 2000 )
    pi.set_servo_pulsewidth( SERVO_PIN_L, 1000 )
    time.sleep( 2 )
    #　遅く後退
    pi.set_servo_pulsewidth( SERVO_PIN_R, 1700 )
    pi.set_servo_pulsewidth( SERVO_PIN_L, 1300 )
    time.sleep( 2 )
    #　右超信地
    pi.set_servo_pulsewidth( SERVO_PIN_R, 1700 )
    pi.set_servo_pulsewidth( SERVO_PIN_L, 1700 )
    time.sleep( 2 )
    #　左超信地
    pi.set_servo_pulsewidth( SERVO_PIN_R, 1300 )
    pi.set_servo_pulsewidth( SERVO_PIN_L, 1300 )
    time.sleep( 2 )