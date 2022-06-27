from cmath import pi
import cv2
import numpy as np
from matplotlib import pyplot as plt
import time
import picamera
import pigpio
pi=pigpio.pi()

def motor(left,right,seconds):
	SERVO_PIN_R = 17
	SERVO_PIN_L = 18
	#　速く前
	pi.set_servo_pulsewidth( SERVO_PIN_R, 1500 - right )
	pi.set_servo_pulsewidth( SERVO_PIN_L, 1500 + left )
	time.sleep( seconds )
	pi.set_servo_pulsewidth( SERVO_PIN_R, 1500 )
	pi.set_servo_pulsewidth( SERVO_PIN_L, 1500 )

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

if __name__ == "__main__" :
    while True:
        arrow = camera_recognition()
        print(arrow)
        max_index = np.argmax(arrow)
        if max_index>256:
            #左回転
            motor(-600,600,0.1)
        elif max_index<=256:
            #右回転
            motor(600,-600,0.1)







