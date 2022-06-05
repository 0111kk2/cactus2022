import cv2
import numpy as np
from matplotlib import pyplot as plt
im = cv2.imread('asakusa.jpg') #画像の読み込み
hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV) #HSV変換
im2=np.copy(hsv)#行列の作成
im3=np.copy(hsv)
im4=np.copy(hsv)
im5=np.copy(hsv)
im6=np.copy(hsv)
imS=np.copy(hsv)
imA=np.copy(hsv)
#H:0~179,(R:0~30,150,179),S:64~255,V:0~255
im3[:,:,0]=np.where(im2[:,:,0]>179,1 , 0)
im4[:,:,0]=np.where(50>im2[:,:,0], 1, 0)
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
a = np.arange(1108).reshape((1, 1108))
plt.plot(a,arrow2,marker='o')
plt.show()
cv2.imwrite('syorigo.png ', im6)
cv2.imshow('syorigo',im6)
cv2.waitKey(0)
cv2.destroyAllWindows()
import cv2
import numpy as np
from matplotlib import pyplot as plt
im = cv2.imread('.jpg') #画像の読み込み
hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV) #HSV変換
im2=np.copy(hsv)#行列の作成
im3=np.copy(hsv)
im4=np.copy(hsv)
im5=np.copy(hsv)
im6=np.copy(hsv)
imS=np.copy(hsv)
imA=np.copy(hsv)
#H:0~179,(R:0~30,150,179),S:64~255,V:0~255
im3[:,:,0]=np.where(im2[:,:,0]>179,1 , 0)
im4[:,:,0]=np.where(50>im2[:,:,0], 1, 0)
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
a = np.arange(width).reshape((1, width))
plt.plot(a,arrow2,marker='o')
plt.show()
cv2.imwrite('syorigo.png ', im6)
cv2.imshow('syorigo',im6)
cv2.waitKey(0)
cv2.destroyAllWindows()