import cv2
import numpy as np
import math
import serial
import struct
from collections import  deque
cap = cv2.VideoCapture(0)
cap.set(6,cv2.VideoWriter.fourcc('M','J','P','G'))
cap.set(3,640)
cap.set(4,480)

greenLower = np.array([57,26,54])
greenUpper = np.array([100,255,255])
mybuffer = 64  #初始化追踪点的列表
pts = deque(maxlen=mybuffer) 

ser = serial.Serial("/dev/ttyAMA0",115200)
if ser.isOpen == False:
    ser.open()
    
def send_data_packet(x, y):
    temp = struct.pack("<bbhhhhb",  # 格式为俩个字符俩个整型
                       0x2C,  # 帧头1
                       0x3C,  # 帧头2
                       int(x),  # up sample by 4    #数据1
                       int(y),  # up sample by 4    #数据2
                       int(2),
                       int(3),
                       0x5b)
    ser.write(temp)  # 串口发送


while True:
    ret, frame = cap.read()
    rows, cols = frame.shape[:2]
    M = cv2.getRotationMatrix2D((cols / 2, rows / 2), 20, 1)
    #第一个参数旋转中心，第二个参数旋转角度，第三个参数：缩放比例
    #自适应图片边框大小
    frame = cv2.warpAffine(frame, M, (cols, rows))
    frame1 = frame.copy
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask1 = mask.copy()
    mask2 = mask.copy()
    res = cv2.bitwise_and(frame,frame,mask = mask)
    res1 = cv2.cvtColor(res,cv2.COLOR_HSV2BGR)
    gray = cv2.cvtColor(res1,cv2.COLOR_BGR2GRAY)
    #mask = cv2.erode(mask, None, iterations=0)
    #mask = cv2.dilate(mask, None, iterations=1)
    #cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    img = cv2.GaussianBlur(gray,(3,3),0)  #
    cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
    circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 1, 20, param1=200, param2=30, minRadius=0, maxRadius=0)  # 霍夫梯度法
    
    if circles is not None:
        for i in circles[0, 0:1]:
            cv2.circle(cimg, (int(i[0]), int(i[1])), int(i[2]), (0, 255, 0), 2)
            cv2.circle(cimg, (int(i[0]), int(i[1])), 2, (0, 255, 0), 2)

     
   
    cv2.imshow('Frame', frame)
    
    cv2.imshow('image', cimg)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
