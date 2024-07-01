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

greenLower = np.array([52,75,0])
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
    '''
    cos = np.abs(M[0, 0])
    sin = np.abs(M[0, 1])
    new_w = rows * sin + cols * cos
    new_h = rows * cos + cols * sin
    M[0, 2] += (new_w - cols) * 0.5
    M[1, 2] += (new_h - rows) * 0.5
    w = int(np.round(new_w))
    h = int(np.round(new_h))
    '''
    #new frame
    frame = cv2.warpAffine(frame, M, (cols, rows))
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=0)
    mask = cv2.dilate(mask, None, iterations=2)
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    cnts_list = []
    cnts_list_1 = []
    if len(cnts) > 0:
        for i in range(len(cnts)):
            area = cv2.contourArea(cnts[i])
            if area > 2000 and area <= 60000:
                cnts_list.append(cnts[i])
        if len(cnts_list)>0:
            for i in range(len(cnts_list)):
                rect_green = cv2.minAreaRect(cnts_list[i])
                if rect_green[1][0]/rect_green[1][1] <= 1.1 and rect_green[1][0]/rect_green[1][1] >= 0.9:
                    cnts_list_1.append(cnts_list[i])
            if len(cnts_list_1) > 0:       
                c_green = max(cnts_list_1, key = cv2.contourArea)
                epsilon = 0.1 * cv2.arcLength(c_green, True) #多边形拟合的距离参数，下一个函数要用到。原理见代码后链接
                approx = cv2.approxPolyDP(c_green, epsilon, True)
                corners = len(c_green)
                
                #if corners >= 4 and corners <= 10:
                rect_green = cv2.minAreaRect(c_green)  #确定面积最大的轮廓的juxing
                box_green = cv2.boxPoints(rect_green)
                max_green = rect_green[1][0]*rect_green[1][1]
                cv2.drawContours(frame,[np.int0(c_green)] ,-1, (0, 0, 255), 2)  #计算质心
                cv2.drawContours(frame, [np.int0(box_green)],-1, (0, 255, 255), 2)
                x = rect_green[0][0] - 320
                y = -rect_green[0][1] + 240
                print('x:' + str(x))
                print('y:' + str(y))
                print('area:' + str(max_green))
                #send_data_packet(x,y)
   
    cv2.imshow('Frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

