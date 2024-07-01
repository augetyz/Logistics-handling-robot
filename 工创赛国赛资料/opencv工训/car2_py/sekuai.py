from collections import  deque  
import numpy as np   
import cv2  
import time  
import serial
import struct
redLower_1= np.array([0, 120, 64]) #设定红色阈值，HSV空间 
redUpper_1 = np.array([4, 255, 234])
redLower= np.array([151, 67, 0]) #设定红色阈值，HSV空间 
redUpper = np.array([179, 255, 234])

blueLower = np.array([86, 124, 144])
blueUpper = np.array([125, 255, 255])
greenLower = np.array([49,68,55])
greenUpper = np.array([90,255,255])
mybuffer = 64  #初始化追踪点的列表
pts = deque(maxlen=mybuffer)  
cap = cv2.VideoCapture(0)

ser = serial.Serial("/dev/ttyAMA0",115200)
def send_data_packet(x, y):
    temp = struct.pack("<bbhhhhb",  # 格式为俩个字符俩个整型
                       0x2C,  # 帧头1
                       0x12,  # 帧头2
                       int(x),  # up sample by 4    #数据1
                       int(y),  # up sample by 4    #数据2
                       int(2),
                       int(3),
                       0x5b)
    ser.write(temp)  # 串口发送
n_red = 0
n_blue = 0
n_green = 0

while True:             #遍历每一帧，检测红色瓶盖
    #读取帧
    max_red = 0
    max_blue = 0
    max_green = 0
    max_red_1 = 0
    ret, frame = cap.read()
    if not ret:
        print ('No Camera')
        break
    frame = frame[0:300,0:300]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)   #转到HSV空间   
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)   #转到HSV空间   
    mask1 = cv2.inRange(hsv, redLower, redUpper)  #根据阈值构建掩膜
    mask2 = cv2.inRange(hsv, blueLower, blueUpper)
    mask3 = cv2.inRange(hsv, greenLower, greenUpper)
    mask4 = cv2.inRange(hsv, redLower_1, redUpper_1)
    mask1 = cv2.erode(mask1, None, iterations=2)  #腐蚀操作
    mask2 = cv2.erode(mask2, None, iterations=2)
    mask3 = cv2.erode(mask3,None, iterations=2)
    mask4 = cv2.erode(mask4, None, iterations=2) 
    mask1 = cv2.dilate(mask1, None, iterations=2)#膨胀操作，其实先腐蚀再膨胀的效果是开运算，去除噪点
    mask2 = cv2.dilate(mask2, None, iterations=2)
    mask3 = cv2.dilate(mask3,None, iterations=2)
    mask4 = cv2.dilate(mask4, None, iterations=2)
    cnts1 = cv2.findContours(mask1.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]  #轮廓检测
    cnts2 = cv2.findContours(mask2.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    cnts3 = cv2.findContours(mask3.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    cnts4 = cv2.findContours(mask4.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None  #初始化瓶盖圆形轮廓质心
    #如果存在轮廓  
    if len(cnts1) > 0:    
        c_red = max(cnts1, key = cv2.contourArea) #找到面积最大的轮廓   
        rect_red = cv2.minAreaRect(c_red)  #确定面积最大的轮廓的juxing
        max_red = rect_red[1][0]*rect_red[1][1]
        box_red = cv2.boxPoints(rect_red)  
        #cv2.drawContours(frame, [np.int0(box_red)],-1, (0, 255, 255), 2)  #计算质心
        #print('red' + str(rect_red[0]))
        #print(c_red)
    if len(cnts2) > 0:    
        c_blue = max(cnts2, key = cv2.contourArea)  
        rect_blue = cv2.minAreaRect(c_blue)  #确定面积最大的轮廓的juxing
        max_blue = rect_blue[1][0]*rect_blue[1][1]
        box_blue = cv2.boxPoints(rect_blue)  
        #cv2.drawContours(frame, [np.int0(box_blue)],-1, (0, 255, 255), 2)  #计算质心
        #print(rect_blue[0])
    if len(cnts3) > 0:    
        c_green = max(cnts3, key = cv2.contourArea)  
        rect_green = cv2.minAreaRect(c_green)  #确定面积最大的轮廓的juxing
        max_green = rect_green[1][0]*rect_green[1][1]
        box_green = cv2.boxPoints(rect_green)  
        #cv2.drawContours(frame, [np.int0(box_green)],-1, (0, 255, 255), 2)  #计算质心
        #print(rect_green[0])
    if len(cnts4) > 0:    
        c_red_1 = max(cnts4, key = cv2.contourArea) #找到面积最大的轮廓   
        rect_red_1 = cv2.minAreaRect(c_red_1)  #确定面积最大的轮廓的juxing
        max_red_1 = rect_red_1[1][0]*rect_red_1[1][1]
        box_red_1 = cv2.boxPoints(rect_red_1)  
        #cv2.drawContours(frame, [np.int0(box_red)],-1, (0, 255, 255), 2)  #计算质心
        #print('red' + str(rect_red[0]))
        #print(c_red)
    if (max_red > max_green and max_red > max_blue and max_red > max_red_1 and max_red > 1000):
        cv2.drawContours(frame, [np.int0(box_red)],-1, (0, 0, 255), 2)
        send_data_packet(rect_red[0][0],1)
    elif max_green > max_red and max_green > max_blue and max_green > max_red_1 and max_green > 1000:
        cv2.drawContours(frame, [np.int0(box_green)],-1, (0, 255, 0), 2)
        send_data_packet(rect_green[0][0],2)
    elif max_blue > max_red and max_blue > max_green and max_blue > max_red_1 and max_blue > 1000:
        cv2.drawContours(frame, [np.int0(box_blue)],-1, (255, 0, 0), 2)
        send_data_packet(rect_blue[0][0],3)
    elif (max_red_1 > max_green and max_red_1 > max_blue and max_red_1 > max_red and max_red_1 > 1000):
        cv2.drawContours(frame, [np.int0(box_red_1)],-1, (0, 0, 255), 2)
        send_data_packet(rect_red_1[0][0],1)

    cv2.imshow('Frame', frame)      
    k = cv2.waitKey(5)&0xFF  #键盘检测，检测到esc键退出
    if k == 27:  
        break  
camera.release()  #摄像头释放 
cv2.destroyAllWindows()#销毁所有窗口 
