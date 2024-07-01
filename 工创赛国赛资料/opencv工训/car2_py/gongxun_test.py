import numpy as np
import cv2
import math
import serial
import struct
from collections import  deque
import pyzbar.pyzbar as pyzbar
pi = math.pi
cap = cv2.VideoCapture(0)
cap.set(6,cv2.VideoWriter.fourcc('M','J','P','G'))
cap.set(3,640)
cap.set(4,480)
font = cv2.FONT_HERSHEY_SIMPLEX  # 设置字体样式
kernel = np.ones((5, 5), np.uint8)  # 卷积核
ser = serial.Serial("/dev/ttyAMA0",115200)
if ser.isOpen == False:
    ser.open()                # 打开串口



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
'''
# 定义旋转函数
def ImageRotate(image):
 
    height, width = image.shape[:2]    # 输入(H,W,C)，取 H，W 的zhi
    center = (width / 2, height / 2)   # 绕图片中心进行旋转
    angle = 90 # 旋转方向取（-180，180）中的随机整数值，负为逆时针，正为顺势针
    scale = 1                        # 将图像缩放为80%
 
    # 获得旋转矩阵
    M = cv2.getRotationMatrix2D(center, angle, scale)
    # 进行仿射变换，边界填充为255，即白色，默认为黑色
    image_rotation = cv2.warpAffine(src=image, M=M, dsize=(height, width), borderValue=(255, 255, 255))
 
    return image_rotation
'''
def erweima():
    b = 0
    task1 = ser.read()
    while (task1 != b'D' and task1 != b'B'):
           # 读取二维码
        ret, frame = cap.read()
        #cv2.imshow("recognize_face", frame)
        h1, w1 = frame.shape[0], frame.shape[1]
        text = pyzbar.decode(frame)
        for texts in text:
            textdate = texts.data.decode('utf-8')
            print(textdate)
            (x,y,w,h) = texts.rect # 获取二维码的外接矩形顶点坐标
            print('识别内容:' + textdate)
            
            # 二维码中心坐标
            cx = int(x + w / 2)
            cy = int(y + h / 2)
            cv2.circle(frame, (cx, cy), 2, (0, 255, 0), 8)  # 做出中心坐标
            print('中间点坐标：', cx, cy)
            coordinate = (cx, cy)

            # 在画面左上角写出二维码中心位置
            cv2.putText(frame, 'QRcode_location' + str(coordinate), (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # 画出画面中心与二维码中心的连接线
            cv2.line(frame, (cx, cy), (int(w1 / 2), int(h1 / 2)), (255, 0, 0), 2)
            # cv2.rectangle(dst, (x, y), (x + w, y + h), (0, 255, 255), 2)  # 做出外接矩形

            # 二维码最小矩形
            cv2.line(frame, texts.polygon[0], texts.polygon[1], (255, 0, 0), 2)
            cv2.line(frame, texts.polygon[1], texts.polygon[2], (255, 0, 0), 2)
            cv2.line(frame, texts.polygon[2], texts.polygon[3], (255, 0, 0), 2)
            cv2.line(frame, texts.polygon[3], texts.polygon[0], (255, 0, 0), 2)

            # 写出扫描内容
            txt = '(' + texts.type + ')  ' + textdate
            cv2.putText(frame, txt, (x - 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 50, 255), 2)
            tep = ",!"+"0" + textdate + "["
            ser.write(tep.encode("utf-8"))

    # 显示图片
    
        cv2.imshow("result", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        size = ser.inWaiting()
        if(size != 0):
            task1 = ser.read()
            ser.write(task1)
def sekuai():
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
    
    b = 0
    task2 = ser.read()
    while (task2 != b'D' and task2 != b'A'):
        ret, frame = cap.read()
        if not ret:
            print ('No Camera')
            break
        frame = frame[0:200,0:300]
        max_red = 0
        max_blue = 0
        max_green = 0
        
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
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        size = ser.inWaiting()
        if(size != 0):
            task2 = ser.read()
            ser.write(task2)          
    
while(True):
    task = ser.read(1)
    #ser.flushInput()
    if(task == b'A'):
        erweima()
    elif(task == b'B'):
        sekuai()
    elif(task == b'C'):
        ser.write("tiaoxingma")
cap.release()
cv2.destroyAllWindows()
