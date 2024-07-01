import cv2
import numpy as np
import pyzbar.pyzbar as pyzbar
import math
import serial
import struct

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
while True:
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
cv2.waitKey(0)
cv2.destroyAllWindows()