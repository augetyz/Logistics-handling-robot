from maix import image, display, camera
import struct
import serial, time
ser = serial.Serial("/dev/ttyS1",115200, timeout=0.2)
def send_data_packet(x, y):
    temp = struct.pack("<bbhhb",                #格式为俩个字符俩个整型
                   0x2C,                       #帧头1
                   0x12,                       #帧头2
                   int(x), # up sample by 4    #数据1
                   int(y), # up sample by 4    #数据2
                   0x5b)
    ser.write(temp)                           #串口发送
while True:
    img = camera.capture()
    line = img.find_line()  #rect： 矩形的四个顶点 pixels：矩形的面积，cx、cy:矩形的中心坐标，rotation：矩形的倾斜角
    if (line):
        rho = line["cx"] - 120
        if (line["rotation"]>90):
            theta = line["rotation"] - 180  #偏转角
        else:
            theta = line["rotation"]
        img.draw_line(line["rect"][0], line["rect"][1], line["rect"][2],
                line["rect"][3], color=(255, 255, 255), thickness=1)
        img.draw_line(line["rect"][2], line["rect"][3], line["rect"][4],
                line["rect"][5], color=(255, 255, 255), thickness=1)
        img.draw_line(line["rect"][4], line["rect"][5], line["rect"][6],
                line["rect"][7], color=(255, 255, 255), thickness=1)
        img.draw_line(line["rect"][6], line["rect"][7], line["rect"][0],
                line["rect"][1], color=(255, 255, 255), thickness=1)
        img.draw_circle(line["cx"], line["cy"], 4,
                  color=(255, 255, 255), thickness=1)
    display.show(img)
    send_data_packet(rho, theta)
    time.sleep(0.1)