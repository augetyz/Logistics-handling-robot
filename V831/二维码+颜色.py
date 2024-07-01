from maix import image, display, camera, gpio
import struct
import serial, time
ser = serial.Serial("/dev/ttyS1",115200, timeout=0.2) 

set_LAB_red = [(23, 53, 23, 68, 25, 63)]  # red
set_LAB_blue = [(22, 74, -5, 22, -69, -13)] # blue
set_LAB_green =  [(23, 59, -58, -25, -8, 45)] #green

flag = '1'
now_time = time.time()
j = 0

def send_data_packet(x, y, w, h):
    temp = struct.pack("<bbhhhhb",                #格式为俩个字符俩个整型
                   0x2C,                       #帧头1
                   0x12,                       #帧头2
                   int(x), # up sample by 4    #数据1
                   int(y), # up sample by 4    #数据2
                   int(w),
                   int(h),
                   0x5b)
    ser.write(temp)                           #串口发送         

def Erweima_task():
    task1 = ser.read(16)
    while (task1 != b'D'):
        task1 = ser.read(16)
        img = camera.capture()
        mks = img.find_qrcodes()
        for mk in mks:
            
              #外框数据
            X = mk['x']
            Y = mk['y']
            W = mk['w']
            H = mk['h']
      
              #二维码信息
            string = mk['payload']
            tep = ",!"+"0" +string +"["
            ser.write(tep.encode("utf-8"))
            
              #内框数据
            x1,y1 = mk['corners'][0]   #访问字典的列表
            x2,y2 = mk['corners'][1]
            x3,y3 = mk['corners'][2]
            x4,y4 = mk['corners'][3]
      
              #画外框
            img.draw_rectangle(X, Y, X + W, Y + H, color=(0, 0, 255), thickness = 2) 
              #打印信息
            #img.draw_string(int(X) , int(Y - 35) , str(string), scale = 2.0, color = (255, 0, 0), thickness = 2)  #内框ID
              #画内框
            img.draw_line(x1, y1, x2, y2, color = (0, 255, 0), thickness = 3)  
            img.draw_line(x2, y2, x3, y3, color = (0, 255, 0), thickness = 3)  
            img.draw_line(x3, y3, x4, y4, color = (0, 255, 0), thickness = 3)  
            img.draw_line(x4, y4, x1, y1, color = (0, 255, 0), thickness = 3)  
            
        display.show(img)
        
def Yanse_task():
    task2 = ser.read(16)
    while (task2 != b'D'):
        task2 = ser.read(16)
        img = camera.capture()
        blobs = []
        size_max = 0
        size_red_max = 2000
        size_blue_max = 2000
        size_green_max = 2000
        blobs_red = img.find_blobs(set_LAB_red)# 在图片中查找lab阈值内的颜色色块
        blobs_blue = img.find_blobs(set_LAB_blue)
        blobs_green = img.find_blobs(set_LAB_green)
        if blobs_red:
            for blob in blobs_red:
                size_red = blob["w"] * blob["h"]
                if size_red > size_red_max:
                    size_red_max = size_red
                blobs.append(blob)
        if blobs_blue:
            for blob in blobs_blue:
                size_blue= blob["w"] * blob["h"]
                if size_blue > size_blue_max:
                    size_blue_max = size_blue
                blobs.append(blob)
        if blobs_green:
            for blob in blobs_green:
                size_green= blob["w"] * blob["h"]
                if size_green > size_green_max:
                    size_green_max = size_green
                blobs.append(blob)
        if size_red_max>size_blue_max and size_red_max>size_green_max:
            j = 1
        elif size_blue_max>size_red_max and size_blue_max>size_green_max:
            j = 2 
        elif size_green_max>size_red_max and size_green_max>size_blue_max:
            j = 3

        if blobs: 
            for i in blobs:
                size= i["w"] * i["h"]
                if (size>size_max):
                    size_max = size
                    blob_max = i
            w = blob_max["w"]
            h = blob_max["h"]
            x_start = blob_max["x"]
            x_end = blob_max["x"] + blob_max["w"]
            x_center = int((x_start + x_end) / 2)  # 中心坐标
            y_start = blob_max["y"]
            y_end = blob_max["y"] + blob_max["h"]
            y_center = int((y_start + y_end) / 2)
            m = max((x_center - i["w"] * 0.3), 0)
            n = max((y_center - i["h"] * 0.3), 0)
            m = min((x_center - i["w"] * 0.3), 240)
            n = min((y_center - i["h"] * 0.3), 240)
            mk = [int(m), int(n), 20, 20]

            if (mk[0] + 20) < 220 and (mk[1] + 20) < 220:
                git_color = img.get_blob_color(mk, 0, 0)
                img.draw_rectangle(9, 9, 21, 21, color=(255, 255, 255), thickness=1)  # 左上角颜色区域画出来
                color = (int(git_color[0]), int(git_color[1]), int(git_color[2]))
                img.draw_rectangle(10, 10, 20, 20, color, thickness=-1)  # 将颜色填充到左上角
                img.draw_rectangle(x_start, y_start, x_end,y_end ,color,
                                thickness=3)

            if j == 1:
                string = 'red'
                send_data_packet( x_center - 120,0, w, h)
            elif j == 2:
                string = 'blue'
                send_data_packet( x_center - 120,1, w, h)  
            elif j == 3:
                string = 'green' 
                send_data_packet( x_center - 120, 2, w, h)     
            str_size = image.get_string_size(string)
            img.draw_string(x_center - int(str_size[0] / 2) - 5, y_start - 35, string, scale=1.5,
                    color=(int(git_color[0]), int(git_color[1]), int(git_color[2])), thickness=2)

            now_time = time.time()  # time.asctime()

        display.show(img)
while True:
    task  =  ser.read(16)
    if (task == b'A'):
        Erweima_task()
    elif (task == b'B'):     
        Yanse_task()