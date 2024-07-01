from maix import image, display, camera, gpio
import struct
import serial, time

ser = serial.Serial("/dev/ttyS1", 115200, timeout=0.2)

set_LAB_red = [(10, 53, 23, 60, 25, 63)]  # red
set_LAB_blue = [(22, 74, -5, 22, -69, -13)]  # blue
set_LAB_green = [(10, 59, -58, -25, -8, 45)]  # green
j = 0


def send_data_packet(x, y, w, h):
    temp = struct.pack("<bbhhhhb",  # 格式为俩个字符俩个整型
                       0x2C,  # 帧头1
                       0x32,  # 帧头2
                       int(x),  # up sample by 4    #数据1
                       int(y),  # up sample by 4    #数据2
                       int(w),
                       int(h),
                       0x5b)
    ser.write(temp)  # 串口发送


now_time = time.time()

string = ' '

while True:
    img = camera.capture()
    blobs = []
    size_max = 0
    size_red_max = 2000
    size_blue_max = 2000
    size_green_max = 2000
    blobs_red = img.find_blobs(set_LAB_red)  # 在图片中查找lab阈值内的颜色色块
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
            size_blue = blob["w"] * blob["h"]
            if size_blue > size_blue_max:
                size_blue_max = size_blue
            blobs.append(blob)
    if blobs_green:
        for blob in blobs_green:
            size_green = blob["w"] * blob["h"]
            if size_green > size_green_max:
                size_green_max = size_green
            blobs.append(blob)
    if size_red_max > size_blue_max and size_red_max > size_green_max:
        j = 1
    elif size_blue_max > size_red_max and size_blue_max > size_green_max:
        j = 2
    elif size_green_max > size_red_max and size_green_max > size_blue_max:
        j = 3

    if blobs:
        for i in blobs:
            size = i["w"] * i["h"]
            if (size > size_max):
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
            img.draw_rectangle(x_start, y_start, x_end, y_end, color,
                               thickness=3)

        if j == 1:
            string = 'red'
            send_data_packet(x_center - 120, 0, w, h)
        elif j == 2:
            string = 'blue'
            send_data_packet(x_center - 120, 1, w, h)
        elif j == 3:
            string = 'green'
            send_data_packet(x_center - 120, 2, w, h)
        str_size = image.get_string_size(string)
        img.draw_string(x_center - int(str_size[0] / 2) - 5, y_start - 35, string, scale=1.5,
                        color=(0xff, 0xff, 0xff), thickness=2)

        now_time = time.time()  # time.asctime()

    display.show(img)