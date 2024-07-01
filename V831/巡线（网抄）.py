from maix import camera, display, gpio, pwm

class FindLine():

    def __init__(self):
        self.THRESHOLD = (4, 53, -99, 87, -72, 70)  # 黑色
        self.roi = [(i*48, j*48, 48, 48) for i in range(5) for j in range(5)]
        self.round = 0  # 定义走了几圈
        # 左轮
        self.left_round_1 = gpio.gpio(0, "H", 1)
        self.left_round_2 = gpio.gpio(1, "H", 1)
        self.left_pwm = pwm.PWM(6)
        # 右轮
        self.right_round_1 = gpio.gpio(2, "H", 1)
        self.right_round_2 = gpio.gpio(3, "H", 1)
        self.right_pwm = pwm.PWM(7)
        # PWM数据初始化
        self.forward = 10000000
        self.fast = 7000000
        self.slow = 13000000
        # 激活
        self.left_pwm.export()
        self.right_pwm.export()
        self.left_pwm.period = 20000000
        self.right_pwm.period = 20000000
        self.left_pwm.duty_cycle = self.forward
        self.right_pwm.duty_cycle = self.forward
        self.left_pwm.enable = True
        self.right_pwm.enable = True

    def forward_move(self):
        self.left_pwm.duty_cycle = self.forward
        self.right_pwm.duty_cycle = self.forward

    def left(self):
        self.left_pwm.duty_cycle = self.slow
        self.right_pwm.duty_cycle = self.fast
    def right(self):
        self.left_pwm.duty_cycle = self.fast
        self.right_pwm.duty_cycle = self.slow

    # 获取黑线中心
    def get_black_center(self, img, roi):
        most_pixels = 0
        max_x = 0
        if img:
            blob0 = img.find_blobs([self.THRESHOLD], roi=roi, pixels_threshold=100, area_threshold=100, merge=True)
            if blob0:
                for n in range(len(blob0)):
                    if blob0[n]["pixels"] > most_pixels:
                        most_pixels = blob0[n]["pixels"]
                        max_x = n
                return blob0[max_x]["x"] + int(blob0[max_x]["w"] / 2), blob0[max_x]["y"] + int(blob0[max_x]["h"] / 2)
            else:
                return 0
        else:
            return 0

    # 连线函数（可用可不用，看性能）
    def connect_line(self, img, dot_list):
        x_orl = 0
        y_orl = 0
        c = 0
        for i in dot_list:
            if i == 0:
                continue
            else:
                if c != 0:
                    img.draw_line(x_orl, y_orl, i[0],
                                  i[1], color=(255, 0, 0), thickness=1)
                c = 1
                x_orl = i[0]
                y_orl = i[1]
        return img

    # 扫描停止标志
    def stop_sign(self, img):
        flag = 0
        roi_list = [(40, 190, 60, 50), (140, 190, 60, 50)]
        max_num_list = []
        for roi in roi_list:
            max_num = img.get_statistics(roi=roi)[0]   # 获取图像中的平均数
            max_num_list.append(max_num)
        for i in max_num_list:
            if i > 50:
                flag = 1
        return flag

    # 识别进分叉
    def bifurcate_identify(self, img):
        flag = 0
        roi = (48, 30, 48, 48)
        max_num = img.get_statistics(roi=roi)[0]  # 获取图像中的平均数
        if max_num < 50:
            flag = 1
        return flag

    # 向左拐进内圈
    def turn_left(self, img):
        dot_list = []
        roi = [(0, i * 48, 120, 48) for i in range(5)]  # 左边大半块
        for i in range(5):
            roii = roi[i]
            dot_list.append(self.get_black_center(img, roii))
        for j in dot_list:
            if j == 0:
                continue
            else:
                img.draw_circle(j[0], j[1], 4, color=(255, 255, 255), thickness=1)
        if dot_list == [0, 0, 0, 0, 0]:
            count = 0
            count_num = 0
            for i in dot_list:
                if i == 0:
                    continue
                count_num += 1
                count += i[0]
            if count_num != 0:
                x_middle = int(count / count_num)
            else:
                x_middle = 60
        else:
            x_middle = 60
        img = self.connect_line(img, dot_list)
        display.show(img)
        return x_middle

    # 前进函数
    def move_forward(self, img):
        dot_list = []
        roi = [(90, i * 48, 60, 48) for i in range(5)]  # 中间
        for i in range(5):
            roii = roi[i]
            dot_list.append(self.get_black_center(img, roii))
        for j in dot_list:
            if j == 0:
                continue
            else:
                img.draw_circle(j[0], j[1], 4, color=(255, 255, 255), thickness=1)
        if dot_list == [0, 0, 0, 0, 0]:
            count = 0
            count_num = 0
            for i in dot_list:
                if i == 0:
                    continue
                count_num += 1
                count += i[0]
            if count_num != 0:
                x_middle = int(count / count_num)
            else:
                x_middle = 120
        else:
            x_middle = 120
        img = self.connect_line(img, dot_list)
        display.show(img)
        return x_middle

    # 检测右边还有没有线（判断是否进入内圈）
    def detect_whether_into_inner(self, img):
        yes_inner = 0
        roi = [(150, i * 48, 90, 48) for i in range(5)]  # 左边大半块
        for i in range(5):
            roii = roi[i]
            flag = self.get_black_center(img, roii)
            if flag:
                yes_inner = 1
                break
        return yes_inner

    def run(self):
        while 1:
            img = camera.capture()
            # 逻辑函数自己写喽，想怎么处理自己决定


if __name__ == '__main__':
    findline = FindLine()
    findline.run()
