import cv2
from threading import Thread
import uuid
import os
import time
count = 0
def image_collect(cap):
    global count
    while True:
        success, img = cap.read()
        if success:
            file_name = str(uuid.uuid4())+'.jpg'
            cv2.imwrite(os.path.join('images',file_name),img)
            count = count+1
            print("save %d %s"%(count,file_name))
        time.sleep(0.4)

if __name__ == "__main__":
    
    os.makedirs("images",exist_ok=True)
    
    # 打开摄像头
    cap = cv2.VideoCapture(0)

    m_thread = Thread(target=image_collect, args=([cap]),daemon=True)
    

    while True:

        # 读取一帧图像

        success, img = cap.read()

        if not success:

            continue

        cv2.imshow("video",img)

        key =  cv2.waitKey(1) & 0xFF   

        # 按键 "q" 退出
        if key ==  ord('c'):
            m_thread.start()
            continue
        elif key ==  ord('q'):
            break

    cap.release() 

       

    

    

    

    

    

    

    

    

    

    

    

    

    
