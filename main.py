import cv2 as cv
import numpy as np

# 打开 USB 摄像头
# 参数 0 通常代表电脑默认摄像头或插入的第一个 USB 摄像头
# 如果打不开，可以尝试改成 1 或 2
cap = cv.VideoCapture(1)

# 检查摄像头是否成功打开
if not cap.isOpened():
    print(' Error: 无法打开 USB 摄像头！' \
    '请检查 USB 连接或修改设备索引号 (如改用 cap = cv.VideoCapture(1))')
    exit()

# 可选：设置摄像头的分辨率（例如设置为 640x480，保证处理流畅度）
cap.set(cv.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 1080)

while True:
    # 逐帧读取图像
    # ret: bool 类型，True 表示成功读取到一帧图片；frame: 当前帧的彩色图像矩阵 (BGR)
    ret, frame = cap.read()



    cv.imshow("img",frame)
    i = cv.waitKey(1)
    if i == ord('q'):
        break

