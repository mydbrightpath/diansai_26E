import cv2 as cv
import numpy as np
import serial  # 导入串口通信库 (pyserial)
import struct  # 导入二进制数据打包库
import os



# ==========================================
# 第一步：外设配置
# ==========================================

#在linux终端上输入这行命令:echo $DISPLAY 输出的所有结果赋值过来,例如:'localhost:11.0'
#这样就可以把图像直接显示出来
os.environ['DISPLAY'] = 'localhost:11.0'


window_name = 'img'
# 创建窗口，并指定为 WINDOW_NORMAL（允许自由调整大小/放大）
cv.namedWindow(window_name, cv.WINDOW_NORMAL)
#（可选）直接通过代码设定窗口的初始放大尺寸，比如宽 800、高 600
cv.resizeWindow(window_name, 800, 600)


# 打开 USB 摄像头
# 参数 0 通常代表电脑默认摄像头或插入的第一个 USB 摄像头
# 如果打不开，可以尝试改成 1 或 2
# cap = cv.VideoCapture('/dev/video0',cv.CAP_V4L2)
cap = cv.VideoCapture(0)


# 检查摄像头是否成功打开
if not cap.isOpened():
    print(' Error: 无法打开 USB 摄像头！' \
    '请检查 USB 连接或修改设备索引号 (如改用 cap = cv.VideoCapture(1))')
    exit()

# 可选：设置摄像头的分辨率（例如设置为 640x480，1920x1080保证处理流畅度）
cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)



# ==========================================
# 第二步：赋值
# ==========================================
pieces = [] #储存每一个碎片的字典的列表








while True:
    # 逐帧读取图像
    # ret: bool 类型，True 表示成功读取到一帧图片；frame: 当前帧的彩色图像矩阵 (BGR)
    ret, frame = cap.read()

    #--------------图像处理部分------------#
    #将图像转为灰度图
    gray = cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
    #高斯模糊和阈值分割和二值化
    blur = cv.GaussianBlur(gray,(5,5),0)
    ret,th = cv.threshold(blur,0,255,cv.THRESH_BINARY+cv.THRESH_OTSU)
    #提取轮廓
    contours,hierarchy = cv.findContours(th,cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)

    #--------------获取发送给主控信息部分------------#
    #要先标定，将像素顶点经过坐标变换转成毫米坐标
    #1.碎片编号 piece_0
    #2.碎片顶点 vertices_mm
    #3.机械臂抓取位置 pick_mm
    
    #(1)多边形近似，提取每个碎片对应顶点    得到piece_0,vertices_mm
    for index,cnt in enumerate(contours):   #遍历元素的同时获得其索引
        #轮廓面积过滤
        if cv.contourArea(cnt) < 100:
                    continue
        #计算单个轮廓cnt的周长(弧长)
        peri = cv.arcLength(cnt,True)   #第二个参数Ture表示轮廓是"闭合"的
        #定义动态计算精度epsilon(这里取周长的10%作为允许误差)
        epsilon = 0.1 * peri
        #执行多边形近似，提取到这个轮廓的多边形的顶点
        approx = cv.approxPolyDP(cnt,epsilon,True)  #第二个参数Ture表示轮廓是"闭合"的
        #将输出结果，即顶点的数组 从三维数组变成二维数组
        vertices_px = approx.reshape(-1,2)

        #！！！ 标定后，要将像素坐标 vertices_px转换为毫米坐标 vertices_mm

        #将此轮廓的信息添加到字典里，并将此字典放在列表中
        pieces.append(
             {
                  'id':index,
                  'vertices_px':vertices_px,    #！！！ 转换后此处坐标变为毫米坐标
             }
        )



    #最小外接矩形(旋转矩形)：传入单个轮廓的轮廓点集，计算旋转矩形信息
    for index,cnt in enumerate(contours):   
        #轮廓面积过滤
        if cv.contourArea(cnt) < 100:
            continue
        #计算当前这一个轮廓的旋转矩形的信息
        rect = cv.minAreaRect(cnt) #元组((cx,cy) 中心点坐标, (w,h)宽与高 , angle旋转角度)
        #将信息打包成字典，传入列表，列表元素为每一个碎片的信息字典
        pieces.append(
            {
                'id' : index,
                'contour' : cnt,
                'center_point' : rect[0],
                'angle' : rect[0],
            }
        )
        #测试:画出其边界
        cv.drawContours(frame,[cnt],0,(0,0,255),2)



    
    #图像显示与退出部分
    cv.imshow(window_name,frame)
    i = cv.waitKey(1)
    if i == ord('q'):
        break










