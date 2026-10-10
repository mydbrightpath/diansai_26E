import time
import serial  # 导入串口通信库 (pyserial)
import struct  # 导入二进制数据打包库
import cv2 as cv

# ==========================================
# 第一步：初始化串口配置与连接
# 这个选择的串口号是我用来发送接收的那个串口号
# ==========================================




# ==========================================
# 第二步：构建数据打包与发送函数
# ==========================================
#大端传输 16位无符号整数
def send_uint16_data(num):
    """ 将一个 16位无符号 数据打包成二进制字节流并发送给单片机
        数据: 0~65535(FFFF)
        将传进来的数据拆成 高八位,低八位分开的两个uint8
    """
    #强制类型转换,将列表转换为bytearray类型,bytearray的元素必须是字节
    data_packet = bytearray(
        [
        #提取出高八位
        (num >> 8) & 0xFF,    #0xFF:1111 1111
        #提取出低八位
        num & 0xFF,
        ]
    )
    # 将打包好的二进制数据写入串口
    ser.write(data_packet)

#大端传输 16位有符号整数
def send_int16_data(num):
    """ 将一个 16位有符号 数据打包成二进制字节流并发送给单片机
        数据: -32768~32767
        将传进来的数据拆成 高八位,低八位分开的两个uint8
    """
    #将Python的有符号整数转换为 16 位补码无符号的 (高字节, 低字节)
    # & 0xFFFF 作用：将负数限制在 16 位补码范围内（如 -120 转换为 0xFF88）
    num_16bit = num & 0xFFFF

    #强制类型转换,将列表转换为bytearray类型,bytearray的元素必须是字节
    data_packet = bytearray(
        [
        #提取出高八位
        (num_16bit >> 8) & 0xFF,    #0xFF:1111 1111
        #提取出低八位
        num_16bit & 0xFF,
        ]
    )
    # 将打包好的二进制数据写入串口
    ser.write(data_packet)


#只有在这个文件里面运行程序时才会被运行的代码
if __name__ == "__main__":
    ser = serial.Serial(port='COM3', baudrate=115200, timeout=1)

    try:
        
        while True:
            #发送数据
            data = -255
            send_uint16_data(data)
            #控制发送频率（避免频繁发送挤爆单片机串口缓冲区）
            time.sleep(0.05)  # 20Hz 发送频率


    except KeyboardInterrupt as e:
        print("Ctrl+C")

    
    #释放与关闭串口资源
    if ser and ser.isOpen():
        ser.close()
        print('串口已安全关闭')