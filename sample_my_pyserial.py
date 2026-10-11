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
def tosend_uint16_data(nums):
    """ 将一个 16位无符号 数据打包成二进制字节流并发送给单片机
        数据: 0~65535(FFFF)
        将传进来的数据拆成 高八位,低八位分开的两个uint8
    """
    copy_nums = []
    for num in nums:
        #提取出高八位
        copy_nums.append = (num >> 8) & 0xFF,    #0xFF:1111 1111
        #提取出低八位
        copy_nums.append = num & 0xFF,
    #强制类型转换,将列表转换为bytearray类型,bytearray的元素必须是字节
    data_packet = bytearray(copy_nums)
    return data_packet
    

#大端传输 16位有符号整数(不用这个，用struct)
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
            #1.自己封装的打包函数
            #发送数据
            data = -255
            ser.write(tosend_uint16_data(data))
            #控制发送频率（避免频繁发送挤爆单片机串口缓冲区）
            time.sleep(0.05)  # 20Hz 发送频率


            #2.用struct
            # 准备要发送的数据（包含正负数、浮点数）
            frame_header = 0xAA  # 1字节
            cmd_id = 1  # 1字节
            target_x = -120  # 有符号数 int16 (-32768 ~ 32767)
            target_y = 240  # 有符号数 int16
            kp = 1.5  # 浮点数 float (4字节)
            frame_tail = 0x55  # 1字节

            # 使用 struct.pack 打包
            # 格式串 '> B B h h f B' 依次代表：
            # 大端序 | 1字节无符号(B) | 1字节无符号(B) | 2字节有符号(h) | 2字节有符号(h) | 4字节浮点(f) | 1字节无符号(B)
            packet = struct.pack('>BBhhfB', frame_header, cmd_id, target_x, target_y, kp, frame_tail)

            # 此时 packet 的类型是 bytes，可以直接通过串口发送
            ser.write(packet)
            print(f'成功打包并发送了 {len(packet)} 个字节的数据！')

    except KeyboardInterrupt as e:
        print("Ctrl+C")

    
    #释放与关闭串口资源
    if ser and ser.isOpen():
        ser.close()
        print('串口已安全关闭')
