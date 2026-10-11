import cv2 as cv
import numpy as np


#计算边长
def get_side_lengths(pts):
    """
    计算边长：
    传入顶点列表 pts (形状为 N*2,是二维列表)，返回所有边长的列表
    """
    num_pts = len(pts)
    side_lengths = []

    for i in range(num_pts):
        # 当前点 p1，下一个点 p2（最后一个点与第0个点相连形成闭合）
        p1 = pts[i]
        p2 = pts[(i + 1) % num_pts]

        # 计算两点间距离
        length = np.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)
        side_lengths.append(round(length, 2))  # 保留两位小数

    return side_lengths

#计算夹角
def calculate_angle(A, B, C):
    """计算以 B 为顶点的夹角 Angle(ABC)，单位为度(°)"""
    BA = np.array(A) - np.array(B)
    BC = np.array(C) - np.array(B)

    # 计算点积与向量模长
    dot_product = np.dot(BA, BC)
    norm_BA = np.linalg.norm(BA)
    norm_BC = np.linalg.norm(BC)

    # 防止浮点数误差导致 cosine 超出 [-1, 1] 范围
    cosine_angle = dot_product / (norm_BA * norm_BC + 1e-7)
    cosine_angle = np.clip(cosine_angle, -1.0, 1.0)

    # 弧度转角度
    angle = np.arccos(cosine_angle)
    return np.degrees(angle)

#计算多边形的每一个内角
def get_all_angles(pts):
    """遍历多边形所有顶点，计算出每一个顶点的内角"""
    num_pts = len(pts)
    angles = []

    for i in range(num_pts):
        # A 为上一个点，B 为当前顶点，C 为下一个点
        A = pts[(i - 1) % num_pts]
        B = pts[i]
        C = pts[(i + 1) % num_pts]

        angle = calculate_angle(A, B, C)
        angles.append(round(angle, 1))

    return angles