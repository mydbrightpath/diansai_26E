"""小规模直边拼图求解器：毫米坐标、只旋转平移、不翻面。

依赖：Python >= 3.10、numpy。运行 python puzzle_solver.py 执行合成案例。
这是一份算法参考实现，不直接驱动机械臂；输出的是抓取/放置几何任务。
"""
from __future__ import annotations
from dataclasses import dataclass
import json
import math
import time
from typing import Callable
import numpy as np


def cross(a, b):
    return float(a[0] * b[1] - a[1] * b[0])


def signed_area(p):
    return sum(cross(p[i], p[(i + 1) % len(p)]) for i in range(len(p))) / 2


def rotation(theta):
    """二维主动旋转矩阵；theta 为弧度，正角在数学坐标系中逆时针。"""
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


def wrap(theta):
    return (theta + math.pi) % (2 * math.pi) - math.pi


def centroid(p):
    """面积质心，用于演示。实际吸盘位置应由视觉检查安全内接距离后提供。"""
    q = np.roll(p, -1, axis=0)
    weights = p[:, 0] * q[:, 1] - q[:, 0] * p[:, 1]
    return ((p + q) * weights[:, None]).sum(axis=0) / (3 * weights.sum())


def inside_triangle(x, a, b, c):
    return min(cross(b-a, x-a), cross(c-b, x-b), cross(a-c, x-c)) >= -1e-9


def triangulate(p):
    """耳切法分解简单多边形，支持凹多边形。输入必须逆时针且无共线冗余点。
    分解后，两个多边形交叠面积等于各三角形两两交叠面积之和。
    """
    ids, triangles = list(range(len(p))), []
    while len(ids) > 3:
        for k in range(len(ids)):
            i, j, m = ids[k-1], ids[k], ids[(k+1) % len(ids)]
            if cross(p[j]-p[i], p[m]-p[j]) <= 1e-9:
                continue
            if any(inside_triangle(p[t], p[i], p[j], p[m])
                   for t in ids if t not in (i, j, m)):
                continue
            triangles.append(p[[i, j, m]])
            ids.pop(k)
            break
        else:
            raise ValueError('轮廓自交、退化或含共线冗余顶点；请在视觉端清理')
    triangles.append(p[ids])
    return triangles


def convex_intersection_area(a, b):
    """Sutherland-Hodgman 裁剪；b 是逆时针凸多边形。接触边面积为零。"""
    out = list(a)
    for i in range(len(b)):
        u, v = b[i], b[(i+1) % len(b)]
        inp, out = out, []
        if not inp:
            return 0.0
        for j in range(len(inp)):
            s, e = inp[j-1], inp[j]
            ds, de = cross(v-u, s-u), cross(v-u, e-u)
            if (ds >= -1e-9) != (de >= -1e-9):
                out.append(s + (e-s) * ds / (ds-de))
            if de >= -1e-9:
                out.append(e)
    return abs(signed_area(np.array(out))) if len(out) >= 3 else 0.0


@dataclass
class Piece:
    id: str
    vertices_mm: np.ndarray
    pick_mm: np.ndarray
    source_yaw_rad: float = 0.0

    def __post_init__(self):
        self.vertices_mm = np.asarray(self.vertices_mm, dtype=float)
        self.pick_mm = np.asarray(self.pick_mm, dtype=float)
        p = self.vertices_mm
        if p.ndim != 2 or p.shape[1] != 2 or not 3 <= len(p) <= 5:
            raise ValueError('每片应提供 3~5 个拐角；不要传完整像素轮廓')
        if not np.isfinite(p).all() or self.pick_mm.shape != (2,) or not np.isfinite(self.pick_mm).all():
            raise ValueError('输入必须是有限数值')
        if not math.isfinite(self.source_yaw_rad):
            raise ValueError('source_yaw_rad 必须有限')
        if signed_area(p) < 0:
            self.vertices_mm = p[::-1].copy()
        if abs(signed_area(p)) < 1e-6:
            raise ValueError('碎片面积为零')
        self.triangles = triangulate(self.vertices_mm)
        if not any(inside_triangle(self.pick_mm, *t) for t in self.triangles):
            raise ValueError('抓取点不在碎片内部')


@dataclass
class Pose:
    """从当前实测毫米坐标到拼图坐标：q = R(theta) @ p + translation。"""
    theta: float
    translation: np.ndarray

    def apply(self, p):
        return np.asarray(p) @ rotation(self.theta).T + self.translation


@dataclass
class Config:
    # 现场矩形范围，长边 90~120 mm，短边 50~90 mm；自备题可自行覆盖。
    long_range: tuple = (90.0, 120.0)
    short_range: tuple = (50.0, 90.0)
    area_gap_fraction: float = 0.015
    overlap_mm2: float = 0.5
    dimension_tolerance_mm: float = 1.0
    min_contact_mm: float = 3.0
    max_seconds: float = 12.0
    max_nodes: int = 100000
    max_solutions: int = 32


def overlap(piece_a, pose_a, piece_b, pose_b):
    a, b = pose_a.apply(piece_a.vertices_mm), pose_b.apply(piece_b.vertices_mm)
    if np.any(np.minimum(a.max(axis=0), b.max(axis=0))-
              np.maximum(a.min(axis=0), b.min(axis=0)) <= 1e-8):
        return 0.0
    return sum(convex_intersection_area(pose_a.apply(a), pose_b.apply(b))
               for a in piece_a.triangles for b in piece_b.triangles)


def contact_length(a, b):
    """计算两个逆时针轮廓的共线反向边接触长度；允许长边被多片分段。
    这里仅检测严格边对齐的候选，不用“边长相同”作为必要条件。
    """
    total = 0.0
    for i in range(len(a)):
        u, v = a[i], a[(i+1) % len(a)]
        d = v-u
        length = np.linalg.norm(d)
        if length < 1e-8:
            continue
        axis = d / length
        for j in range(len(b)):
            x, y = b[j], b[(j+1) % len(b)]
            if np.dot(y-x, axis) >= 0:
                continue
            if abs(cross(axis, x-u)) > 1e-6 or abs(cross(axis, y-u)) > 1e-6:
                continue
            t0, t1 = sorted((np.dot(x-u, axis), np.dot(y-u, axis)))
            total += max(0.0, min(length, t1)-max(0.0, t0))
    return total


def candidates(piece, placed_piece, placed_pose):
    """把新片每条边旋转到与已放片某条边反向，再枚举两端点对齐。
    反向是因为两片逆时针轮廓在公共边上的行走方向相反。
    不允许镜像变换，避免产生需要翻面的虚假解。
    """
    p, q = piece.vertices_mm, placed_pose.apply(placed_piece.vertices_mm)
    for i in range(len(p)):
        a, b = p[i], p[(i+1) % len(p)]
        for j in range(len(q)):
            c, d = q[j], q[(j+1) % len(q)]
            theta = wrap(math.atan2(*(c-d)[::-1]) - math.atan2(*(b-a)[::-1]))
            r = rotation(theta)
            for src in (a, b):
                for dst in (c, d):
                    yield Pose(theta, dst-r @ src)


def solve(pieces: list[Piece], config=None, pattern_score: Callable | None = None):
    """返回候选、搜索状态；pattern_score(pieces, normalized_poses) 越大越好。

    算法是有限端点对齐搜索，并非对任意连续切割的完备证明。
    噪声较大的实测轮廓应先拟合/校正；无解时禁止自动执行机械臂。
    达到时间/节点/候选上限时保留已找到的解，并明确返回 search_complete=False。
    """
    cfg = config or Config()
    if not 1 <= len(pieces) <= 4 or len({p.id for p in pieces}) != len(pieces):
        raise ValueError('需要 1~4 片且 id 唯一')
    start = time.monotonic()
    nodes, stopped = 0, False
    solutions, solution_keys, state_keys = [], set(), set()
    total_area = sum(abs(signed_area(p.vertices_mm)) for p in pieces)

    def visit(poses):
        nonlocal nodes, stopped
        if stopped:
            return
        if (time.monotonic()-start >= cfg.max_seconds or nodes >= cfg.max_nodes
                or len(solutions) >= cfg.max_solutions):
            stopped = True
            return
        nodes += 1
        points = np.vstack([pose.apply(pieces[i].vertices_mm) for i, pose in poses.items()])
        low, high = points.min(axis=0), points.max(axis=0)
        w, h = high-low
        # 拼装的包围盒只能增长；超过允许最大尺寸可立即剪枝。
        tol = cfg.dimension_tolerance_mm
        if max(w, h) > cfg.long_range[1]+tol or min(w, h) > cfg.short_range[1]+tol:
            return
        if w*h > total_area/(1-cfg.area_gap_fraction)+1e-6:
            return
        # 同一部分拼装可由不同放片顺序到达，归一化平移后缓存避免重复搜索。
        state_key = tuple((i, *np.round(p.theta % (2*math.pi), 6).reshape(1),
                           *np.round(p.translation-low, 4)) for i,p in sorted(poses.items()))
        if state_key in state_keys:
            return
        state_keys.add(state_key)
        if len(poses) == len(pieces):
            long, short = max(w, h), min(w, h)
            if not (cfg.long_range[0]-tol <= long <= cfg.long_range[1]+tol
                    and cfg.short_range[0]-tol <= short <= cfg.short_range[1]+tol):
                return
            # 不重叠且总面积接近包围盒面积，才能视为矩形；不能只检查边匹配。
            rectangle_area = w*h
            gap = max(0.0, rectangle_area-total_area)/rectangle_area
            if gap > cfg.area_gap_fraction:
                return
            norm = {i: Pose(p.theta, p.translation-low) for i, p in poses.items()}
            key = tuple(tuple(np.round(norm[i].apply(pieces[i].vertices_mm).ravel(), 2))
                        for i in range(len(pieces)))
            if key in solution_keys:
                return
            solution_keys.add(key)
            score = float(pattern_score(pieces, norm)) if pattern_score else 0.0
            if not math.isfinite(score):
                raise ValueError('图案评分必须是有限数值')
            solutions.append({'poses': norm, 'width_mm': float(w), 'height_mm': float(h),
                              'gap_fraction': float(gap), 'pattern_score': score})
            return
        # 枚举下一片及与任一已放片的接触；不能固定为 id 顺序，否则会漏掉邻接链。
        for i in range(len(pieces)):
            if i in poses:
                continue
            seen = set()
            for j, anchor in list(poses.items()):
                for pose in candidates(pieces[i], pieces[j], anchor):
                    key = tuple(np.round([pose.theta, *pose.translation], 5))
                    if key in seen:
                        continue
                    seen.add(key)
                    poly = pose.apply(pieces[i].vertices_mm)
                    if contact_length(poly, anchor.apply(pieces[j].vertices_mm)) < cfg.min_contact_mm:
                        continue
                    if any(overlap(pieces[i], pose, pieces[k], pk) > cfg.overlap_mm2
                           for k, pk in poses.items()):
                        continue
                    visit({**poses, i: pose})
                    if stopped:
                        return

    # 每片至少有一条外边，所以枚举首片每条边作为水平外边，消除全局自由旋转。
    # 只固定第一片会忽略某些外边假设，故首片也枚举；去重在完整解阶段完成。
    for i, piece in enumerate(pieces):
        for e in range(len(piece.vertices_mm)):
            a, b = piece.vertices_mm[e], piece.vertices_mm[(e+1) % len(piece.vertices_mm)]
            theta = -math.atan2((b-a)[1], (b-a)[0])
            visit({i: Pose(theta, -rotation(theta) @ a)})
            if stopped:
                break
        if stopped:
            break
    solutions.sort(key=lambda s: (-s['pattern_score'], s['gap_fraction']))
    return {'solutions': solutions, 'search_complete': not stopped, 'nodes': nodes,
            'elapsed_s': time.monotonic()-start,
            'status': 'candidate_found' if solutions else ('limit_reached' if stopped else 'no_solution')}


def make_tasks(pieces, solution, target_origin_mm, target_yaw_rad=0.0,
               paper_bounds=(0.0, 148.5, 210.0, 297.0)):
    """把归一化拼图放到 A4 下半区，生成 JSON 可序列化抓放任务。

    纸面坐标规定原点在左上、x 向右、y 向下。因此正角在纸面图上为顺时针。
    target_origin_mm 是旋转前矩形左上角的落点。完整检查每片顶点在允许区域内。
    控制端必须另外做工作空间、Z 高度、工具偏置、关节路径、碰撞和闭环检查。
    """
    origin = np.asarray(target_origin_mm, dtype=float)
    r = rotation(target_yaw_rad)
    xmin, ymin, xmax, ymax = paper_bounds
    tasks = []
    for i, piece in enumerate(pieces):
        pose = solution['poses'][i]
        destination_poly = pose.apply(piece.vertices_mm) @ r.T + origin
        if (destination_poly[:, 0].min() < xmin or destination_poly[:, 0].max() > xmax
                or destination_poly[:, 1].min() < ymin or destination_poly[:, 1].max() > ymax):
            raise ValueError('目标碎片越出允许拼图区，请调整目标原点/朝向')
        theta = wrap(pose.theta+target_yaw_rad)
        translation = r @ pose.translation+origin
        pick_dst = rotation(theta) @ piece.pick_mm+translation
        tasks.append({'piece_id': piece.id, 'pick_xy_mm': piece.pick_mm.tolist(),
                      'place_xy_mm': pick_dst.tolist(), 'delta_yaw_deg': math.degrees(theta),
                      'source_yaw_deg': math.degrees(piece.source_yaw_rad),
                      'target_yaw_deg': math.degrees(wrap(piece.source_yaw_rad+theta)),
                      'source_to_target_R': rotation(theta).tolist(),
                      'source_to_target_t_mm': translation.tolist(),
                      'target_vertices_mm': destination_poly.tolist()})
    return {'coordinate_frame': 'paper_xy_mm_x_right_y_down',
            'status': 'geometry_candidate_requires_validation',
            'width_mm': solution['width_mm'], 'height_mm': solution['height_mm'],
            'tasks': tasks}


def demo():
    """4 片合成矩形，包含长边分段、随机姿态。真实识别由视觉模块替换。"""
    polygons = [np.array(p, float) for p in [
        [(0, 0), (40, 0), (40, 30), (0, 30)],
        [(40, 0), (100, 0), (100, 30), (40, 30)],
        [(0, 30), (55, 30), (55, 60), (0, 60)],
        [(55, 30), (100, 30), (100, 60), (55, 60)]]]
    pieces = []
    for i, p in enumerate(polygons):
        theta = [0.31, -0.63, 1.13, -1.4][i]
        # 散放在上半区的四个固定位置，便于重复测试；只用局部外形生成姿态。
        local = p-centroid(p)
        observed = local @ rotation(theta).T + np.array([(45, 40), (140, 40), (45, 110), (140, 110)][i])
        pieces.append(Piece(str(i), observed, centroid(observed), theta))
    result = solve(pieces, Config(max_solutions=12))
    if not result['solutions']:
        raise RuntimeError(result['status'])
    solution = result['solutions'][0]
    # 将矩形居中放在下半区；这里选择朝向 0。
    origin = [(210-solution['width_mm'])/2, 180]
    output = make_tasks(pieces, solution, origin)
    output['search_complete'] = result['search_complete']
    output['candidate_count'] = len(result['solutions'])
    output['nodes'] = result['nodes']
    print(json.dumps(output, ensure_ascii=False, indent=2))
    return pieces, result, output


if __name__ == '__main__':
    demo()
