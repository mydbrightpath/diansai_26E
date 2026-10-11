"""基于接缝两侧颜色连续性的图案评分基线，仅用于给几何候选排序。
白底或重复图案会产生歧义，不能单独证明扑克牌牌面正确。
视觉端提供 sampler(piece_id, original_points_mm) -> N×3 RGB，范围 0~1。
sampler 应使用校正后的俯视图和毫米到图像变换，并采用双线性采样。
"""
import numpy as np
from puzzle_solver import cross, rotation


def make_pattern_scorer(sampler, sample_spacing_mm=0.8, inset_mm=0.8):
    """比较公共边内侧的颜色。inset 应大于切割缝/轮廓误差，且小于细节尺度。
    相邻真实图案本来可能在接缝处剧变，故这是启发式基线，需用实拍数据验证。
    更可靠的扩展是两侧多行梯度外推，或与完整牌面模板进行整体配准。
    """
    def score(pieces, poses):
        differences = []
        for i in range(len(pieces)):
            a = poses[i].apply(pieces[i].vertices_mm)
            for j in range(i+1, len(pieces)):
                b = poses[j].apply(pieces[j].vertices_mm)
                for k in range(len(a)):
                    u, v = a[k], a[(k+1) % len(a)]
                    length = np.linalg.norm(v-u)
                    axis = (v-u)/length
                    normal = np.array([-axis[1], axis[0]])  # 逆时针片内部位于边的左侧
                    for m in range(len(b)):
                        x, y = b[m], b[(m+1) % len(b)]
                        if np.dot(y-x, axis) >= 0:
                            continue
                        if max(abs(cross(axis, x-u)), abs(cross(axis, y-u))) > 1e-5:
                            continue
                        lo, hi = sorted((np.dot(x-u, axis), np.dot(y-u, axis)))
                        lo, hi = max(0, lo)+inset_mm, min(length, hi)-inset_mm
                        if hi <= lo:
                            continue
                        t = np.linspace(lo, hi, max(2, int((hi-lo)/sample_spacing_mm)+1))
                        seam = u+t[:, None]*axis
                        # 接缝两边分别往各自内部移动，再用刚体变换的逆变换回原图。
                        qa = (seam+inset_mm*normal-poses[i].translation) @ rotation(poses[i].theta)
                        qb = (seam-inset_mm*normal-poses[j].translation) @ rotation(poses[j].theta)
                        ca, cb = np.asarray(sampler(pieces[i].id, qa)), np.asarray(sampler(pieces[j].id, qb))
                        if ca.shape != (len(t), 3) or cb.shape != ca.shape:
                            raise ValueError('sampler 必须返回 N×3 RGB')
                        if not np.isfinite(ca).all() or not np.isfinite(cb).all():
                            raise ValueError('图案采样越界或含无效值')
                        differences.extend(np.abs(ca-cb).mean(axis=1).tolist())
        return -float(np.mean(differences)) if differences else 0.0
    return score
