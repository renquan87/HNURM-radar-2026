"""
raycast_calibrator.py — 射线求交 PnP 外参标定工具

通过在相机图像上标注与 PLY 赛场模型标定点对应的像素坐标，
使用 cv2.solvePnP 求解 world-to-camera 外参 (R, T)，
将结果写入 configs/raycast_calib.yaml。

标定点来源:
    field/keypoint_6.txt 中的 6 个 Solidworks 3D 坐标
    (参考港科大 RM2025-Radar-Algorithm)

操作流程:
    1. 运行脚本，显示相机图像或测试图像
    2. 按照终端提示，依次点击 6 个标定点在图像中的像素位置
    3. 右键撤销上一个点，按 'r' 重新开始
    4. 6 个点标注完成后点击 START CALC 计算 PnP
    5. 查看面板重投影误差，点击 SAVE RESULT 保存到 raycast_calib.yaml

入口:
    # 推荐：直接运行，自动使用默认 keypoints 与图像回退链路
    ros2 run hnurm_radar raycast_calibrator

    # 可选：手动指定图像
    ros2 run hnurm_radar raycast_calibrator -- --image path/to/image.jpg

    # 可选：切换为区域赛 keypoints（默认是 field/keypoint_6.txt）
    ros2 run hnurm_radar raycast_calibrator -- --keypoints field/keypoint_6_region.txt
"""

import argparse
import os
import sys
import itertools
import numpy as np
import cv2
import yaml

from ..shared.paths import (
    MAIN_CONFIG_PATH,
    RAYCAST_CALIB_PATH,
    KEYPOINT_6_PATH,
    TEST_RESOURCES_DIR,
    resolve_path,
)

EXAMPLE_IMAGE_PATH = resolve_path(os.path.join(TEST_RESOURCES_DIR, 'raycast_example.png'))


def load_keypoints(path: str) -> np.ndarray:
    """加载 Solidworks 3D 标定点坐标。"""
    points = []
    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) >= 3:
                points.append([float(parts[0]), float(parts[1]), float(parts[2])])
    return np.array(points, dtype=np.float64)


def load_calib_config(path: str) -> dict:
    """加载现有 raycast_calib.yaml 配置。"""
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def check_mesh_keypoints_rule(mesh_path: str, kp_path: str):
    """检查 mesh 与 keypoints 的场景匹配规则。"""
    mesh_l = str(mesh_path).lower()
    kp_name = os.path.basename(kp_path).lower()

    # 规则：National -> keypoint_6.txt；Regional -> keypoint_6_region.txt
    if 'national' in mesh_l and kp_name != 'keypoint_6.txt':
        print("[WARN] 当前 mesh 为 National，建议使用 field/keypoint_6.txt（不带 region）。")
    if 'regional' in mesh_l and kp_name != 'keypoint_6_region.txt':
        print("[WARN] 当前 mesh 为 Regional，建议使用 field/keypoint_6_region.txt。")


def save_calib_config(path: str, config: dict, R: np.ndarray, T: np.ndarray, residual: float):
    """将 PnP 结果写入 raycast_calib.yaml。"""
    config['R'] = R.tolist()
    config['T'] = T.reshape(3, 1).tolist()

    with open(path, 'w', encoding='utf-8') as f:
        yaml.dump(config, f, default_flow_style=None, allow_unicode=True, sort_keys=False)
        # 将误差注释放到文件末尾最后一行
        f.write(f"# PnP 投影残差: {residual:.4f} px\n")

    print(f"\n[OK] 标定结果已保存到 {path}")


def _solve_single_pnp(object_points, image_points, camera_matrix, dist_coeffs,
                      refine_lm=True, use_ransac=False, ransac_thresh=6.0):
    """对固定对应关系求解一次 PnP。"""
    object_points = object_points.astype(np.float64)
    image_points = image_points.astype(np.float64)

    inlier_indices = None
    if use_ransac:
        success, rvec, tvec, inliers = cv2.solvePnPRansac(
            objectPoints=object_points,
            imagePoints=image_points,
            cameraMatrix=camera_matrix,
            distCoeffs=dist_coeffs,
            iterationsCount=200,
            reprojectionError=float(ransac_thresh),
            confidence=0.999,
            flags=cv2.SOLVEPNP_EPNP,
        )
        if not success:
            return None

        if inliers is not None and len(inliers) >= 4:
            inlier_indices = inliers.flatten().astype(int)
            obj_inliers = object_points[inlier_indices]
            img_inliers = image_points[inlier_indices]
            success, rvec, tvec = cv2.solvePnP(
                objectPoints=obj_inliers,
                imagePoints=img_inliers,
                cameraMatrix=camera_matrix,
                distCoeffs=dist_coeffs,
                rvec=rvec,
                tvec=tvec,
                useExtrinsicGuess=True,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
        else:
            success = False
    else:
        success, rvec, tvec = cv2.solvePnP(
            objectPoints=object_points,
            imagePoints=image_points,
            cameraMatrix=camera_matrix,
            distCoeffs=dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )

    if not success:
        return None

    if refine_lm:
        refine_obj = object_points
        refine_img = image_points
        if inlier_indices is not None and len(inlier_indices) >= 4:
            refine_obj = object_points[inlier_indices]
            refine_img = image_points[inlier_indices]

        rvec, tvec = cv2.solvePnPRefineLM(
            objectPoints=refine_obj,
            imagePoints=refine_img,
            cameraMatrix=camera_matrix,
            distCoeffs=dist_coeffs,
            rvec=rvec,
            tvec=tvec,
        )

    R, _ = cv2.Rodrigues(rvec)
    projected, _ = cv2.projectPoints(
        object_points, rvec, tvec, camera_matrix, dist_coeffs)
    projected = projected.reshape(-1, 2)
    residuals = np.linalg.norm(projected - image_points, axis=1)
    mean_residual = float(np.mean(residuals))

    mean_inlier_residual = None
    if inlier_indices is not None and len(inlier_indices) > 0:
        mean_inlier_residual = float(np.mean(residuals[inlier_indices]))

    return {
        'R': R,
        'T': tvec,
        'rvec': rvec,
        'mean_residual': mean_residual,
        'mean_inlier_residual': mean_inlier_residual,
        'residuals': residuals,
        'projected': projected,
        'inlier_indices': [] if inlier_indices is None else inlier_indices.tolist(),
    }


def solve_pnp_silent(object_points, image_points, camera_matrix, dist_coeffs,
                     match_mode='ordered', refine_lm=True,
                     use_ransac=False, ransac_thresh=6.0):
    """求解 PnP 并返回结构化结果（不在终端打印误差）。

    参数:
        match_mode:
            - ordered: 按输入顺序一一对应
            - auto_permute: 小点数时自动搜索最佳 3D 点序
        refine_lm:
            - True: 使用 solvePnPRefineLM
            - False: 仅 solvePnP ITERATIVE（便于对齐外部实现）
    """
    object_points = object_points.astype(np.float64)
    image_points = image_points.astype(np.float64)

    n_obj = len(object_points)
    n_img = len(image_points)
    if n_obj != n_img:
        return None

    if match_mode == 'ordered':
        result = _solve_single_pnp(
            object_points,
            image_points,
            camera_matrix,
            dist_coeffs,
            refine_lm=refine_lm,
            use_ransac=use_ransac,
            ransac_thresh=ransac_thresh,
        )
        if result is None:
            return None
        result['score_residual'] = (
            result['mean_inlier_residual']
            if result['mean_inlier_residual'] is not None
            else result['mean_residual']
        )
        result['matched_indices'] = list(range(n_obj))
        return result

    # auto_permute：点数较小时全排列搜索最佳点序
    if match_mode == 'auto_permute' and n_obj <= 8:
        best_result = None
        best_perm = None
        for perm in itertools.permutations(range(n_obj), n_img):
            object_points_perm = object_points[list(perm)]
            result = _solve_single_pnp(
                object_points_perm,
                image_points,
                camera_matrix,
                dist_coeffs,
                refine_lm=refine_lm,
                use_ransac=use_ransac,
                ransac_thresh=ransac_thresh,
            )
            if result is None:
                continue
            result_score = (
                result['mean_inlier_residual']
                if result['mean_inlier_residual'] is not None
                else result['mean_residual']
            )
            best_score = None if best_result is None else best_result.get('score_residual')
            if best_result is None or result_score < best_score:
                best_result = result
                best_result['score_residual'] = result_score
                best_perm = list(perm)

        if best_result is None:
            return None
        best_result['matched_indices'] = best_perm
        return best_result

    # 点数太大时回退 ordered
    result = _solve_single_pnp(
        object_points,
        image_points,
        camera_matrix,
        dist_coeffs,
        refine_lm=refine_lm,
        use_ransac=use_ransac,
        ransac_thresh=ransac_thresh,
    )
    if result is None:
        return None
    result['score_residual'] = (
        result['mean_inlier_residual']
        if result['mean_inlier_residual'] is not None
        else result['mean_residual']
    )
    result['matched_indices'] = list(range(n_obj))
    return result


class PnPCalibratorGUI:
    """基于 OpenCV 交互的 PnP 标定工具。"""

    def __init__(self, image: np.ndarray, object_points: np.ndarray,
                 camera_matrix: np.ndarray, dist_coeffs: np.ndarray,
                 match_mode: str = 'ordered', refine_lm: bool = True,
                 use_ransac: bool = False, ransac_thresh: float = 6.0):
        self.image = image.copy()
        self.display = image.copy()
        self.object_points = object_points
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.image_points = []
        self.num_points = len(object_points)
        self.done = False
        self.window_name = "Raycast PnP Calibrator"
        self.scale = 1.0
        self.match_mode = match_mode
        self.refine_lm = refine_lm
        self.use_ransac = use_ransac
        self.ransac_thresh = float(ransac_thresh)
        self.nudge_step = 1
        self.start_clicked = False  # 仅在点击 START CALC 前允许方向键微调

        # PnP 结果状态
        self.pnp_result = None
        self.action = None  # save / reset / None

        # 面板按钮区域: x1, y1, x2, y2
        self.btn_start = (20, 60, 220, 100)      # START CALC
        self.btn_save = (20, 110, 220, 150)      # SAVE RESULT
        self.btn_reset = (20, 160, 220, 200)     # RESET CALIB
        self.btn_example = (240, 60, 430, 100)   # SHOW EXAMPLE

        # 标定点颜色
        self.colors = [
            (0, 0, 255),    # 红
            (0, 165, 255),  # 橙
            (0, 255, 255),  # 黄
            (0, 255, 0),    # 绿
            (255, 0, 0),    # 蓝
            (255, 0, 255),  # 紫
        ]

    def _move_last_point(self, dx: int, dy: int):
        """用方向键微调最后一个标定点。"""
        if len(self.image_points) == 0:
            return

        h, w = self.image.shape[:2]
        x, y = self.image_points[-1]
        nx = int(np.clip(x + dx, 0, w - 1))
        ny = int(np.clip(y + dy, 0, h - 1))
        self.image_points[-1] = [nx, ny]

        # 点位变化后，旧结果失效
        self.done = False
        self.pnp_result = None
        self.action = None
        self._redraw()

    def _get_points_for_pnp(self):
        pts = np.array(self.image_points, dtype=np.float64)
        if self.scale != 1.0:
            pts /= self.scale
        return pts

    def _solve_current_points(self):
        if len(self.image_points) != self.num_points:
            print(f"  [提示] 点数不足：{len(self.image_points)}/{self.num_points}，无法计算。")
            return
        pts = self._get_points_for_pnp()
        self.pnp_result = solve_pnp_silent(
            self.object_points,
            pts,
            self.camera_matrix,
            self.dist_coeffs,
            match_mode=self.match_mode,
            refine_lm=self.refine_lm,
            use_ransac=self.use_ransac,
            ransac_thresh=self.ransac_thresh,
        )
        if self.pnp_result is None:
            print("  [FAIL] PnP 求解失败，请检查点序后重试。")
            self.done = False
        else:
            self.done = True
            match_idx = self.pnp_result.get('matched_indices', [])
            mean_res = self.pnp_result.get('mean_residual', float('nan'))
            mean_inlier = self.pnp_result.get('mean_inlier_residual')
            inliers = self.pnp_result.get('inlier_indices', [])
            print(f"  [OK] 计算完成。匹配点序: {match_idx}")
            if mean_inlier is not None:
                print(f"  [Diag] 残差(all/inlier): {mean_res:.3f}px / {mean_inlier:.3f}px, inliers={inliers}")
            else:
                print(f"  [Diag] 残差(all): {mean_res:.3f}px")

            residuals = np.array(self.pnp_result.get('residuals', []), dtype=np.float64)
            if residuals.size > 0:
                topk = min(3, residuals.size)
                worst_idx = np.argsort(-residuals)[:topk]
                worst_str = ", ".join([f"P{i+1}:{residuals[i]:.2f}px" for i in worst_idx])
                print(f"  [Diag] 最大单点误差 Top{topk}: {worst_str}")

    def _show_example_image(self):
        img = cv2.imread(EXAMPLE_IMAGE_PATH)
        if img is None:
            print(f"  [ERROR] 示例图像不存在: {EXAMPLE_IMAGE_PATH}")
            return
        cv2.imshow("Example Image", img)

    def _mouse_callback(self, event, x, y, flags, param):
        if event not in (cv2.EVENT_LBUTTONDOWN, cv2.EVENT_RBUTTONDOWN):
            return

        # 右键撤销：在标定中与结果阶段都可用
        if event == cv2.EVENT_RBUTTONDOWN:
            if len(self.image_points) > 0:
                removed = self.image_points.pop()
                print(f"  [撤销] 移除 ({removed[0]}, {removed[1]})")
                self.start_clicked = False

                # 撤销后如果点数不足，退出结果态；若仍满点则重新求解
                if len(self.image_points) < self.num_points:
                    self.done = False
                    self.pnp_result = None
                else:
                    pts = np.array(self.image_points, dtype=np.float64)
                    if self.scale != 1.0:
                        pts /= self.scale
                    self.pnp_result = solve_pnp_silent(
                        self.object_points,
                        pts,
                        self.camera_matrix,
                        self.dist_coeffs,
                        match_mode=self.match_mode,
                        refine_lm=self.refine_lm,
                        use_ransac=self.use_ransac,
                        ransac_thresh=self.ransac_thresh,
                    )
                    self.done = self.pnp_result is not None

                self.action = None
                self._redraw()
            return

        # 左键：按钮区优先
        if event == cv2.EVENT_LBUTTONDOWN:
            if self._in_rect(x, y, self.btn_example):
                self._show_example_image()
                return

            if self._in_rect(x, y, self.btn_reset):
                self.image_points.clear()
                self.done = False
                self.pnp_result = None
                self.action = 'reset'
                self.start_clicked = False
                self._redraw()
                print("  [重置] 清除所有标注点。请点击第 1 个标定点...")
                return

            if self._in_rect(x, y, self.btn_start):
                self.start_clicked = True
                self._solve_current_points()
                self._redraw()
                return

            if self._in_rect(x, y, self.btn_save):
                if self.pnp_result is not None:
                    self.action = 'save'
                else:
                    print("  [提示] 尚未计算结果，请先点击 START CALC。")
                return

        if event == cv2.EVENT_LBUTTONDOWN:
            if len(self.image_points) < self.num_points:
                self.image_points.append([x, y])
                self.done = False
                self.pnp_result = None
                self.start_clicked = False
                self._redraw()

                remaining = self.num_points - len(self.image_points)
                if remaining > 0:
                    idx = len(self.image_points)
                    print(f"  [{idx}/{self.num_points}] 已标注 ({x}, {y})"
                          f" -> 3D: ({self.object_points[idx-1][0]:.3f}, "
                          f"{self.object_points[idx-1][1]:.3f}, "
                          f"{self.object_points[idx-1][2]:.3f})")
                    print(f"  请点击第 {idx+1} 个标定点...")
                else:
                    print(f"  [{self.num_points}/{self.num_points}] 已标注 ({x}, {y})"
                          f" -> 3D: ({self.object_points[-1][0]:.3f}, "
                          f"{self.object_points[-1][1]:.3f}, "
                          f"{self.object_points[-1][2]:.3f})")
                    print("  [提示] 点已标满，请点击 START CALC 开始计算。")

    @staticmethod
    def _in_rect(x, y, rect):
        x1, y1, x2, y2 = rect
        return x1 <= x <= x2 and y1 <= y <= y2

    def _draw_panel(self):
        cv2.rectangle(self.display, (10, 10), (480, 240), (35, 35, 40), -1)
        cv2.rectangle(self.display, (10, 10), (480, 240), (100, 100, 110), 2)

        def draw_button(rect, color, text, text_scale=0.68):
            x1, y1, x2, y2 = rect
            cv2.rectangle(self.display, (x1, y1), (x2, y2), color, -1)
            cv2.rectangle(self.display, (x1, y1), (x2, y2), (200, 200, 200), 1)
            (tw, th), baseline = cv2.getTextSize(
                text, cv2.FONT_HERSHEY_SIMPLEX, text_scale, 2)
            tx = x1 + max(6, (x2 - x1 - tw) // 2)
            ty = y1 + (y2 - y1 + th) // 2 - baseline
            cv2.putText(self.display, text, (tx, ty),
                        cv2.FONT_HERSHEY_SIMPLEX, text_scale, (255, 255, 255), 2)

        cv2.putText(
            self.display,
            f"Points: {len(self.image_points)}/{self.num_points}",
            (20, 38),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (220, 220, 220), 2
        )

        if self.pnp_result is None:
            cv2.putText(
                self.display, "Reprojection Error: N/A",
                (20, 226), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (150, 150, 150), 1
            )
        else:
            mean_res = self.pnp_result['mean_residual']
            cv2.putText(
                self.display, f"Reprojection Error: {mean_res:.3f}px",
                (20, 226), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (80, 220, 80), 1
            )

        start_ok = (len(self.image_points) == self.num_points)
        draw_button(
            self.btn_start,
            (76, 175, 80) if start_ok else (80, 80, 85),
            "START CALC",
            text_scale=0.68,
        )

        save_ok = (self.pnp_result is not None)
        draw_button(
            self.btn_save,
            (210, 130, 30) if save_ok else (80, 80, 85),
            "SAVE RESULT",
            text_scale=0.68,
        )

        draw_button(self.btn_reset, (54, 67, 230), "RESET CALIB", text_scale=0.68)

        draw_button(self.btn_example, (160, 100, 50), "SHOW EXAMPLE", text_scale=0.60)

        cv2.putText(self.display, "LClick=Pick  RClick=Undo  R=Reset  Q=Quit",
                    (230, 226), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)

    def _redraw(self):
        self.display = self.image.copy()
        for i, pt in enumerate(self.image_points):
            color = self.colors[i % len(self.colors)]
            cv2.circle(self.display, (pt[0], pt[1]), 6, color, -1)
            cv2.circle(self.display, (pt[0], pt[1]), 8, (255, 255, 255), 2)
            label = f"P{i+1}"
            cv2.putText(self.display, label, (pt[0] + 10, pt[1] - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)

        self._draw_panel()
        cv2.imshow(self.window_name, self.display)

    def run(self):
        """运行交互式标注。返回 Nx2 的图像点数组。"""
        # 缩放显示 (大分辨率图像)
        h, w = self.image.shape[:2]
        scale = 1.0
        if w > 1920:
            scale = 1920.0 / w
            self.image = cv2.resize(self.image, None, fx=scale, fy=scale)
            self.display = self.image.copy()
        self.scale = scale

        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(self.window_name, self._mouse_callback)

        print(f"\n=== PnP 标定 ({self.num_points} 个标定点) ===")
        print(f"  3D 标定点坐标 (Solidworks):")
        for i, pt in enumerate(self.object_points):
            print(f"    P{i+1}: ({pt[0]:.3f}, {pt[1]:.3f}, {pt[2]:.3f})")
        print(f"\n  请点击第 1 个标定点...")

        self._redraw()

        # 常见平台的方向键 keycode（OpenCV waitKeyEx）
        LEFT_KEYS = {81, 2424832, 65361}
        UP_KEYS = {82, 2490368, 65362}
        RIGHT_KEYS = {83, 2555904, 65363}
        DOWN_KEYS = {84, 2621440, 65364}

        while True:
            key = cv2.waitKeyEx(50)
            if key == ord('q') or key == ord('Q') or key == 27:
                print("  [取消] 用户退出标定。")
                cv2.destroyAllWindows()
                return None, None
            elif key == ord('r') or key == ord('R'):
                self.image_points.clear()
                self.done = False
                self.pnp_result = None
                self.action = 'reset'
                self.start_clicked = False
                self._redraw()
                print("  [重置] 清除所有标注点。请点击第 1 个标定点...")
            elif key in LEFT_KEYS:
                if not self.start_clicked:
                    self._move_last_point(-self.nudge_step, 0)
            elif key in RIGHT_KEYS:
                if not self.start_clicked:
                    self._move_last_point(self.nudge_step, 0)
            elif key in UP_KEYS:
                if not self.start_clicked:
                    self._move_last_point(0, -self.nudge_step)
            elif key in DOWN_KEYS:
                if not self.start_clicked:
                    self._move_last_point(0, self.nudge_step)

            if self.action == 'save' and self.pnp_result is not None:
                cv2.destroyAllWindows()
                pts = np.array(self.image_points, dtype=np.float64)
                if scale != 1.0:
                    pts /= scale
                return pts, self.pnp_result
        cv2.destroyAllWindows()
        return None, None


def solve_pnp(object_points, image_points, camera_matrix, dist_coeffs):
    """兼容旧接口：内部复用 silent 版本，不在终端输出误差。"""
    result = solve_pnp_silent(
        object_points,
        image_points,
        camera_matrix,
        dist_coeffs,
        match_mode='ordered',
        refine_lm=True,
        use_ransac=False,
        ransac_thresh=6.0,
    )
    if result is None:
        return None, None, None
    return result['R'], result['T'], result['mean_residual']


def main():
    parser = argparse.ArgumentParser(description="射线求交 PnP 外参标定工具")
    parser.add_argument('--image', type=str, default=None,
                        help='待标定的相机图像路径（可选，不传则自动回退到配置图像）')
    parser.add_argument('--keypoints', type=str, default=None,
                        help='3D 标定点文件路径（默认: field/keypoint_6.txt）')
    parser.add_argument('--config', type=str, default=None,
                        help='raycast_calib.yaml 路径 (默认: configs/raycast_calib.yaml)')
    parser.add_argument('--video', type=str, default=None,
                        help='从视频中截取第一帧作为标定图像（可选）')
    parser.add_argument('--match-mode', type=str, default='auto_permute',
                        choices=['ordered', 'auto_permute'],
                        help='PnP 点匹配模式: ordered(严格按点序) / auto_permute(自动搜索最优点序)')
    parser.add_argument('--no-refine-lm', action='store_true',
                        help='关闭 solvePnPRefineLM，仅使用 solvePnP ITERATIVE（用于对齐外部实现）')
    parser.add_argument('--use-ransac', action='store_true',
                        help='启用 solvePnPRansac 抗错点求解，并输出内点残差诊断')
    parser.add_argument('--ransac-thresh', type=float, default=6.0,
                        help='RANSAC 重投影阈值（像素），默认 6.0')
    parser.add_argument('--example', action='store_true',
                        help='使用内置示例图像 test_resources/raycast_example.png')
    args = parser.parse_args()

    # 标定配置文件
    config_path = args.config or RAYCAST_CALIB_PATH
    if not os.path.exists(config_path):
        print(f"[ERROR] 配置文件不存在: {config_path}")
        sys.exit(1)
    config = load_calib_config(config_path)

    # 相机内参
    camera_matrix = np.array(config['K'], dtype=np.float64)
    dist_coeffs = np.array(config.get('dist_coeffs', [0, 0, 0, 0, 0]), dtype=np.float64)

    # 3D 标定点
    kp_path = args.keypoints or KEYPOINT_6_PATH
    if not os.path.exists(kp_path):
        print(f"[ERROR] 标定点文件不存在: {kp_path}")
        print(f"  请确认 field/keypoint_6.txt 存在（从 HKUST 项目复制）")
        sys.exit(1)
    object_points = load_keypoints(kp_path)
    print(f"加载 {len(object_points)} 个 3D 标定点: {kp_path}")
    check_mesh_keypoints_rule(config.get('mesh_path', ''), kp_path)

    print("  标定点坐标 (Solidworks):")
    for i, pt in enumerate(object_points):
        print(f"    P{i+1}: ({pt[0]:+.3f}, {pt[1]:+.3f}, {pt[2]:+.3f})")

    # 图像
    image = None
    if args.image:
        image = cv2.imread(args.image)
        if image is None:
            print(f"[ERROR] 无法加载图像: {args.image}")
            sys.exit(1)
    elif args.video:
        cap = cv2.VideoCapture(args.video)
        ret, image = cap.read()
        cap.release()
        if not ret:
            print(f"[ERROR] 无法从视频中读取帧: {args.video}")
            sys.exit(1)
    elif args.example:
        image = cv2.imread(EXAMPLE_IMAGE_PATH)
        if image is None:
            print(f"[ERROR] 示例图像不存在: {EXAMPLE_IMAGE_PATH}")
            sys.exit(1)
        print(f"使用示例图像: {EXAMPLE_IMAGE_PATH}")
    else:
        # 从 main_config.yaml 读取视频源
        if os.path.exists(MAIN_CONFIG_PATH):
            with open(MAIN_CONFIG_PATH, 'r', encoding='utf-8') as f:
                main_cfg = yaml.safe_load(f)
            camera_cfg = main_cfg.get('camera', {})
            source = camera_cfg.get('video_source', 0)
            if isinstance(source, str) and os.path.exists(source):
                cap = cv2.VideoCapture(source)
                ret, image = cap.read()
                cap.release()
                if ret:
                    print(f"从视频源截取第一帧: {source}")
            if image is None:
                test_img = camera_cfg.get('test_image', 'test_resources/red1.png')
                test_path = resolve_path(test_img)
                image = cv2.imread(test_path)
                if image is not None:
                    print(f"使用测试图像: {test_path}")
        if image is None and os.path.exists(EXAMPLE_IMAGE_PATH):
            image = cv2.imread(EXAMPLE_IMAGE_PATH)
            if image is not None:
                print(f"使用示例图像: {EXAMPLE_IMAGE_PATH}")

    if image is None:
        print("[ERROR] 无法获取图像。已尝试 --image/--video/--example、main_config.yaml 的 camera.test_image 与内置示例图。")
        sys.exit(1)

    print(f"图像尺寸: {image.shape[1]}x{image.shape[0]}")

    # 交互式标注
    gui = PnPCalibratorGUI(
        image,
        object_points,
        camera_matrix,
        dist_coeffs,
        match_mode=args.match_mode,
        refine_lm=(not args.no_refine_lm),
        use_ransac=args.use_ransac,
        ransac_thresh=args.ransac_thresh,
    )
    image_points, pnp_result = gui.run()

    if image_points is None or pnp_result is None:
        print("标定已取消。")
        sys.exit(0)

    save_calib_config(
        config_path,
        config,
        pnp_result['R'],
        pnp_result['T'],
        pnp_result['mean_residual'],
    )


if __name__ == '__main__':
    main()
