"""
perspective_calibrator.py — 透视变换交互式标定工具（GUI 版）

参考 PFA 标定方案，使用 PyQt5 实现：
  - 左侧显示相机/测试图像，点击选取像素特征点
  - 右侧显示赛场地图，点击选取对应赛场位置
  - 支持地面层 / 高地层分别标定
  - 标定结果保存为 perspective_calib.json，供 camera_detector 直接加载

操作流程：
  1. 点击"开始标定"冻结画面
  2. 在左图点 4+ 个地面特征点，右图依次点击对应位置
  3. （可选）点击"切换高度"，再标定高地层 4+ 个点
  4. 点击"保存计算"写入标定文件

入口：
  ros2 run hnurm_radar perspective_calibrator
  或直接：python -m hnurm_radar.camera_locator.perspective_calibrator
"""

import threading
import time
import sys
import os
import json

# ── 解决 PyQt5 与 OpenCV 内置 Qt 插件冲突 ──
# OpenCV (cv2) 自带了 Qt 平台插件路径，会覆盖系统 PyQt5 的插件，
# 导致 "Could not load the Qt platform plugin xcb" 错误。
# 必须在 import cv2 前后都清除该环境变量。
os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)

import cv2

# cv2 import 时会重新设置 QT_QPA_PLATFORM_PLUGIN_PATH，再清一次
os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)

import numpy as np
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPixmap, QImage, QTextCursor
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton,
    QVBoxLayout, QHBoxLayout, QTextEdit, QGridLayout,
)
from ruamel.yaml import YAML

# ======================== 配置路径 ========================
from ..shared.paths import (
    MAIN_CONFIG_PATH as _MAIN_CONFIG_PATH,
    PERSPECTIVE_CALIB_PATH as _PERSPECTIVE_CALIB_PATH,
    PFA_MAP_2025_PATH, TEST_RESOURCES_DIR,
    FIELD_WIDTH, FIELD_HEIGHT,
)
MAIN_CONFIG_PATH = _MAIN_CONFIG_PATH
PERSPECTIVE_CALIB_PATH = _PERSPECTIVE_CALIB_PATH
MAP_IMAGE_PATH = PFA_MAP_2025_PATH

# 地图物理尺寸（从场景配置自动获取）
FIELD_W = FIELD_WIDTH   # m
FIELD_H = FIELD_HEIGHT  # m


# ======================== 相机图像采集线程 ========================
camera_image = None  # 全局相机图像（BGR）


def hik_camera_thread():
    """海康工业相机采集线程"""
    global camera_image
    try:
        # 尝试导入项目自带的 HKCam
        sys.path.insert(0, os.path.join(
            os.path.dirname(__file__), '..'))
        from Camera.HKCam import HKCam
        cam = HKCam(0)
        print("[标定工具] 海康工业相机已连接")
        while True:
            frame = cam.getFrame()
            if frame is not None:
                camera_image = frame
    except Exception as e:
        print(f"[标定工具] 海康相机初始化失败: {e}")


def video_capture_thread(source=0):
    """USB 摄像头 / 视频文件采集线程"""
    global camera_image
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        print(f"[标定工具] 无法打开视频源: {source}")
        return
    print(f"[标定工具] 视频源已打开: {source}")
    while True:
        ret, img = cap.read()
        if ret:
            camera_image = img
        time.sleep(0.016)  # ~60fps


# ======================== 高度层颜色 ========================
# 不同高度层使用不同颜色标记，方便区分
LAYER_COLORS = [
    (255, 255, 255),  # 地面层 — 白色
    (0, 255, 0),      # 高地层 — 绿色
]
LAYER_NAMES = ["地面层", "高地层"]


# ======================== 标定 UI ========================
class CalibrationUI(QWidget):
    def __init__(self, my_color: str):
        super().__init__()
        self.my_color = my_color
        self.capturing = True  # True=实时更新相机画面, False=冻结（标定中）

        # 标定数据：每层各一组 (image_points, map_points)
        # layer 0 = 地面层, layer 1 = 高地层
        self.image_points = [[], []]  # [[list of (px,py)], [list of (px,py)]]
        self.map_points = [[], []]    # [[list of (mx,my) in field meters], ...]
        self.current_layer = 0        # 当前标定层
        self.image_count = 0          # 当前层的图像点计数
        self.map_count = 0            # 当前层的地图点计数

        self.H_results = [None, None]  # 计算结果

        self._init_ui()

    def _init_ui(self):
        # ---- 左侧：相机图像 ----
        self.left_label = QLabel(self)
        self.left_label.setFixedSize(1280, 960)
        self.left_label.setStyleSheet("border: 2px solid black;")
        self.left_label.mousePressEvent = self._left_clicked

        # ---- 右侧：赛场地图 ----
        self.right_label = QLabel(self)
        self.right_label.setFixedSize(560, 300)
        self.right_label.setStyleSheet("border: 2px solid black;")
        self.right_label.mousePressEvent = self._right_clicked

        # ---- 信息文本框 ----
        self.text_edit = QTextEdit(self)
        self.text_edit.setFixedSize(560, 200)
        self.text_edit.setReadOnly(True)

        # ---- 按钮 ----
        self.btn_start = QPushButton('开始标定', self)
        self.btn_start.setFixedSize(130, 36)
        self.btn_start.clicked.connect(self._on_start)

        self.btn_switch = QPushButton('切换到高地层', self)
        self.btn_switch.setFixedSize(130, 36)
        self.btn_switch.clicked.connect(self._on_switch_layer)

        self.btn_undo = QPushButton('撤销上一点', self)
        self.btn_undo.setFixedSize(130, 36)
        self.btn_undo.clicked.connect(self._on_undo)

        self.btn_save = QPushButton('保存计算', self)
        self.btn_save.setFixedSize(130, 36)
        self.btn_save.clicked.connect(self._on_save)

        # ---- 状态标签 ----
        self.status_label = QLabel(self)
        self.status_label.setFixedSize(560, 30)
        self._update_status()

        # ---- 布局 ----
        btn_grid = QGridLayout()
        btn_grid.addWidget(self.btn_start, 0, 0)
        btn_grid.addWidget(self.btn_switch, 0, 1)
        btn_grid.addWidget(self.btn_undo, 1, 0)
        btn_grid.addWidget(self.btn_save, 1, 1)

        right_vbox = QVBoxLayout()
        right_vbox.addWidget(self.right_label)
        right_vbox.addWidget(self.status_label)
        right_vbox.addLayout(btn_grid)
        right_vbox.addWidget(self.text_edit)

        hbox = QHBoxLayout()
        hbox.addWidget(self.left_label)
        hbox.addLayout(right_vbox)

        self.setLayout(hbox)
        self.setWindowTitle(
            f'透视变换标定 — {self.my_color}方')
        self.setGeometry(50, 50, 1870, 980)

        # ---- 读取地图图像 ----
        self.map_orig = cv2.imread(MAP_IMAGE_PATH)
        if self.map_orig is None:
            self._log("⚠ 无法读取地图: " + MAP_IMAGE_PATH)
            self.map_orig = np.zeros((1500, 2800, 3), dtype=np.uint8)

        self.map_h_orig, self.map_w_orig = self.map_orig.shape[:2]
        # 显示缩放比例
        self.right_scale_x = self.map_w_orig / 560.0
        self.right_scale_y = self.map_h_orig / 300.0

        # ---- 读取相机图像 ----
        self.cam_frame = camera_image
        if self.cam_frame is not None:
            self.cam_h_orig, self.cam_w_orig = self.cam_frame.shape[:2]
        else:
            self.cam_w_orig, self.cam_h_orig = 1920, 1080
        self.left_scale_x = self.cam_w_orig / 1280.0
        self.left_scale_y = self.cam_h_orig / 960.0

        # ---- 初始图像 ----
        self._left_display = None
        self._right_display = None
        self._refresh_images()

        # ---- 定时器：更新相机画面 ----
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._timer_tick)
        self.timer.start(50)

        self._log("=== 透视变换标定工具 ===")
        self._log(f"方阵: {self.my_color}方")
        self._log("步骤: 点击「开始标定」冻结画面 → "
                  "左图点击特征点 → 右图点击对应地图位置 → "
                  "点够后点击「保存计算」")
        self._log(f"地图: {MAP_IMAGE_PATH}")
        self._log(f"赛场尺寸: {FIELD_W}m × {FIELD_H}m")
        self._log("─" * 40)

        self.show()

    # ================================================================
    #  UI 事件
    # ================================================================
    def _on_start(self):
        """冻结画面，进入标定模式"""
        if self.capturing:
            self.capturing = False
            # 冻结当前帧
            if camera_image is not None:
                self.cam_frame = camera_image.copy()
                self.cam_h_orig, self.cam_w_orig = self.cam_frame.shape[:2]
                self.left_scale_x = self.cam_w_orig / 1280.0
                self.left_scale_y = self.cam_h_orig / 960.0
            self.btn_start.setText("已冻结")
            self.btn_start.setEnabled(False)
            self._log("✔ 画面已冻结，开始选点。")
            self._log("  先在左图（相机）上点击一个特征点，再在右图（地图）上点击对应位置。")
            self._log(f"  当前标定层: {LAYER_NAMES[self.current_layer]}")
        self._refresh_images()

    def _on_switch_layer(self):
        """切换高度层"""
        if self.capturing:
            self._log("⚠ 请先点击「开始标定」冻结画面。")
            return
        old_layer = self.current_layer
        self.current_layer = 1 - self.current_layer
        self.image_count = len(self.image_points[self.current_layer])
        self.map_count = len(self.map_points[self.current_layer])
        self.btn_switch.setText(
            f"切换到{LAYER_NAMES[1 - self.current_layer]}")
        self._log(f"─ 已切换到: {LAYER_NAMES[self.current_layer]} "
                  f"(已有 {self.image_count} 个图像点, {self.map_count} 个地图点)")
        self._update_status()
        self._refresh_images()

    def _on_undo(self):
        """撤销当前层的最后一对点"""
        if self.capturing:
            return
        layer = self.current_layer
        # 优先撤销地图点（如果地图点多于图像点）
        if len(self.map_points[layer]) > len(self.image_points[layer]):
            self.map_points[layer].pop()
            self.map_count = len(self.map_points[layer])
            self._log(f"  撤销最后一个地图点 (剩 {self.map_count} 个)")
        elif len(self.image_points[layer]) > 0:
            # 同时撤销一对
            if len(self.map_points[layer]) == len(self.image_points[layer]):
                self.map_points[layer].pop()
                self.map_count = len(self.map_points[layer])
            self.image_points[layer].pop()
            self.image_count = len(self.image_points[layer])
            self._log(f"  撤销最后一对点 (剩 {self.image_count} 对)")
        self._update_status()
        self._refresh_images()

    def _on_save(self):
        """计算 Homography 并保存"""
        if self.capturing:
            self._log("⚠ 请先点击「开始标定」冻结画面并选点。")
            return

        success = False
        calib_data = {
            'calibration_time': time.strftime("%Y-%m-%d %H:%M:%S"),
            'my_color': self.my_color,
        }

        # ---- 地面层 ----
        n_ground = min(len(self.image_points[0]), len(self.map_points[0]))
        if n_ground >= 4:
            img_pts = np.array(self.image_points[0][:n_ground], dtype=np.float64)
            fld_pts = np.array(self.map_points[0][:n_ground], dtype=np.float64)
            H, status = cv2.findHomography(img_pts, fld_pts, cv2.RANSAC, 5.0)
            if H is not None:
                self.H_results[0] = H
                calib_data['H_ground'] = H.tolist()
                calib_data['H'] = H.tolist()  # 向后兼容
                calib_data['pixel_points_ground'] = self.image_points[0][:n_ground]
                calib_data['field_points_ground'] = self.map_points[0][:n_ground]
                inliers = int(np.sum(status))
                self._log(f"✔ 地面层 Homography 计算成功 "
                          f"({n_ground} 点, {inliers} 内点)")
                success = True
            else:
                self._log("✘ 地面层 Homography 计算失败！请确认点对合理。")
        else:
            self._log(f"⚠ 地面层点对不足: 需要 ≥4 对, 当前 {n_ground} 对")

        # ---- 高地层 ----
        n_highland = min(len(self.image_points[1]), len(self.map_points[1]))
        if n_highland >= 4:
            img_pts = np.array(self.image_points[1][:n_highland], dtype=np.float64)
            fld_pts = np.array(self.map_points[1][:n_highland], dtype=np.float64)
            H, status = cv2.findHomography(img_pts, fld_pts, cv2.RANSAC, 5.0)
            if H is not None:
                self.H_results[1] = H
                calib_data['H_highland'] = H.tolist()
                calib_data['pixel_points_highland'] = self.image_points[1][:n_highland]
                calib_data['field_points_highland'] = self.map_points[1][:n_highland]
                inliers = int(np.sum(status))
                self._log(f"✔ 高地层 Homography 计算成功 "
                          f"({n_highland} 点, {inliers} 内点)")
                success = True
            else:
                self._log("✘ 高地层 Homography 计算失败！")
        elif n_highland > 0:
            self._log(f"⚠ 高地层点对不足: 需要 ≥4 对, 当前 {n_highland} 对 (跳过)")
        else:
            self._log("ℹ 未标定高地层（可选）。")

        if not success:
            self._log("✘ 没有任何有效的标定结果，请检查选点。")
            return

        # ---- 保存 ----
        # 也保存旧格式字段（pixel_points / field_points）方便兼容
        if 'H' not in calib_data and self.H_results[0] is not None:
            calib_data['H'] = self.H_results[0].tolist()

        os.makedirs(os.path.dirname(PERSPECTIVE_CALIB_PATH), exist_ok=True)
        with open(PERSPECTIVE_CALIB_PATH, 'w') as f:
            json.dump(calib_data, f, indent=2)
        self._log(f"✔ 标定结果已保存到:\n  {PERSPECTIVE_CALIB_PATH}")
        self._log("─" * 40)
        self._log("可以关闭此窗口，然后运行 camera_detector 使用新标定。")

        # ---- 验证：用标定矩阵反算选点，显示误差 ----
        self._show_reprojection_error()

    # ================================================================
    #  鼠标事件
    # ================================================================
    def _left_clicked(self, event):
        """相机图像点击 — 记录像素坐标"""
        if self.capturing:
            return
        layer = self.current_layer
        # 显示坐标 → 原图坐标
        x_orig = int(event.pos().x() * self.left_scale_x)
        y_orig = int(event.pos().y() * self.left_scale_y)

        self.image_points[layer].append([x_orig, y_orig])
        self.image_count = len(self.image_points[layer])

        idx = self.image_count
        self._log(f"  [{LAYER_NAMES[layer]}] 图像点 P{idx}: "
                  f"像素({x_orig}, {y_orig})")
        self._update_status()
        self._refresh_images()

    def _right_clicked(self, event):
        """地图点击 — 将地图像素坐标转换为赛场米坐标"""
        if self.capturing:
            return
        layer = self.current_layer

        # 显示坐标 → 原图像素坐标
        map_px = int(event.pos().x() * self.right_scale_x)
        map_py = int(event.pos().y() * self.right_scale_y)

        # 地图像素 → 赛场米坐标
        # 约定: 地图 W=2800 对应 28m, H=1500 对应 15m
        # 地图像素原点在左上角，赛场 y=0 在地图底部
        field_x = map_px / (self.map_w_orig / FIELD_W)
        field_y = (self.map_h_orig - map_py) / (self.map_h_orig / FIELD_H)

        field_x = round(field_x, 2)
        field_y = round(field_y, 2)

        self.map_points[layer].append([field_x, field_y])
        self.map_count = len(self.map_points[layer])

        idx = self.map_count
        self._log(f"  [{LAYER_NAMES[layer]}] 地图点 M{idx}: "
                  f"赛场({field_x}, {field_y})m "
                  f"[地图像素({map_px}, {map_py})]")
        self._update_status()
        self._refresh_images()

    # ================================================================
    #  绘制
    # ================================================================
    def _refresh_images(self):
        """重绘左右两张图（含标注点）"""
        # ---- 左图：相机 ----
        if self.cam_frame is not None:
            left_rgb = cv2.cvtColor(self.cam_frame, cv2.COLOR_BGR2RGB)
            left_show = cv2.resize(left_rgb, (1280, 960))
        else:
            left_show = np.zeros((960, 1280, 3), dtype=np.uint8)

        # 画已选图像点（所有层都画）
        for layer_idx in range(2):
            color = LAYER_COLORS[layer_idx]
            for i, (px, py) in enumerate(self.image_points[layer_idx]):
                sx = int(px / self.left_scale_x)
                sy = int(py / self.left_scale_y)
                cv2.circle(left_show, (sx, sy), 6, color, -1)
                cv2.putText(left_show,
                            f"{LAYER_NAMES[layer_idx][0]}{i+1}",
                            (sx + 8, sy - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        self._left_display = left_show
        self.left_label.setPixmap(self._cv_to_pixmap(left_show))

        # ---- 右图：地图 ----
        right_rgb = cv2.cvtColor(self.map_orig.copy(), cv2.COLOR_BGR2RGB)
        right_show = cv2.resize(right_rgb, (560, 300))

        # 画已选地图点
        for layer_idx in range(2):
            color = LAYER_COLORS[layer_idx]
            for i, (fx, fy) in enumerate(self.map_points[layer_idx]):
                # 赛场米 → 地图像素 → 显示像素
                mpx = fx * (self.map_w_orig / FIELD_W)
                mpy = self.map_h_orig - fy * (self.map_h_orig / FIELD_H)
                sx = int(mpx / self.right_scale_x)
                sy = int(mpy / self.right_scale_y)
                cv2.circle(right_show, (sx, sy), 5, color, -1)
                cv2.putText(right_show,
                            f"{LAYER_NAMES[layer_idx][0]}{i+1}",
                            (sx + 5, sy - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        self._right_display = right_show
        self.right_label.setPixmap(self._cv_to_pixmap(right_show))

    def _timer_tick(self):
        """定时器：实时更新相机画面（未冻结时）"""
        if self.capturing and camera_image is not None:
            self.cam_frame = camera_image.copy()
            self.cam_h_orig, self.cam_w_orig = self.cam_frame.shape[:2]
            self.left_scale_x = self.cam_w_orig / 1280.0
            self.left_scale_y = self.cam_h_orig / 960.0
            self._refresh_images()

    # ================================================================
    #  辅助
    # ================================================================
    def _update_status(self):
        layer = self.current_layer
        n_img = len(self.image_points[layer])
        n_map = len(self.map_points[layer])
        n_pairs = min(n_img, n_map)
        need = max(0, 4 - n_pairs)
        total_ground = min(len(self.image_points[0]), len(self.map_points[0]))
        total_highland = min(len(self.image_points[1]), len(self.map_points[1]))

        text = (
            f"当前: {LAYER_NAMES[layer]} | "
            f"图像点: {n_img} | 地图点: {n_map} | "
            f"有效对: {n_pairs}"
        )
        if need > 0:
            text += f" | 还需 {need} 对"
        else:
            text += " | ✔ 可保存"
        text += f"  ‖  地面:{total_ground}对 高地:{total_highland}对"

        self.status_label.setText(text)
        self.status_label.setStyleSheet(
            "font-size: 13px; font-weight: bold; "
            f"color: {'green' if need == 0 else 'red'};")

    def _show_reprojection_error(self):
        """显示重投影误差"""
        for layer_idx, layer_name in enumerate(LAYER_NAMES):
            H = self.H_results[layer_idx]
            if H is None:
                continue
            n = min(len(self.image_points[layer_idx]),
                    len(self.map_points[layer_idx]))
            if n < 4:
                continue
            img_pts = np.array(self.image_points[layer_idx][:n],
                               dtype=np.float64)
            fld_pts = np.array(self.map_points[layer_idx][:n],
                               dtype=np.float64)
            # 用 H 变换像素 → 赛场
            pts_in = img_pts.reshape(-1, 1, 2)
            pts_out = cv2.perspectiveTransform(pts_in, H)
            pts_out = pts_out.reshape(-1, 2)

            errors = np.linalg.norm(pts_out - fld_pts, axis=1)
            self._log(f"  [{layer_name}] 重投影误差: "
                      f"均值={np.mean(errors):.3f}m, "
                      f"最大={np.max(errors):.3f}m, "
                      f"各点={[f'{e:.3f}' for e in errors]}")

    def _log(self, text: str):
        """追加日志到文本框"""
        self.text_edit.append(text)
        cursor = self.text_edit.textCursor()
        cursor.movePosition(QTextCursor.End)
        self.text_edit.setTextCursor(cursor)

    @staticmethod
    def _cv_to_pixmap(cv_rgb):
        """OpenCV RGB → QPixmap"""
        h, w, ch = cv_rgb.shape
        qimg = QImage(cv_rgb.data, w, h, ch * w, QImage.Format_RGB888)
        return QPixmap.fromImage(qimg)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Escape:
            self.close()


# ======================== 入口 ========================
def main():
    # 加载配置
    yaml = YAML()
    with open(MAIN_CONFIG_PATH, encoding='utf-8') as f:
        cfg = yaml.load(f)

    my_color = cfg['global'].get('my_color', 'Blue')
    camera_cfg = cfg.get('camera', {})
    camera_mode = camera_cfg.get('mode', 'test')
    video_source = camera_cfg.get('video_source', 0)
    test_img_path = camera_cfg.get(
        'test_image',
        os.path.join(TEST_RESOURCES_DIR, 'test1.jpg'))

    global camera_image

    if camera_mode == 'test':
        camera_image = cv2.imread(test_img_path)
        if camera_image is None:
            print(f"[标定工具] 无法加载测试图片: {test_img_path}")
            sys.exit(1)
        print(f"[标定工具] 测试模式: {test_img_path} "
              f"({camera_image.shape[1]}×{camera_image.shape[0]})")
    elif camera_mode == 'hik':
        thread = threading.Thread(target=hik_camera_thread, daemon=True)
        thread.start()
    elif camera_mode == 'video':
        thread = threading.Thread(
            target=video_capture_thread, args=(video_source,), daemon=True)
        thread.start()

    # 等待相机图像就绪
    if camera_mode != 'test':
        print("[标定工具] 等待相机图像 ...")
        for _ in range(100):
            if camera_image is not None:
                break
            time.sleep(0.1)
        if camera_image is None:
            print("[标定工具] 超时！无法获取相机图像。使用空白图像。")

    # 启动 GUI
    app = QApplication(sys.argv)
    ui = CalibrationUI(my_color)
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
