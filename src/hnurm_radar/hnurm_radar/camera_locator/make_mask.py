"""
make_mask.py — 赛场分区掩码绘制工具
====================================
用于在赛场地图上绘制高度分区掩码：
  - 黑色区域 (0,0,0)  → 地面层（默认）
  - 绿色区域 (0,255,0) → 高地层（中央高地 / 环形高地）

使用方法:
  python -m hnurm_radar.camera_locator.make_mask --map /path/to/map/std_map.png --out /path/to/map/map_mask.png

操作说明:
  - 左键拖拽 → 绘制多边形顶点
  - 右键        → 闭合当前多边形并填充为绿色（高地区域）
  - 按 'u'      → 撤销上一个多边形
  - 按 's'      → 保存掩码并退出
  - 按 'q'      → 不保存直接退出
"""

import cv2
import numpy as np
import argparse
import os


class MaskMaker:
    """交互式赛场分区掩码绘制器"""

    # 颜色定义
    COLOR_GROUND = (0, 0, 0)        # 黑色 → 地面层
    COLOR_HIGHLAND = (0, 255, 0)    # 绿色 → 高地层

    def __init__(self, map_image_path: str, output_path: str):
        self.map_img = cv2.imread(map_image_path)
        if self.map_img is None:
            raise FileNotFoundError(f"无法读取地图图片: {map_image_path}")

        self.output_path = output_path
        self.h, self.w = self.map_img.shape[:2]

        # 掩码图：初始全黑（地面层）
        self.mask = np.zeros((self.h, self.w, 3), dtype=np.uint8)

        # 已完成的多边形列表（用于撤销）
        self.polygons = []

        # 当前正在绘制的多边形顶点
        self.current_points = []

        self.window_name = "Mask Maker - 左键加点 | 右键闭合 | u撤销 | s保存 | q退出"

    def _redraw(self):
        """重绘掩码和显示图像"""
        # 重建掩码
        self.mask = np.zeros((self.h, self.w, 3), dtype=np.uint8)
        for poly in self.polygons:
            pts = np.array(poly, dtype=np.int32)
            cv2.fillPoly(self.mask, [pts], self.COLOR_HIGHLAND)

        # 叠加显示：地图 + 半透明掩码
        display = self.map_img.copy()
        overlay = self.mask.copy()
        alpha = 0.4
        cv2.addWeighted(overlay, alpha, display, 1.0, 0, display)

        # 画已完成的多边形轮廓
        for poly in self.polygons:
            pts = np.array(poly, dtype=np.int32)
            cv2.polylines(display, [pts], True, (0, 200, 0), 2)

        # 画当前正在绘制的点
        for i, pt in enumerate(self.current_points):
            cv2.circle(display, tuple(pt), 5, (0, 0, 255), -1)
            if i > 0:
                cv2.line(display, tuple(self.current_points[i - 1]),
                         tuple(pt), (0, 0, 255), 2)

        # 显示状态
        status = f"已完成 {len(self.polygons)} 个区域 | 当前 {len(self.current_points)} 个点"
        cv2.putText(display, status, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        cv2.imshow(self.window_name, display)

    def _mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # 左键：添加顶点
            self.current_points.append([x, y])
            self._redraw()

        elif event == cv2.EVENT_RBUTTONDOWN:
            # 右键：闭合当前多边形
            if len(self.current_points) >= 3:
                self.polygons.append(self.current_points.copy())
                self.current_points = []
                self._redraw()
                print(f"[make_mask] 已完成第 {len(self.polygons)} 个高地区域")
            else:
                print("[make_mask] 至少需要 3 个点才能闭合多边形")

    def run(self):
        """启动交互式掩码绘制"""
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, min(self.w, 1400), min(self.h, 800))
        cv2.setMouseCallback(self.window_name, self._mouse_callback)
        self._redraw()

        print("=" * 60)
        print("赛场分区掩码绘制工具")
        print("  左键: 添加多边形顶点")
        print("  右键: 闭合当前多边形（标记为高地区域）")
        print("  u:    撤销上一个多边形")
        print("  s:    保存掩码并退出")
        print("  q:    不保存退出")
        print("=" * 60)

        while True:
            key = cv2.waitKey(50) & 0xFF
            if key == ord('u') or key == ord('U'):
                # 撤销
                if self.current_points:
                    self.current_points = []
                    print("[make_mask] 已清除当前绘制点")
                elif self.polygons:
                    self.polygons.pop()
                    print(f"[make_mask] 已撤销，剩余 {len(self.polygons)} 个区域")
                self._redraw()

            elif key == ord('s') or key == ord('S'):
                # 保存
                if self.current_points and len(self.current_points) >= 3:
                    self.polygons.append(self.current_points.copy())
                    self.current_points = []

                # 最终重建掩码
                self.mask = np.zeros((self.h, self.w, 3), dtype=np.uint8)
                for poly in self.polygons:
                    pts = np.array(poly, dtype=np.int32)
                    cv2.fillPoly(self.mask, [pts], self.COLOR_HIGHLAND)

                os.makedirs(os.path.dirname(self.output_path), exist_ok=True)
                cv2.imwrite(self.output_path, self.mask)
                print(f"[make_mask] 掩码已保存到: {self.output_path}")
                print(f"[make_mask] 共 {len(self.polygons)} 个高地区域")
                break

            elif key == ord('q') or key == ord('Q'):
                print("[make_mask] 未保存，退出。")
                break

        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="赛场分区掩码绘制工具")
    parser.add_argument("--map", type=str,
                        default=None,
                        help="赛场地图图片路径")
    parser.add_argument("--out", type=str,
                        default=None,
                        help="输出掩码图片路径")
    args = parser.parse_args()

    # 使用 paths 模块提供默认值
    map_path = args.map
    out_path = args.out
    if map_path is None or out_path is None:
        try:
            from ..shared.paths import STD_MAP_PATH, MAP_DIR
            if map_path is None:
                map_path = STD_MAP_PATH
            if out_path is None:
                out_path = os.path.join(MAP_DIR, "map_mask.png")
        except ImportError:
            # 直接运行时回退到相对路径
            base = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
            if map_path is None:
                map_path = os.path.join(base, "map", "std_map.png")
            if out_path is None:
                out_path = os.path.join(base, "map", "map_mask.png")

    maker = MaskMaker(map_path, out_path)
    maker.run()


if __name__ == "__main__":
    main()
