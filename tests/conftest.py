"""
conftest.py — pytest 全局配置

设置 Python 路径，使得测试可以直接 import 项目源码模块。
"""

import os
import sys

# 将 src/hnurm_radar 加入 Python 路径，使 hnurm_radar 包可被 import
_PROJECT_ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), ".."))
_SRC_HNURM = os.path.join(_PROJECT_ROOT, "src", "hnurm_radar")
_SRC_EKF = os.path.join(_PROJECT_ROOT, "src", "ekf")

for _p in [_SRC_HNURM, _SRC_EKF]:
    if _p not in sys.path:
        sys.path.insert(0, _p)
