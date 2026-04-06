"""
label_mappings.py — 装甲板标签映射常量

RoboMaster 雷达站三阶段推理中，Stage 2 输出 12 类（B1-B5,B7,R1-R5,R7），
Stage 3 输出 6 类灰色装甲板（gray_0~gray_5）。灰色装甲板需要根据当前
track_id 的历史投票结果重映射到红/蓝阵营的具体编号。

标签索引约定（与 detector_config.yaml params.labels 一致）：
  0=B1, 1=B2, 2=B3, 3=B4, 4=B5, 5=B7
  6=R1, 7=R2, 8=R3, 9=R4, 10=R5, 11=R7

灰色装甲板索引（Stage 3 输出）：
  12=gray_B7/R7(哨兵), 13=gray_B2/R2, 14=gray_B1/R1,
  15=gray_B4/R4, 16=gray_B3/R3, 17=gray_B5/R5
"""

# ── 灰色装甲板 → 蓝方编号 ──
# key: Stage 3 灰色类别索引, value: 对应蓝方 Stage 2 类别索引
Gray2Blue = {12: 5, 13: 1, 14: 0, 15: 3, 16: 2, 17: 4}

# ── 灰色装甲板 → 红方编号 ──
Gray2Red = {12: 11, 13: 7, 14: 6, 15: 9, 16: 8, 17: 10}

# ── Stage 3 灰色内部索引 → 全局灰色索引 ──
# key: Stage 3 模型输出的 class_id (0~5), value: 全局灰色索引 (12~17)
gray2gray = {0: 14, 1: 13, 2: 16, 3: 15, 4: 17, 5: 12}

# ── 反向映射（自动生成） ──
Blue2Gray = {v: k for k, v in Gray2Blue.items()}
Red2Gray = {v: k for k, v in Gray2Red.items()}
