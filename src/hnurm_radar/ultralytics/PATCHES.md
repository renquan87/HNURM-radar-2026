# Ultralytics 修改点清单

> 本文档记录项目内嵌 Ultralytics 相对于官方版本的所有自定义修改。
> 
> **基线版本**: Ultralytics `8.1.9`（`src/hnurm_radar/ultralytics/__init__.py` 中 `__version__`）
> 
> **内嵌位置**: `src/hnurm_radar/ultralytics/`
> 
> **修改目的**: 添加多种自定义 Backbone、检测头、注意力模块，用于 YOLO 模型架构实验。

---

## 1. 自定义 Backbone（`nn/backbone/`）

项目新增了 14 个 Backbone 模块，均以 `from ultralytics.nn.backbone.xxx import *` 方式在 `nn/tasks.py` 中注册。

| 文件 | Backbone 名称 | 说明 |
|------|--------------|------|
| `convnextv2.py` | ConvNeXt V2 | Facebook 2023，纯卷积 Backbone（atto/femto/pico/nano/tiny/base/large/huge 变体） |
| `fasternet.py` | FasterNet | 基于部分卷积的高效网络（t0/t1/t2/s/m/l 变体），需 `faster_cfg/` 配置 |
| `efficientViT.py` | EfficientViT | 高效 Vision Transformer（M0-M5 变体） |
| `EfficientFormerV2.py` | EfficientFormer V2 | 高效 Transformer（s0/s1/s2/l 变体） |
| `VanillaNet.py` | VanillaNet | 极简深度学习网络（5-13 层变体 + x1.5 + ada_pool） |
| `revcol.py` | RevCol | 可逆列网络 |
| `lsknet.py` | LSKNet | 大选择性核注意力网络（t/s 变体） |
| `SwinTransformer.py` | Swin Transformer | 层级化 Vision Transformer（Tiny 变体） |
| `repvit.py` | RepViT | 重参数化 ViT（m0.9/m1.0/m1.1/m1.5/m2.3 变体） |
| `CSwomTramsformer.py` | CSWin Transformer | 交叉形窗口注意力（tiny/small/base/large 变体） |
| `UniRepLKNet.py` | UniRepLKNet | 统一重参数化大核网络（a/f/p/n/t/s/b/l/xl 变体） |
| `TransNext.py` | TransNeXt | 像素级 Transformer（micro/tiny/small/base 变体），含 CUDA 加速版（`TransNeXt/` 子目录） |
| `rmt.py` | RMT | 保持性记忆 Transformer（T/S/B/L 变体） |
| `pkinet.py` | PKINet | 多核内连网络（T/S/B 变体） |

### TransNeXt 特殊说明

`nn/backbone/TransNeXt/` 目录包含：
- `TransNext_cuda.py` — CUDA 优化版本（需编译 `swattention_extension/`）
- `TransNext_native.py` — 纯 PyTorch 实现
- `swattention_extension/` — CUDA 自定义算子源码（`.cu` 文件 + `setup.py`）

---

## 2. 额外模块（`nn/extra_modules/`）

在 `nn/extra_modules/__init__.py` 中统一导入，提供自定义的检测头、注意力、卷积模块。

| 文件 | 模块类别 | 关键组件 |
|------|---------|---------|
| `afpn.py` | 特征金字塔 | AFPN（自适应特征金字塔网络）、`Detect_AFPN_P2345`、`Detect_AFPN_P345` 及 Custom 变体 |
| `attention.py` | 注意力模块 | EMA、CBAM、BAMBlock、LSKBlock、SEAttention、CPCA、CoordAtt、TripletAttention、LSKA、DAttention、MLCA、FocusedLinearAttention、ELA、CAA 等 30+ 种注意力 |
| `block.py` | 卷积/特征块 | C2f/C3 变体（Faster、ODConv、DBB、CloAtt、SCConv、EMSC、EMSCP、DCNv2/v3/v4、DySnakeConv、OREPA、DRB、iRMB、VSS、RVB、PKIModule、FADC、PPA 等）；GSConv/VoVGSCSP 系列；CSPStage、SPDConv、RepBlock、ADown 等 |
| `head.py` | 检测头 | `Detect_DyHead`、`Detect_Efficient`、`DetectAux`、`Detect_SEAM`、`Detect_MultiSEAM`、`Detect_DyHeadWithDCNV3/V4`、`Detect_DyHead_Prune`、`Detect_LSCD`、`Detect_TADDH`、`Segment_Efficient/LSCD/TADDH`、`Pose_LSCD/TADDH`、`OBB_LSCD/TADDH` |
| `rep_block.py` | 重参数化块 | RepVGGBlock、OREPA 相关的重参数化卷积 |
| `kernel_warehouse.py` | 核仓库 | KWConv（动态卷积核仓库）、C2f_KW、C3_KW |
| `dynamic_snake_conv.py` | 动态蛇形卷积 | DySnakeConv、C2f_DySnakeConv、C3_DySnakeConv |
| `orepa.py` | 在线重参数化 | OREPA、OREPA_LargeConv、RepVGGBlock_OREPA |
| `RFAConv.py` | 感受野注意力卷积 | RFAConv、RFCAConv、RFCBAMConv 及 C2f/C3 封装 |
| `hcfnet.py` | HCF-Net | 分层上下文融合网络模块（PPA 等） |
| `dyhead_prune.py` | 动态头剪枝 | DyHead 剪枝版本 |
| `fadc.py` | 频率自适应空洞卷积 | FADC、C2f_FADC、C3_FADC |
| `shiftwise_conv.py` | 移位卷积 | SWC（Shift-Wise Conv）、C2f_SWC、C3_SWC |
| `mamba_vss.py` | Mamba/VMamba | VSS（Visual State Space）块、C2f_VSS、C3_VSS、LVMB |
| `prune_module.py` | 剪枝工具 | 模型剪枝辅助模块 |
| `MyPruner.py` | 自定义剪枝器 | 自定义剪枝实现 |

---

## 3. 第三方 CUDA 算子（`nn/extra_modules/`）

| 目录 | 算子 | 说明 |
|------|------|------|
| `DCNv4_op/` | DCNv4（可变形卷积 V4） | 完整的 CUDA 实现 + Python 绑定（需 `make.sh` 编译） |
| `ops_dcnv3/` | DCNv3（可变形卷积 V3） | CUDA + CPU 实现（需 `make.sh` 编译） |
| `mamba/` | Mamba SSM | Selective Scan 选择性扫描算子（Triton + CUDA） |

---

## 4. 核心文件修改（`nn/tasks.py`）

`tasks.py` 是 Ultralytics 模型构建的核心文件，**大量修改**：

### 4.1 Import 区域（L28-42）
新增 14 个 backbone 的 wildcard import：
```python
from ultralytics.nn.backbone.convnextv2 import *
from ultralytics.nn.backbone.fasternet import *
# ... 共 14 行
from ultralytics.nn.backbone.pkinet import *
```

### 4.2 `_predict_once()` / `_predict_augment()` 方法
在 Detect 类型判断处扩展了所有自定义检测头类型（`Detect_DyHead`, `Detect_AFPN_*`, `Detect_Efficient`, `DetectAux`, `Detect_SEAM` 等）。

### 4.3 `parse_model()` 函数（L820+）
这是最大修改区域，添加了 **200+ 个自定义模块类** 到模型解析逻辑中：
- Conv/Block 注册区（L823-839）：所有 C2f/C3 变体、注意力模块、特殊卷积等
- Backbone 注册区（L921-935）：所有自定义 backbone 的构造函数
- 注意力模块注册区（L941-945）：独立的 channel-preserving 注意力模块
- Detect 头注册区（L889-895）：所有自定义检测头

### 4.4 模型类型判断
`guess_model_task()` 中扩展了 Detect/Segment/Pose/OBB 的自定义头类型判断。

---

## 5. 其他可能修改的文件

| 文件 | 修改类型 | 说明 |
|------|---------|------|
| `nn/modules/block.py` | 可能扩展 | 标准模块可能有微调 |
| `nn/modules/conv.py` | 可能扩展 | 卷积模块可能有修改 |
| `nn/modules/head.py` | 可能扩展 | 检测头基类可能有修改 |
| `nn/autobackend.py` | 未确认 | 推理后端可能有适配修改 |

> ⚠️ 以上文件未做逐行 diff 确认，建议未来升级时使用 `diff -r` 与官方 v8.1.9 对比。

---

## 6. 升级建议

### 短期（当前）
- ✅ 本文档已记录所有修改点
- 保持内嵌版本不动，稳定优先

### 中期
- 将 `nn/backbone/` 和 `nn/extra_modules/` 提取为独立 Python 包
- 利用 Ultralytics 的插件注册机制（`register_module()`），基于官方最新版加载自定义模块
- 这样可以跟随官方版本升级而不丢失自定义功能

### 长期
- 评估 YOLOv11 / YOLO-World 等新架构
- 评估开放词汇检测（Open Vocabulary Detection）降低标注成本
- 确认自定义 backbone 是否仍有性能优势

---

## 7. 当前实际使用的模块

根据三阶段模型的推理配置，**实际训练使用的自定义模块**可能只是所有可用模块的一个子集。
具体使用了哪些 backbone/head 取决于训练时选择的 YAML 配置文件（不在本仓库中）。

**确认方法**：
```python
# 加载模型查看实际架构
from ultralytics import YOLO
model = YOLO("data/weights/stage_one/stage_one.pt")
print(model.model)  # 查看完整模型结构
```

---

*最后更新: 2026-04-06*
