#!/bin/bash
# ============================================================================
# setup_assets.sh — 克隆仓库后一键获取运行所需的大文件
# ============================================================================
#
# 用法:
#   cd <项目根目录>
#   bash scripts/setup_assets.sh
#
# 该脚本完成以下工作:
#   1. 通过 Git LFS 拉取 .pt 模型文件 和 .pcd 点云数据
#   2. 从 .pt 文件导出 .engine 文件 (TensorRT, GPU特定)
#
# 环境要求:
#   - git-lfs (apt install git-lfs)
#   - Python 3 + ultralytics (用于导出 .engine, 可选)
#   - NVIDIA GPU + TensorRT (用于导出 .engine, 可选)
# ============================================================================

set -euo pipefail

# ---- 颜色输出 ----
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

info()  { echo -e "${BLUE}[INFO]${NC} $*"; }
ok()    { echo -e "${GREEN}[OK]${NC} $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC} $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; }

# ---- 项目根目录 ----
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PROJECT_ROOT"

# ============================================================================
# 步骤 1: Git LFS — 拉取 .pt 模型 + .pcd 点云数据
# ============================================================================
echo ""
echo "============================================"
info "步骤 1/2: 通过 Git LFS 拉取大文件"
echo "============================================"

if ! command -v git-lfs &>/dev/null && ! git lfs version &>/dev/null; then
    error "git-lfs 未安装！请先执行:"
    echo "  sudo apt install git-lfs"
    echo "  git lfs install"
    exit 1
fi
ok "git-lfs 已安装: $(git lfs version 2>&1 | head -1)"

# 拉取所有 LFS 文件
info "正在拉取 LFS 文件 (.pt 模型 + .pcd 点云)..."
git lfs pull

# 验证
LFS_FILES=(
    "weights/stage_one.pt"
    "weights/stage_two.pt"
    "weights/stage_three.pt"
    "data/background.pcd"
    "data/pcds.pcd"
    "data/pcds_downsampled.pcd"
)

# 可选的 lab 文件
LFS_OPTIONAL=(
    "data/lab_pcds.pcd"
    "data/lab_pcds_downsampled.pcd"
    "data/normalized_map.pcd"
    "data/normalized_map_downsample.pcd"
)

ALL_OK=true
for f in "${LFS_FILES[@]}"; do
    if [ -f "$f" ] && [ "$(wc -c < "$f")" -gt 1000 ]; then
        ok "$f ✓ ($(du -h "$f" | cut -f1))"
    else
        error "$f 拉取失败或仍为 LFS pointer"
        ALL_OK=false
    fi
done

for f in "${LFS_OPTIONAL[@]}"; do
    if [ -f "$f" ] && [ "$(wc -c < "$f")" -gt 1000 ]; then
        ok "$f ✓ ($(du -h "$f" | cut -f1)) [可选]"
    else
        info "$f - 未找到 (lab场景可选，不影响比赛运行)"
    fi
done

if [ "$ALL_OK" != true ]; then
    error "部分必需文件拉取失败，请检查网络连接和 Git LFS 配置"
    exit 1
fi

# ============================================================================
# 步骤 2: 导出 .engine 文件 (TensorRT)
# ============================================================================
echo ""
echo "============================================"
info "步骤 2/2: 从 .pt 导出 .engine 文件 (TensorRT)"
echo "============================================"

ENGINE_FILES=("weights/stage_one.engine" "weights/stage_two.engine" "weights/stage_three.engine")
NEED_EXPORT=false

for eng in "${ENGINE_FILES[@]}"; do
    if [ ! -f "$eng" ]; then
        warn "$eng 不存在，需要导出"
        NEED_EXPORT=true
    else
        ok "$eng 已存在 ($(du -h "$eng" | cut -f1))"
    fi
done

if [ "$NEED_EXPORT" = true ]; then
    if ! python3 -c "import ultralytics" 2>/dev/null; then
        warn "ultralytics 未安装，跳过 .engine 导出"
        warn "请先安装依赖: pip install -r requirements.txt"
        warn "然后手动运行: python3 scripts/export_models.py"
    elif ! python3 -c "import torch; assert torch.cuda.is_available()" 2>/dev/null; then
        warn "CUDA 不可用，跳过 .engine 导出"
        warn "请在有 NVIDIA GPU 的机器上运行: python3 scripts/export_models.py"
    else
        info "正在导出 .engine 文件（首次可能需要几分钟）..."
        python3 scripts/export_models.py
        for eng in "${ENGINE_FILES[@]}"; do
            if [ -f "$eng" ]; then
                ok "$eng 导出成功 ($(du -h "$eng" | cut -f1))"
            else
                error "$eng 导出失败"
            fi
        done
    fi
else
    ok "所有 .engine 文件已就绪"
fi

# ============================================================================
# 最终状态
# ============================================================================
echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  ✅ 资产准备完成！${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "如果 .engine 文件未导出，请手动执行:"
echo "  python3 scripts/export_models.py"
