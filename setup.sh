#!/bin/bash
# setup.sh — OpenSim release 包环境检测与依赖自动安装
# 用 uv 管理 Python 虚拟环境，安装 redis/pyyaml 等依赖。
# 幂等：已装的跳过，重复运行安全。

set -eu

PACK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

NEED_APT=()
HAS_APT=0
command -v apt-get >/dev/null 2>&1 && HAS_APT=1

# 1. 系统命令（Ubuntu 一般都有，缺了装对应的包）
command -v lsof   >/dev/null 2>&1 || NEED_APT+=(lsof)
command -v curl   >/dev/null 2>&1 || NEED_APT+=(curl)
command -v pgrep  >/dev/null 2>&1 || NEED_APT+=(procps)

# 2. uv（Python 虚拟环境 & 依赖管理）
if ! command -v uv >/dev/null 2>&1; then
    echo "✗ uv 未安装。uv 是管理 Python 虚拟环境的必需工具。"
    echo "  安装方式（任选其一）："
    echo "    curl -LsSf https://astral.sh/uv/install.sh | sh"
    echo "    pip install uv"
    echo "  安装后重新运行 ./setup.sh"
    exit 1
fi
echo "✓ uv: $(uv --version)"

# 3. glibc 版本检测（引擎要求 >= 2.35，即 Ubuntu 22.04+；不满足警告但不阻断）
GLIBC_VER=$(ldd --version 2>/dev/null | head -1 | awk '{print $NF}')
warn_glibc() {
    if [ -n "${GLIBC_VER:-}" ]; then
        major=$(echo "$GLIBC_VER" | cut -d. -f1)
        minor=$(echo "$GLIBC_VER" | cut -d. -f2)
        if [ "${major:-0}" -lt 2 ] 2>/dev/null || { [ "${major:-0}" -eq 2 ] 2>/dev/null && [ "${minor:-0}" -lt 35 ] 2>/dev/null; }; then
            echo "⚠️  glibc $GLIBC_VER < 2.35，引擎二进制可能无法运行（需 Ubuntu 22.04+）"
        fi
    fi
}

# 4. 汇总系统包安装
if [ ${#NEED_APT[@]} -gt 0 ]; then
    echo "缺少系统包: ${NEED_APT[*]}"
    if [ "$HAS_APT" -eq 1 ]; then
        echo "使用 apt-get 安装（需要 sudo 权限）..."
        SUDO=""
        [ "$(id -u)" -ne 0 ] && SUDO="sudo"
        $SUDO apt-get update -qq && $SUDO apt-get install -y "${NEED_APT[@]}"
    else
        echo "✗ 非 Debian/Ubuntu 系统（无 apt-get），请手动安装上述包后重试。"
        echo "  Ubuntu/Debian: sudo apt-get install ${NEED_APT[*]}"
        exit 1
    fi
fi

# 5. Python 虚拟环境（uv 管理）
BUNDLED_PY="$PACK_ROOT/python/bin/python3.12"
VENV_PY="$PACK_ROOT/.venv/bin/python3"

if [ -x "$VENV_PY" ]; then
    echo "✓ Python venv 已存在: $PACK_ROOT/.venv"
else
    if [ ! -x "$BUNDLED_PY" ]; then
        echo "✗ 捆绑 Python 缺失（python/bin/python3.12），发行包不完整"
        exit 1
    fi
    echo "创建 Python 虚拟环境（uv venv）..."
    uv venv --python "$BUNDLED_PY" "$PACK_ROOT/.venv"
    echo "✓ Python venv 已创建: $PACK_ROOT/.venv"
fi

# 6. 安装 Python 依赖
echo "安装 Python 依赖（redis + pyyaml）..."
uv pip install --python "$VENV_PY" redis pyyaml
echo "✓ Python 依赖已安装到 venv"

# 7. 验证
if "$VENV_PY" -c "import redis, yaml" 2>/dev/null; then
    echo "✓ Python 依赖验证通过（redis + pyyaml）"
else
    echo "⚠️  venv 的 redis/pyyaml 不可用，发行包可能损坏"
    exit 1
fi

warn_glibc
echo "✓ 环境就绪，可运行 ./start.sh"
