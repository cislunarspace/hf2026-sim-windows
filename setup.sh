#!/bin/bash
# setup.sh — OpenSim release 包环境检测与依赖自动安装
# 检测 python3/pip/redis(python)/pyyaml 等依赖，缺失则自动 apt/pip 安装。
# 幂等：已装的跳过，重复运行安全。

set -eu

NEED_APT=()
NEED_PIP=()
HAS_APT=0
command -v apt-get >/dev/null 2>&1 && HAS_APT=1

# 1. 系统命令（Ubuntu 一般都有，缺了装对应的包）
command -v lsof   >/dev/null 2>&1 || NEED_APT+=(lsof)
command -v curl   >/dev/null 2>&1 || NEED_APT+=(curl)
command -v pgrep  >/dev/null 2>&1 || NEED_APT+=(procps)

# 2. Python 解释器
if ! command -v python3 >/dev/null 2>&1; then
    NEED_APT+=(python3 python3-pip)
else
    # 3. pip
    if ! python3 -m pip --version >/dev/null 2>&1; then
        NEED_APT+=(python3-pip)
    fi
    # 4. Python 包
    python3 -c "import redis" 2>/dev/null || NEED_PIP+=(redis)
    python3 -c "import yaml"  2>/dev/null || NEED_PIP+=(pyyaml)
fi

# 5. glibc 版本检测（引擎要求 >= 2.39，不满足警告但不阻断）
GLIBC_VER=$(ldd --version 2>/dev/null | head -1 | awk '{print $NF}')
warn_glibc() {
    if [ -n "${GLIBC_VER:-}" ]; then
        major=$(echo "$GLIBC_VER" | cut -d. -f1)
        minor=$(echo "$GLIBC_VER" | cut -d. -f2)
        if [ "${major:-0}" -lt 2 ] 2>/dev/null || { [ "${major:-0}" -eq 2 ] 2>/dev/null && [ "${minor:-0}" -lt 39 ] 2>/dev/null; }; then
            echo "⚠️  glibc $GLIBC_VER < 2.39，引擎二进制可能无法运行（建议 Ubuntu 24.04+）"
        fi
    fi
}

# 6. 汇总与安装
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

# pip 安装（apt 装完 python3-pip 后重新探测）
if ! python3 -m pip --version >/dev/null 2>&1; then
    echo "✗ pip 仍不可用，请手动安装 python3-pip"
    exit 1
fi
if [ ${#NEED_PIP[@]} -gt 0 ]; then
    echo "缺少 Python 包: ${NEED_PIP[*]}"
    python3 -m pip install --user "${NEED_PIP[@]}"
fi

warn_glibc
echo "✓ 环境就绪，可运行 ./start.sh"
