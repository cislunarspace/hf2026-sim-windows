# OpenSim 仿真平台

版本: 1.1.1 | 平台: Linux x86-64 / Windows x64

## 平台支持

本发布包同时提供 Linux 和 Windows 两套脚本：

| 平台 | 脚本后缀 | 快速开始 |
|---|---|---|
| Linux x86-64（glibc ≥ 2.39，Ubuntu 24.04+） | `.sh` | `./setup.sh && ./start.sh` |
| Windows 10 1809+ | `.ps1` | `.\setup.ps1; .\start.ps1` |

两套脚本功能等价（端口顺延、pidfile 幂等、两段式 kill、UE 孤儿清理）。
Windows 版内置 `bin\redis-server.exe`（redis-windows-fork）+ 依赖 DLL，
不需要单独安装 Redis。

## 快速开始

### Linux
```bash
./setup.sh    # 检测 uv、创建 Python venv、安装 redis/pyyaml（需 uv；apt 包需 sudo）
./start.sh    # 启动 Redis + bridge + 前端
# 浏览器打开 http://localhost:3000
#   → 选赛题 → (可选)「算法」框填 module:Class → 点「开始仿真」
```

### Windows (PowerShell)
```powershell
.\setup.ps1   # 检测 uv、创建 Python venv、安装 redis/pyyaml/vcredist；vcredist 安装可能需要管理员
.\start.ps1   # 启动 Redis + bridge + 前端
# 浏览器自动打开 http://localhost:3000
#   → 选赛题 → (可选)「算法」框填 module:Class → 点「开始仿真」
```

> 注：`setup.ps1` 会自动检测 Microsoft Visual C++ 2015-2022 Redistributable (x64)，若未安装，会静默运行包内附带的 `vc_redist.x64.exe`。该步骤可能需要管理员权限；安装成功后建议重启系统以确保 SxS 运行时装载生效。

## 系统要求

- **uv**：Python 虚拟环境与依赖管理工具（[安装指南](https://docs.astral.sh/uv/getting-started/installation/)）
- **Linux**: x86-64，glibc ≥ 2.39（Ubuntu 24.04+），apt 包管理器
- **Windows**: Windows 10 1809+（内置 tar/curl/PowerShell 5.1+），推荐管理员权限运行 `setup.ps1` 以便自动安装 VC++ Redistributable
- 捆绑 Python 3.12 standalone，通过 uv 创建虚拟环境管理依赖
- GPU（可选，仅 UE 渲染用；无 GPU 自动降级 Three.js 自渲染）

## 选手算法接入

1. 写一个继承 SDK 基类的 agent（参考 `competition/baselines/`）：

```python
# my_agent.py
from competition.sdk.scenarios.search_track import SearchTrackAgent
from competition.sdk.core.commands import Command, fly_to, point_gimbal

class MyAgent(SearchTrackAgent):
    def decide(self, obs, dt) -> list:
        # 你的算法逻辑：根据 obs（观测）返回指令列表
        return []
```

2. 把 `my_agent.py` 放到 release 包根目录（或设 `PYTHONPATH` 包含其所在目录）

3. 前端选赛题 → 「算法」框填 `my_agent:MyAgent` → 点「开始仿真」

也可用命令行（进阶）：

```bash
# Linux
PYTHONPATH=. python -m competition run --scenario search_track \
  --agent my_agent:MyAgent --start-sim --visualize
```

```powershell
# Windows (PowerShell)
$env:PYTHONPATH = '.'
python -m competition run --scenario search_track `
  --agent my_agent:MyAgent --start-sim --visualize
```

## UE 渲染器配置（可选）

默认使用 Three.js 自渲染。若要用 UE 真实渲染：

1. 复制 `config/renderers/ue_testwl.template.json` 为 `ue_testwl.json`
2. 把 `workdir` 改成你的 UE 打包产物路径
3. 确保机器有 GPU（≥8GB VRAM，支持 Vulkan）
4. 重启 `./start.sh`

## 端口配置

所有端口可通过环境变量覆盖：

### Linux
```bash
OPENSIM_REDIS_PORT=6380 OPENSIM_WEB_PORT=3001 ./start.sh
```

### Windows (PowerShell)
```powershell
$env:OPENSIM_REDIS_PORT = '6380'; $env:OPENSIM_WEB_PORT = '3001'; .\start.ps1
```

| 变量 | 默认 | 用途 |
|---|---|---|
| OPENSIM_REDIS_PORT | 6379 | Redis |
| OPENSIM_WS_PORT | 8080 | bridge WebSocket |
| OPENSIM_CAM_PORT | 8081 | bridge HTTP（相机帧 + sim 控制） |
| OPENSIM_WEB_PORT | 3000 | 前端静态服务 |

## 停止与检查

### Linux
```bash
./stop.sh      # 停止所有进程（含 UE 孤儿兜底清理）
./verify.sh    # 健康检查
```

### Windows (PowerShell)
```powershell
.\stop.ps1     # 停止所有进程（含 UE 孤儿兜底清理）
.\verify.ps1   # 健康检查
```

## 故障排查

### 一键诊断（推荐给远程支持场景）

遇到问题且无法自行定位时，在发布包根目录运行诊断脚本，它会自动收集
系统信息、包完整性、依赖状态、端口/进程和全部日志，生成一个压缩包：

```bash
./diagnose.sh        # Linux —— 生成 opensim-diagnostics-<时间戳>.tar.gz
```

```powershell
.\diagnose.ps1       # Windows —— 生成 opensim-diagnostics-<时间戳>.zip
# 若被执行策略拦截：powershell -ExecutionPolicy Bypass -File .\diagnose.ps1
```

把生成的压缩包发回给运维/开发即可，无需手动翻日志。脚本只读不写，可随时重复执行。

### 手动排查（Linux）
```bash
./verify.sh                          # 定位哪个组件有问题
tail -f run/logs/redis.log           # Redis 日志
tail -f run/logs/bridge.log          # bridge 日志
tail -f run/logs/frontend.log        # 前端服务日志
ls competition/scenarios/*/output/     # 引擎/控制器输出（点赛题后才有）
```

### 手动排查（Windows PowerShell）
```powershell
.\verify.ps1                                    # 定位哪个组件有问题
Get-Content run\logs\redis.log -Wait            # Redis 日志
Get-Content run\logs\bridge.log -Wait           # bridge 日志
Get-Content run\logs\frontend.log -Wait         # 前端服务日志
Get-ChildItem competition\scenarios\*\output\         # 引擎/控制器输出（点赛题后才有）
# 进程/端口排查：
Get-NetTCPConnection -State Listen              # 监听端口
Get-CimInstance Win32_Process | Where CommandLine -match 'opensim'  # 找进程
Stop-Process -Id <pid> -Force                   # 强杀进程
```

## 目录结构

| 路径 | 说明 |
|---|---|
| opensim-sim | C++ 仿真引擎（hiredis/redis++/cesium 已静态链接） |
| opensim-render-ctl | 渲染编排 CLI（UE spawn plan） |
| bin/node | 内置 Node.js v22 |
| bin/redis-server, redis-cli | 内置 Redis |
| visualization/dist-bridge/ | bridge 编译产物（Redis↔WebSocket 转发） |
| frontend/ | 前端静态文件（webpack 构建） |
| competition/ | 比赛 SDK + baseline 算法（Python） |
| config/ | 仿真配置 + 地形数据（CSV）+ UE 配置 |
| run/ | 运行时产物（日志/PID/输出，脚本启动时自动创建） |
| output/ | 评分结果 `*.evaluation.json` 落盘目录（运行后生成） |
