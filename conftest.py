"""pytest 配置：确保 algorithms/ 和 rust_core 可 import。"""
import sys
from pathlib import Path

# 把 worktree 根目录加入 sys.path，使 `import algorithms` 和 `import rust_core` 可用
_root = Path(__file__).resolve().parent
if str(_root) not in sys.path:
    sys.path.insert(0, str(_root))
