"""Bearing-Only EKF / IMM Python 接口（薄壳）。

ImmFilter 优先用 rust_core（PyO3）实现；rust_core 不可用时
（如提交环境没有编译产物）回退到纯 Python 版
`algorithms.estimation.imm_py.ImmFilter`，API 完全一致。

BearingOnlyEKF 只有 Rust 实现，rust_core 不可用时为 None。
"""

try:
    from rust_core import BearingOnlyEKF, ImmFilter
except ImportError:
    from algorithms.estimation.imm_py import ImmFilter

    BearingOnlyEKF = None

__all__ = ["BearingOnlyEKF", "ImmFilter"]
