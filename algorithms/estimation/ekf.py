"""Bearing-Only EKF Python 接口（薄壳）。

底层实现在 rust_core（PyO3 扁平模块），这里只做 import 转发。
"""
from rust_core import BearingOnlyEKF

__all__ = ["BearingOnlyEKF"]
