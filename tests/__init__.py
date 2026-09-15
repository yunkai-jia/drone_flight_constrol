"""测试包：把 src 加入路径，便于直接 ``python -m unittest``。

不依赖真机；模式映射与 URI 解析都是纯函数测试。
"""

from __future__ import annotations

import sys
from pathlib import Path

_SRC = Path(__file__).resolve().parents[1] / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))
