from __future__ import annotations

import numpy as np


def topk_support(abs_g, k: int) -> set[int]:
    values = np.asarray(abs_g, dtype=float)
    if k <= 0 or values.size == 0 or np.all(values <= 0):
        return set()
    k = min(k, values.size)
    idx = np.argsort(values)[-k:]
    return set(int(i) for i in idx if values[i] > 0)


def alpha_support(abs_g, alpha: float) -> set[int]:
    values = np.asarray(abs_g, dtype=float)
    if values.size == 0 or np.all(values <= 0):
        return set()
    threshold = alpha * float(np.max(values))
    return set(int(i) for i, value in enumerate(values) if value >= threshold and value > 0)
