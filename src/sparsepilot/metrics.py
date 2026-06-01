from __future__ import annotations

import math

import numpy as np


def topk_coverage(abs_g, k: int) -> float:
    values = np.asarray(abs_g, dtype=float)
    total = float(np.sum(values))
    if total <= 0 or k <= 0:
        return 0.0
    k = min(k, values.size)
    return float(np.sum(np.sort(values)[-k:]) / total)


def effective_sparsity(g) -> float:
    values = np.asarray(g, dtype=float)
    l2_sq = float(np.sum(values * values))
    if l2_sq <= 0:
        return 0.0
    l1 = float(np.sum(np.abs(values)))
    return (l1 * l1) / l2_sq


def normalized_entropy(abs_g) -> float:
    values = np.asarray(abs_g, dtype=float)
    total = float(np.sum(values))
    if total <= 0 or values.size <= 1:
        return 0.0
    p = values / total
    p = p[p > 0]
    entropy = -float(np.sum(p * np.log(p)))
    return entropy / math.log(values.size)


def jaccard(a: set, b: set) -> float:
    if not a and not b:
        return 1.0
    union = a | b
    if not union:
        return 0.0
    return len(a & b) / len(union)


def mass_overlap(s_old: set, abs_g_new) -> float:
    values = np.asarray(abs_g_new, dtype=float)
    total = float(np.sum(values))
    if total <= 0:
        return 0.0
    valid = [i for i in s_old if 0 <= i < values.size]
    return float(np.sum(values[valid]) / total)
