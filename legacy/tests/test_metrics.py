import numpy as np

from cadet.metrics import effective_sparsity, jaccard, mass_overlap, normalized_entropy, topk_coverage


def test_zero_gradient_edges():
    g = np.zeros(4)
    assert topk_coverage(g, 2) == 0.0
    assert effective_sparsity(g) == 0.0
    assert normalized_entropy(g) == 0.0
    assert mass_overlap({0, 1}, g) == 0.0


def test_single_point_gradient():
    g = np.array([0.0, 3.0, 0.0, 0.0])
    assert topk_coverage(g, 1) == 1.0
    assert effective_sparsity(g) == 1.0
    assert normalized_entropy(g) == 0.0
    assert mass_overlap({1}, g) == 1.0


def test_uniform_gradient():
    g = np.ones(4)
    assert topk_coverage(g, 2) == 0.5
    assert effective_sparsity(g) == 4.0
    assert normalized_entropy(g) == 1.0


def test_jaccard():
    assert jaccard({1, 2}, {2, 3}) == 1 / 3
    assert jaccard(set(), set()) == 1.0
