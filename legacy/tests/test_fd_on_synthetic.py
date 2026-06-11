import numpy as np

from cadet.config import load_config
from cadet.gradients import finite_difference_snapshot
from cadet.groups import build_groups
from cadet.input_model import zero_theta
from cadet.metrics import mass_overlap
from cadet.support import topk_support
from cadet.vehicle.synthetic import SyntheticAdapter


def run_snapshot(tmp_path, noise_fraction):
    config = load_config("configs/synthetic_sanity.yaml")
    config.simulator["synthetic"]["noise_fraction"] = noise_fraction
    groups = build_groups(config.input["horizon_s"], config.input["window_s"], config.input["channels"])
    theta = zero_theta(groups)
    return finite_difference_snapshot(theta, config.scenarios[0], 0, config, tmp_path)


def check_snapshot(snapshot, min_recall, min_overlap):
    for prop, g in snapshot.gradients.items():
        abs_g = np.abs(g)
        true_support = SyntheticAdapter.TRUE_SUPPORT[prop]
        estimated = topk_support(abs_g, 5)
        recall = len(estimated & true_support) / len(true_support)
        overlap = mass_overlap(true_support, abs_g)
        assert recall >= min_recall, (prop, recall)
        assert overlap >= min_overlap, (prop, overlap)


def test_fd_recovers_clean_sparse_support(tmp_path):
    snapshot = run_snapshot(tmp_path / "clean", 0.0)
    check_snapshot(snapshot, 0.85, 0.7)


def test_fd_recovers_noisy_sparse_support(tmp_path):
    snapshot = run_snapshot(tmp_path / "noisy", 0.05)
    check_snapshot(snapshot, 0.5, 0.5)
