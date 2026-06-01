import numpy as np

from sparsepilot.config import load_config
from sparsepilot.groups import build_groups
from sparsepilot.input_model import perturb_group, project_theta, theta_to_sequence, zero_theta


def cfg():
    return load_config("configs/synthetic_sanity.yaml")


def test_zero_theta_is_all_zero():
    config = cfg()
    groups = build_groups(config.input["horizon_s"], config.input["window_s"], config.input["channels"])
    theta = zero_theta(groups)
    assert theta.shape == (40,)
    assert np.all(theta == 0)


def test_perturb_group_only_changes_one_group_near_zero():
    config = cfg()
    groups = build_groups(config.input["horizon_s"], config.input["window_s"], config.input["channels"])
    theta = zero_theta(groups)
    perturbed = perturb_group(theta, 7, 0.08, +1, config)
    changed = np.flatnonzero(np.abs(perturbed - theta) > 1e-12)
    assert changed.tolist() == [7]
    assert perturbed[7] == 0.08


def test_project_bounds_and_window_delta():
    config = cfg()
    theta = np.full(40, 2.0)
    projected = project_theta(theta, config)
    assert np.all(projected <= config.input["max_value"])
    assert np.all(projected >= config.input["min_value"])
    groups = build_groups(config.input["horizon_s"], config.input["window_s"], config.input["channels"])
    for channel in config.input["channels"]:
        vals = [projected[g.group_id] for g in groups if g.channel == channel]
        assert np.all(np.abs(np.diff(vals)) <= config.input["max_delta_per_window"] + 1e-12)


def test_theta_to_sequence_neutral_tail_is_zero():
    config = cfg()
    groups = build_groups(config.input["horizon_s"], config.input["window_s"], config.input["channels"])
    theta = np.ones(40) * 0.2
    sequence = theta_to_sequence(theta, groups, config)
    tail = sequence[sequence["t_s"] >= config.input["horizon_s"]]
    assert not tail.empty
    assert np.allclose(tail[config.input["channels"]].to_numpy(), 0.0)
