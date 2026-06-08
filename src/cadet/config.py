from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping

import yaml


@dataclass(frozen=True)
class ScenarioCfg:
    id: str
    platform: str
    perturb_mode: str
    observe_mode: str
    takeoff_alt_m: float
    properties: list[str]
    t_switch_s: float | None = None
    param_overrides: Mapping[str, float] = field(default_factory=dict)


@dataclass(frozen=True)
class ExperimentConfig:
    path: Path
    experiment_id: str
    input: dict[str, Any]
    properties: dict[str, dict[str, Any]]
    scenarios: list[ScenarioCfg]
    seeds: list[int]
    persistence_path: dict[str, Any]
    simulator: dict[str, Any]
    logging: dict[str, Any]

    def scenario_by_id(self, scenario_id: str) -> ScenarioCfg:
        for scenario in self.scenarios:
            if scenario.id == scenario_id:
                return scenario
        raise KeyError(f"Unknown scenario_id: {scenario_id}")


def load_config(path: str | Path) -> ExperimentConfig:
    cfg_path = Path(path)
    with cfg_path.open("r", encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    scenarios = [ScenarioCfg(**item) for item in raw["scenarios"]]
    return ExperimentConfig(
        path=cfg_path,
        experiment_id=raw["experiment_id"],
        input=raw["input"],
        properties=raw["properties"],
        scenarios=scenarios,
        seeds=list(raw["seeds"]),
        persistence_path=raw.get("persistence_path", {}),
        simulator=raw.get("simulator", {}),
        logging=raw.get("logging", {}),
    )
