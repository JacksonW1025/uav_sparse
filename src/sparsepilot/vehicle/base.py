from __future__ import annotations

from abc import ABC, abstractmethod
from pathlib import Path

import pandas as pd

from sparsepilot.config import ScenarioCfg


class VehicleAdapter(ABC):
    @abstractmethod
    def prepare(self, scenario: ScenarioCfg, seed: int) -> None:
        """Start or reset simulator and wait for steady hover."""

    @abstractmethod
    def run(self, input_sequence: pd.DataFrame, scenario: ScenarioCfg, output_dir: Path) -> Path:
        """Execute input_sequence and return raw log path."""

    @abstractmethod
    def parse_log(self, raw_log_path: Path) -> pd.DataFrame:
        """Convert raw log into the unified schema."""

    @abstractmethod
    def shutdown(self) -> None:
        """Clean up any child processes."""
