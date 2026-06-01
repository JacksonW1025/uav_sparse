from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


def plot_gradient_heatmap(gradient_csv, output_png) -> None:
    df = pd.read_csv(gradient_csv)
    value_col = "abs_g" if "abs_g" in df.columns else "abs_mean_g"
    pivot = df.pivot(index="channel", columns="window_id", values=value_col)
    order = [c for c in ["roll", "pitch", "yaw", "throttle"] if c in pivot.index]
    pivot = pivot.loc[order]
    fig, ax = plt.subplots(figsize=(8, 3))
    im = ax.imshow(pivot.values, aspect="auto")
    ax.set_xticks(range(len(pivot.columns)), labels=pivot.columns)
    ax.set_yticks(range(len(pivot.index)), labels=pivot.index)
    ax.set_xlabel("window_id")
    ax.set_ylabel("channel")
    fig.colorbar(im, ax=ax, label="abs_g")
    Path(output_png).parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(output_png)
    plt.close(fig)


def plot_topk_coverage_curve(coverage_table, output_png):
    df = pd.read_csv(coverage_table) if not isinstance(coverage_table, pd.DataFrame) else coverage_table
    fig, ax = plt.subplots(figsize=(8, 4))
    for label, part in df.groupby("series"):
        style = {"linewidth": 2.5} if label == "primary_median" else {"linewidth": 1.0, "alpha": 0.45}
        ax.plot(part["k"], part["coverage"], label=label, **style)
    ax.axhline(0.5, color="black", linestyle="--", linewidth=0.8, alpha=0.5)
    ax.axhline(0.75, color="black", linestyle=":", linewidth=0.8, alpha=0.5)
    ax.axvline(4, color="gray", linestyle="--", linewidth=0.8, alpha=0.5)
    ax.axvline(8, color="gray", linestyle=":", linewidth=0.8, alpha=0.5)
    ax.set_xlabel("top k groups")
    ax.set_ylabel("coverage")
    ax.set_ylim(0.0, 1.02)
    ax.grid(True, axis="y", alpha=0.25)
    ax.legend(fontsize=7, ncol=2)
    Path(output_png).parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(output_png)
    plt.close(fig)


def plot_jaccard_grid(jaccard_table, output_png):
    df = pd.read_csv(jaccard_table) if not isinstance(jaccard_table, pd.DataFrame) else jaccard_table
    labels = [
        f"{row.comparison}\n{row.property_a}->{row.property_b}\ntop{int(row.k)}"
        for row in df.itertuples(index=False)
    ]
    x = range(len(df))
    width = 0.36
    fig, ax = plt.subplots(figsize=(max(8, len(df) * 0.7), 4))
    ax.bar([i - width / 2 for i in x], df["jaccard"], width=width, label="jaccard")
    ax.bar([i + width / 2 for i in x], df["mass_overlap_mean"], width=width, label="mass overlap")
    ax.set_xticks(list(x), labels=labels, rotation=45, ha="right")
    ax.set_ylim(0.0, 1.02)
    ax.set_ylabel("score")
    ax.grid(True, axis="y", alpha=0.25)
    ax.legend()
    Path(output_png).parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(output_png)
    plt.close(fig)


def plot_mass_overlap_curve(persistence_table, output_png):
    raise NotImplementedError("Phase 4 analysis plot")
