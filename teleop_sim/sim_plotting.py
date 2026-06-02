#!/usr/bin/env python3
"""Shared matplotlib helpers for teleop_sim (save PNG under teleop_sim/figures/)."""

from __future__ import annotations

import os

_FIG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "figures")


def figure_dir() -> str:
    os.makedirs(_FIG_DIR, exist_ok=True)
    return _FIG_DIR


def configure_matplotlib():
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams["axes.grid"] = True
    plt.rcParams["figure.figsize"] = (10, 8)
    plt.rcParams["font.size"] = 10
    return plt


def save_current_figure(path: str):
    import matplotlib.pyplot as plt

    fig = plt.gcf()
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
