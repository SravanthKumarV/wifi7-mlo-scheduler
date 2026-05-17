"""
plot_multistep.py -- CO time-series plots for the multi-flow scenario.

Reads _sched.csv files (time_s, co0_ap, co1_ap, p_tid1),
averages across seeds, and plots CO on both links over time
with vertical markers at each flow start (t=2, 4, 6, 8 s).

Usage:
    python3 plot_multistep.py [--datadir PATH] [--outdir PATH]
"""

import argparse
import glob
import os
import sys

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams.update({
    "figure.dpi": 150,
    "font.family": "serif",
    "font.size": 9,
    "axes.labelsize": 9,
    "axes.titlesize": 10,
    "legend.fontsize": 8,
    "xtick.labelsize": 8,
    "ytick.labelsize": 8,
    "lines.linewidth": 1.5,
})

SCHED_NAMES = {0: "MLO-STR", 1: "Adaptive OA", 2: "MCAB"}
FLOW_STARTS = [2, 4, 6, 8]
SEEDS       = [1, 2, 3]


def load_sched_csvs(datadir: str, sched: int, seeds: list) -> pd.DataFrame:
    """Load and average sched CSVs across seeds, aligned on time grid."""
    frames = []
    for seed in seeds:
        pat = os.path.join(datadir, f"s{sched}_sd{seed}_sched.csv")
        files = glob.glob(pat)
        if not files:
            continue
        df = pd.read_csv(files[0])
        df["seed"] = seed
        frames.append(df)
    if not frames:
        return pd.DataFrame()
    all_df = pd.concat(frames, ignore_index=True)
    # Round time to nearest 0.25s window for alignment across seeds
    all_df["time_r"] = (all_df["time_s"] / 0.25).round() * 0.25
    avg = all_df.groupby("time_r")[["co0_ap", "co1_ap", "p_tid1"]].mean().reset_index()
    avg.rename(columns={"time_r": "time_s"}, inplace=True)
    return avg.sort_values("time_s")


def plot_co_timeseries(datadir: str, outdir: str):
    """3-panel CO time-series, one subplot per scheduler."""
    fig, axes = plt.subplots(1, 3, figsize=(11, 3.4), sharey=True)

    for si, (sched_id, sched_name) in enumerate(SCHED_NAMES.items()):
        ax   = axes[si]
        data = load_sched_csvs(datadir, sched_id, SEEDS)

        if data.empty:
            ax.set_title(f"{sched_name}\n(no data)")
            continue

        ax.plot(data["time_s"], data["co0_ap"],
                color="#d62728", linewidth=1.6, label="Link0 (5GHz)")
        ax.plot(data["time_s"], data["co1_ap"],
                color="#1f77b4", linewidth=1.6, linestyle="--", label="Link1 (6GHz)")

        # Flow start markers
        for i, t in enumerate(FLOW_STARTS):
            ax.axvline(t, color="red", linewidth=0.9, linestyle=":", alpha=0.8)
            ax.text(t + 0.1, ax.get_ylim()[1] * 0.92 if ax.get_ylim()[1] > 0 else 0.22,
                    f"Flow {i+1}", color="red", fontsize=7, rotation=90, va="top")

        ax.set_xlim(left=1.5)
        ax.set_ylim(0, 0.28 if data["co0_ap"].max() < 0.3 else None)
        ax.set_xlabel("Time (s)")
        ax.set_title(sched_name)
        ax.grid(True, linewidth=0.4, alpha=0.5)
        if si == 0:
            ax.set_ylabel("Channel Occupancy")
            ax.legend(loc="upper left", fontsize=7)

    fig.suptitle(
        "CO Variation as Flows Start at t = 2, 4, 6, 8 s  (80 Mbps/flow, window = 250 ms)",
        fontsize=10)
    fig.tight_layout()
    out = os.path.join(outdir, "fig_co_timeseries.png")
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {out}")


def plot_p_timeseries(datadir: str, outdir: str):
    """Split-ratio p_tid1 over time for OA and MCAB."""
    fig, axes = plt.subplots(1, 2, figsize=(8, 3.2), sharey=True)

    for si, (sched_id, sched_name) in enumerate([(1, "Adaptive OA"), (2, "MCAB")]):
        ax   = axes[si]
        data = load_sched_csvs(datadir, sched_id, SEEDS)
        if data.empty:
            ax.set_title(f"{sched_name}\n(no data)")
            continue
        ax.plot(data["time_s"], data["p_tid1"],
                color="#2ca02c", linewidth=1.6)
        for i, t in enumerate(FLOW_STARTS):
            ax.axvline(t, color="red", linewidth=0.9, linestyle=":", alpha=0.8)
            ax.text(t + 0.1, 0.92, f"Flow {i+1}", color="red",
                    fontsize=7, rotation=90, va="top",
                    transform=ax.get_xaxis_transform())
        ax.set_xlim(left=1.5)
        ax.set_ylim(0, 1.05)
        ax.set_xlabel("Time (s)")
        ax.set_title(f"{sched_name} — p_tid1 (fraction to Link1)")
        ax.grid(True, linewidth=0.4, alpha=0.5)
        if si == 0:
            ax.set_ylabel("p_tid1")

    fig.suptitle("Traffic Split Ratio Over Time", fontsize=10)
    fig.tight_layout()
    out = os.path.join(outdir, "fig_p_timeseries.png")
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {out}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--datadir",
                        default="/home/mynavajha/ns-3-dev/scratch/mlo_results_multistep")
    parser.add_argument("--outdir",
                        default="/home/mynavajha/wifi7-mlo-scheduler/results/multistep/plots")
    args = parser.parse_args()

    if not os.path.isdir(args.datadir):
        sys.exit(f"ERROR: {args.datadir} not found. Run the simulations first.")

    os.makedirs(args.outdir, exist_ok=True)

    plot_co_timeseries(args.datadir, args.outdir)
    plot_p_timeseries(args.datadir, args.outdir)

    print("Done.")


if __name__ == "__main__":
    main()
