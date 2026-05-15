"""
analyze_v4.py -- Publication-quality plots for mlo-eval-v4 results.

Compares STR (sched=0), Adaptive OA (sched=1), MCAB (sched=2)
across loads and window sizes (250ms, 125ms, 100ms).

Usage:
    python3 analyze_v4.py [--csv PATH] [--outdir PATH]

Outputs (saved to outdir):
    fig1_latency_vs_load.png     -- mean latency vs load, all 3 schedulers, w=250ms
    fig2_p99_vs_load.png         -- p99 latency vs load, all 3 schedulers, w=250ms
    fig3_tput_vs_load.png        -- throughput vs load, all 3 schedulers, w=250ms
    fig4_window_comparison.png   -- effect of window size on mean latency (OA + MCAB)
    fig5_p99_window.png          -- effect of window size on p99 (OA + MCAB)
    fig6_co_balance.png          -- avg CO balance (|co0-co1|) vs load per scheduler
"""

import argparse
import os
import sys

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

# ── Style ──────────────────────────────────────────────────────────────────
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
    "lines.markersize": 5,
})

SCHED_LABELS = {0: "MLO-STR", 1: "Adaptive OA", 2: "MCAB"}
SCHED_COLORS = {0: "#e41a1c", 1: "#377eb8", 2: "#4daf4a"}
SCHED_MARKERS = {0: "s", 1: "o", 2: "^"}
WIN_STYLES = {250: "-", 125: "--", 100: ":"}
FIGSIZE_SINGLE = (3.5, 2.8)
FIGSIZE_WIDE   = (6.5, 2.8)


def load_data(csv_path: str) -> pd.DataFrame:
    df = pd.read_csv(csv_path)
    df["sched"]     = df["sched"].astype(int)
    df["window_ms"] = df["window_ms"].astype(int)
    df["co_imbal"]  = (df["avg_co0_ap"] - df["avg_co1_ap"]).abs()
    return df


def agg(df: pd.DataFrame, group_cols: list) -> pd.DataFrame:
    """Mean + 95% CI (1.96 * std / sqrt(n)) across seeds."""
    agg_df = df.groupby(group_cols).agg(
        mean_ms_mean=("mean_ms", "mean"),
        mean_ms_std=("mean_ms", "std"),
        p99_ms_mean=("p99_ms", "mean"),
        p99_ms_std=("p99_ms", "std"),
        tput_mean=("tput_mbps", "mean"),
        tput_std=("tput_mbps", "std"),
        co_imbal_mean=("co_imbal", "mean"),
        co_imbal_std=("co_imbal", "std"),
        n=("mean_ms", "count"),
    ).reset_index()

    for col in ["mean_ms", "p99_ms", "tput", "co_imbal"]:
        agg_df[f"{col}_ci"] = 1.96 * agg_df[f"{col}_std"] / np.sqrt(agg_df["n"])

    return agg_df


def save(fig, path: str):
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {path}")


# ── Plot helpers ────────────────────────────────────────────────────────────

def plot_metric_vs_load(agg_df, window, y_col, y_ci_col, ylabel, title, path):
    fig, ax = plt.subplots(figsize=FIGSIZE_SINGLE)
    sub = agg_df[agg_df["window_ms"] == window]
    for sched in [0, 1, 2]:
        s = sub[sub["sched"] == sched].sort_values("load_mbps")
        ax.errorbar(s["load_mbps"], s[y_col], yerr=s[y_ci_col],
                    label=SCHED_LABELS[sched],
                    color=SCHED_COLORS[sched],
                    marker=SCHED_MARKERS[sched],
                    linestyle="-", capsize=3)
    ax.set_xlabel("Offered Load (Mbps)")
    ax.set_ylabel(ylabel)
    ax.set_title(f"{title} (window={window} ms)")
    ax.legend()
    ax.grid(True, linewidth=0.4, alpha=0.5)
    fig.tight_layout()
    save(fig, path)


def plot_window_effect(agg_df, scheds, y_col, y_ci_col, ylabel, title, path):
    fig, ax = plt.subplots(figsize=FIGSIZE_SINGLE)
    for sched in scheds:
        for window in [250, 125, 100]:
            s = agg_df[(agg_df["sched"] == sched) &
                       (agg_df["window_ms"] == window)].sort_values("load_mbps")
            label = f"{SCHED_LABELS[sched]} w={window}ms"
            ax.errorbar(s["load_mbps"], s[y_col], yerr=s[y_ci_col],
                        label=label,
                        color=SCHED_COLORS[sched],
                        marker=SCHED_MARKERS[sched],
                        linestyle=WIN_STYLES[window],
                        capsize=3)
    ax.set_xlabel("Offered Load (Mbps)")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend(fontsize=7, ncol=2)
    ax.grid(True, linewidth=0.4, alpha=0.5)
    fig.tight_layout()
    save(fig, path)


# ── Main ────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv",    default="results/v4/summary.csv")
    parser.add_argument("--outdir", default="results/v4/plots")
    args = parser.parse_args()

    if not os.path.exists(args.csv):
        sys.exit(f"ERROR: {args.csv} not found. Run the sweep first.")

    os.makedirs(args.outdir, exist_ok=True)

    df     = load_data(args.csv)
    agg_df = agg(df, ["sched", "load_mbps", "window_ms"])

    print(f"Loaded {len(df)} rows, {df['seed'].nunique()} seeds, "
          f"windows={sorted(df['window_ms'].unique())}")

    # Fig 1: mean latency vs load (w=250ms)
    plot_metric_vs_load(agg_df, 250,
                        "mean_ms_mean", "mean_ms_ci",
                        "Mean Latency (ms)", "Mean Latency",
                        f"{args.outdir}/fig1_latency_vs_load.png")

    # Fig 2: p99 latency vs load (w=250ms)
    plot_metric_vs_load(agg_df, 250,
                        "p99_ms_mean", "p99_ms_ci",
                        "P99 Latency (ms)", "P99 Latency",
                        f"{args.outdir}/fig2_p99_vs_load.png")

    # Fig 3: throughput vs load (w=250ms)
    plot_metric_vs_load(agg_df, 250,
                        "tput_mean", "tput_std",
                        "Throughput (Mbps)", "Throughput",
                        f"{args.outdir}/fig3_tput_vs_load.png")

    # Fig 4: window size effect on mean latency (OA + MCAB only)
    plot_window_effect(agg_df, [1, 2],
                       "mean_ms_mean", "mean_ms_ci",
                       "Mean Latency (ms)",
                       "Window Size Effect on Mean Latency",
                       f"{args.outdir}/fig4_window_mean_latency.png")

    # Fig 5: window size effect on p99 (OA + MCAB only)
    plot_window_effect(agg_df, [1, 2],
                       "p99_ms_mean", "p99_ms_ci",
                       "P99 Latency (ms)",
                       "Window Size Effect on P99 Latency",
                       f"{args.outdir}/fig5_window_p99.png")

    # Fig 6: CO imbalance vs load (w=250ms)
    fig, ax = plt.subplots(figsize=FIGSIZE_SINGLE)
    sub = agg_df[agg_df["window_ms"] == 250]
    for sched in [0, 1, 2]:
        s = sub[sub["sched"] == sched].sort_values("load_mbps")
        ax.errorbar(s["load_mbps"], s["co_imbal_mean"], yerr=s["co_imbal_ci"],
                    label=SCHED_LABELS[sched],
                    color=SCHED_COLORS[sched],
                    marker=SCHED_MARKERS[sched],
                    linestyle="-", capsize=3)
    ax.set_xlabel("Offered Load (Mbps)")
    ax.set_ylabel("|CO0 - CO1|")
    ax.set_title("CO Imbalance vs Load (window=250 ms)")
    ax.legend()
    ax.grid(True, linewidth=0.4, alpha=0.5)
    fig.tight_layout()
    save(fig, f"{args.outdir}/fig6_co_imbalance.png")

    print("Done. All figures saved to", args.outdir)


if __name__ == "__main__":
    main()
