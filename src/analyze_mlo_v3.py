#!/usr/bin/env python3
"""
analyze_mlo_v3.py  --  MLO scheduler evaluation analysis (v4, CO-equalizing OA).

Figures produced:
  Fig 1  Calibration: co0 and co1 vs UDP load under Adaptive OA (equalization).
  Fig 2  CO time-series at each op-point: both schedulers (2x4 panel).
  Fig 3  Latency time-series: mean, P50, P90, P99 at each op-point (2x4 panel).
  Fig 4  Scheduler probability (p_tid1) over time -- Adaptive OA (1x4 panel).
  Fig 5  Summary bars: mean, P50, P90, P99 latency + throughput, both schedulers.
  Fig 6  CO overlay: both schedulers per op-point (1x4 panel).
  Fig 7  Latency CDFs per op-point, both schedulers (1x4 panel).
  Fig 8  Boxplots of raw latency per op-point, both schedulers (1x4 panel).

Usage:
    python3 analyze_mlo_v3.py \\
        --calib-dir  scratch/mlo_results_v4/calibration \\
        --comp-dir   scratch/mlo_results_v4/comparative \\
        --plots-dir  scratch/mlo_results_v4/plots_v4 \\
        --op-loads   "80 310 390 450"
"""

import argparse
import glob
import os
import warnings

import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
import numpy as np
import pandas as pd
from scipy import stats as scipy_stats

warnings.filterwarnings("ignore")


def bootstrap_ci(data, stat_fn=np.percentile, stat_args=(99,),
                 n_boot=2000, ci=0.95, rng=None):
    """Return (lo, hi) bootstrap CI for stat_fn(data, *stat_args)."""
    if rng is None:
        rng = np.random.default_rng(42)
    data = np.asarray(data)
    if len(data) < 2:
        point = stat_fn(data, *stat_args)
        return point, point
    boots = [stat_fn(rng.choice(data, len(data), replace=True), *stat_args)
             for _ in range(n_boot)]
    alpha = (1 - ci) / 2
    return float(np.percentile(boots, alpha * 100)), float(np.percentile(boots, (1 - alpha) * 100))


def mann_whitney_p99(comp_dir, load, seeds):
    """
    Return Mann-Whitney U p-value comparing per-packet latency
    of STR (sched=0) vs Adaptive (sched=1) at the given load.
    """
    samples = {0: [], 1: []}
    for sched in [0, 1]:
        for seed in seeds:
            df = load_samples_csv(comp_dir, sched, load, seed, max_rows=100000)
            if df is not None and "latency_ms" in df.columns:
                samples[sched].extend(df["latency_ms"].tolist())
    if not samples[0] or not samples[1]:
        return np.nan
    _, p = scipy_stats.mannwhitneyu(samples[0], samples[1], alternative="two-sided")
    return p


def setup_style():
    mpl.rcParams.update({
        "font.family":        "DejaVu Sans",
        "font.size":          9,
        "axes.titlesize":     9,
        "axes.labelsize":     9,
        "xtick.labelsize":    8,
        "ytick.labelsize":    8,
        "legend.fontsize":    8,
        "legend.framealpha":  0.85,
        "axes.grid":          True,
        "grid.alpha":         0.3,
        "grid.linewidth":     0.5,
        "axes.spines.top":    False,
        "axes.spines.right":  False,
        "lines.linewidth":    1.6,
        "figure.dpi":         150,
        "savefig.dpi":        300,
        "savefig.bbox":       "tight",
        "savefig.pad_inches": 0.05,
    })


C = {
    "STR":      "#2166ac",
    "Adaptive": "#d6604d",
    "link0":    "#e41a1c",
    "link1":    "#377eb8",
}
SCHED_NAME = {0: "MLO-STR", 1: "Adaptive (OA)"}
SCHED_COLOR = {0: C["STR"], 1: C["Adaptive"]}

NEW_COLS = ["sched", "load_mbps", "seed", "n_pkts", "mean_ms", "p50_ms",
            "p90_ms", "p99_ms", "p999_ms", "max_ms", "tput_mbps",
            "avg_co0_ap", "avg_co1_ap"]


# ── Loaders ───────────────────────────────────────────────────────────────────
def load_summary(path):
    rows = []
    with open(path) as f:
        for line in f:
            parts = line.strip().split(",")
            if len(parts) == 13 and parts[0] != "sched":
                rows.append(parts)
    if not rows:
        return None
    df = pd.DataFrame(rows, columns=NEW_COLS)
    for col in df.columns:
        df[col] = pd.to_numeric(df[col], errors="coerce")
    return df.dropna(subset=["sched", "load_mbps", "seed"])


def load_sched_csv(directory, sched, load, seed):
    pat = os.path.join(directory, f"s{sched}_l{int(load)}_sd{seed}_sched.csv")
    files = glob.glob(pat)
    if not files:
        return None
    try:
        return pd.read_csv(files[0])
    except Exception:
        return None


def load_lat_csv(directory, sched, load, seed):
    pat = os.path.join(directory, f"s{sched}_l{int(load)}_sd{seed}_lat.csv")
    files = glob.glob(pat)
    if not files:
        return None
    try:
        return pd.read_csv(files[0])
    except Exception:
        return None


def load_samples_csv(directory, sched, load, seed, max_rows=50000):
    pat = os.path.join(directory, f"s{sched}_l{int(load)}_sd{seed}_samples.csv")
    files = glob.glob(pat)
    if not files:
        return None
    try:
        df = pd.read_csv(files[0])
        if len(df) > max_rows:
            df = df.sample(max_rows, random_state=42)
        return df
    except Exception:
        return None


# ── Operating points ──────────────────────────────────────────────────────────
def find_operating_points(calib_df, targets=(0.50, 0.75, 0.82, 0.88),
                          op_loads_override=None,
                          op_target_labels=None):
    """
    Returns dict keyed by *target* CO fraction (used as label throughout).
    op_loads_override : list of loads to use as operating points.
    op_target_labels  : parallel list of target CO fractions (e.g. [0.50, 0.75,
                        0.82, 0.88, 0.95, 0.99]).  Required when override loads
                        would map to the same auto-computed CO (saturation zone).
    """
    df = calib_df[calib_df["sched"] == 1].copy()
    df["avg_co"] = (df["avg_co0_ap"] + df["avg_co1_ap"]) / 2
    df = df.sort_values("load_mbps").drop_duplicates("load_mbps")

    if op_loads_override:
        sorted_loads = sorted(op_loads_override)
        if op_target_labels is None:
            # Auto-assign labels from nearest CO; use load as tiebreak
            seen = set()
            labels = []
            for load in sorted_loads:
                row = df.iloc[(df["load_mbps"] - load).abs().argsort()[:1]]
                avg = round((float(row["avg_co0_ap"].values[0]) +
                             float(row["avg_co1_ap"].values[0])) / 2, 2)
                while avg in seen:
                    avg = round(avg + 0.001, 3)
                seen.add(avg)
                labels.append(avg)
        else:
            labels = list(op_target_labels)

        ops = {}
        for load, label in zip(sorted_loads, labels):
            row = df.iloc[(df["load_mbps"] - load).abs().argsort()[:1]]
            co0 = float(row["avg_co0_ap"].values[0])
            co1 = float(row["avg_co1_ap"].values[0])
            avg = (co0 + co1) / 2
            ops[label] = {
                "load": float(row["load_mbps"].values[0]),
                "co0": co0, "co1": co1, "avg": avg
            }
        return ops

    ops = {}
    for t in targets:
        row = df.iloc[(df["avg_co"] - t).abs().argsort()[:1]]
        load = float(row["load_mbps"].values[0])
        co0  = float(row["avg_co0_ap"].values[0])
        co1  = float(row["avg_co1_ap"].values[0])
        avg  = (co0 + co1) / 2
        ops[t] = {"load": load, "co0": co0, "co1": co1, "avg": avg}
    return ops


# ── Fig 1: Calibration ────────────────────────────────────────────────────────
def fig_calibration(calib_df, ops, plots_dir):
    df = calib_df[calib_df["sched"] == 1].sort_values("load_mbps").drop_duplicates("load_mbps")
    if df.empty:
        print("  [SKIP] fig1"); return

    # Protocol ceiling: average CO at loads ≥ 450 Mbps
    ceiling_co = df[df["load_mbps"] >= 450]["avg_co0_ap"].mean() * 100

    fig, ax = plt.subplots(figsize=(6.5, 3.8))
    ax.plot(df["load_mbps"], df["avg_co0_ap"] * 100,
            color=C["link0"], lw=1.8, marker="o", ms=3, label="Link0 (5 GHz, 40 MHz)")
    ax.plot(df["load_mbps"], df["avg_co1_ap"] * 100,
            color=C["link1"], lw=1.8, marker="s", ms=3, label="Link1 (6 GHz, 80 MHz)")

    # Protocol ceiling line
    ax.axhline(ceiling_co, color="0.45", lw=1.0, ls="-.", alpha=0.8)
    ax.annotate(f"Protocol ceiling ≈{ceiling_co:.0f}%\n(ACK/SIFS/DIFS overhead)",
                xy=(df["load_mbps"].min() + 20, ceiling_co),
                xytext=(4, -18), textcoords="offset points",
                fontsize=6.5, color="0.4", style="italic")

    # 95% and 99% unachievable target lines
    for pct_tgt in (95, 99):
        ax.axhline(pct_tgt, color="crimson", lw=0.9, ls=":", alpha=0.6)
        ax.annotate(f"{pct_tgt}% target\n(unachievable)",
                    xy=(df["load_mbps"].max(), pct_tgt),
                    xytext=(-70, 3), textcoords="offset points",
                    fontsize=6, color="crimson", alpha=0.8)

    # Operating point verticals
    for pct, info in sorted(ops.items()):
        ax.axvline(info["load"], color="0.4", lw=0.8, ls="--", alpha=0.7)
        achieved = info["avg"] * 100
        label = f"~{int(pct*100)}% target\n@{int(info['load'])}M\n(actual {achieved:.0f}%)"
        ax.annotate(label,
                    xy=(info["load"], achieved),
                    xytext=(5, 4), textcoords="offset points",
                    fontsize=6, color="0.3")

    ax.set_xlabel("Offered UDP Load (Mbps)")
    ax.set_ylabel("Channel Occupancy -- AP view (%)")
    ax.set_title("Calibration: Adaptive OA (CO-equalizing, no fixed BG)")
    ax.set_ylim(0, 108)
    ax.yaxis.set_major_formatter(mticker.PercentFormatter(xmax=100))
    ax.legend(loc="upper left")

    out = os.path.join(plots_dir, "fig1_calibration.png")
    fig.savefig(out); plt.close(fig)
    print(f"  [OK] {out}")


# ── Fig 2: CO time-series ─────────────────────────────────────────────────────
def fig_co_timeseries(comp_dir, ops, plots_dir, seed=1):
    op_items = sorted(ops.items())
    n_ops = len(op_items)
    fig, axes = plt.subplots(2, n_ops, figsize=(2.8 * n_ops, 4.8),
                             sharex=False, sharey=True)

    row_labels = ["MLO-STR (Greedy)", "Adaptive (OA)"]
    for col, (pct, info) in enumerate(op_items):
        load = info["load"]
        achieved = info["avg"] * 100
        for row, sched in enumerate([0, 1]):
            ax = axes[row][col]
            df = load_sched_csv(comp_dir, sched, load, seed)
            if df is None:
                ax.text(0.5, 0.5, "no data", ha="center", va="center",
                        transform=ax.transAxes)
            else:
                co0 = "co0_ap" if "co0_ap" in df.columns else "co0"
                co1 = "co1_ap" if "co1_ap" in df.columns else "co1"
                ax.plot(df["time_s"], df[co0] * 100, color=C["link0"], lw=1.4, label="Link0")
                ax.plot(df["time_s"], df[co1] * 100, color=C["link1"], lw=1.4, label="Link1")
                ax.axhline(pct * 100, color="0.5", lw=0.8, ls="--")

            ax.set_ylim(0, 105)
            ax.yaxis.set_major_formatter(mticker.PercentFormatter(xmax=100))
            ax.set_xlabel("Time (s)" if row == 1 else "")
            if col == 0:
                ax.set_ylabel("CO -- AP view (%)")
                ax.annotate(row_labels[row], xy=(-0.38, 0.5),
                            xycoords="axes fraction", rotation=90,
                            va="center", fontsize=8, fontweight="bold")
            # Show target and actual CO in title
            suffix = f"\n(actual {achieved:.0f}%)" if pct >= 0.90 else ""
            ax.set_title(f"~{int(pct*100)}% target\n({int(load)} Mbps){suffix}", fontsize=7)
            if row == 0 and col == n_ops - 1:
                ax.legend(loc="upper right", fontsize=7)

    fig.suptitle("Channel Occupancy Evolution (AP perspective)", fontsize=9, y=1.01)
    fig.tight_layout()
    out = os.path.join(plots_dir, "fig2_co_timeseries.png")
    fig.savefig(out); plt.close(fig)
    print(f"  [OK] {out}")


# ── Fig 3: Latency time-series (mean, P50, P90, P99) ─────────────────────────
def fig_latency_timeseries(comp_dir, ops, plots_dir, seed=1):
    op_items = sorted(ops.items())
    n_ops = len(op_items)
    fig, axes = plt.subplots(2, n_ops, figsize=(2.8 * n_ops, 4.8),
                             sharex=False, sharey=False)

    row_labels = ["MLO-STR (Greedy)", "Adaptive (OA)"]
    lat_styles = [
        ("mean_ms", "Mean",  "steelblue",  "-",  1.4),
        ("p50_ms",  "P50",   "seagreen",   "-",  1.0),
        ("p90_ms",  "P90",   "orange",     "--", 1.0),
        ("p99_ms",  "P99",   "crimson",    "--", 1.0),
    ]

    for col, (pct, info) in enumerate(op_items):
        load = info["load"]
        for row, sched in enumerate([0, 1]):
            ax = axes[row][col]
            df = load_lat_csv(comp_dir, sched, load, seed)
            if df is None or len(df) < 2:
                ax.text(0.5, 0.5, "no data", ha="center", va="center",
                        transform=ax.transAxes)
            else:
                for col_name, label, color, ls, lw in lat_styles:
                    if col_name in df.columns:
                        ax.plot(df["time_s"], df[col_name],
                                color=color, lw=lw, ls=ls, label=label)
                ax.set_ylim(bottom=0)

            ax.set_xlabel("Time (s)" if row == 1 else "")
            if col == 0:
                ax.set_ylabel("Latency (ms)")
                ax.annotate(row_labels[row], xy=(-0.38, 0.5),
                            xycoords="axes fraction", rotation=90,
                            va="center", fontsize=8, fontweight="bold")
            ax.set_title(f"~{int(pct*100)}% target\n({int(load)} Mbps)", fontsize=8)
            if row == 0 and col == n_ops - 1:
                ax.legend(loc="upper right", fontsize=6)

    fig.suptitle("E2E Latency (Mean / P50 / P90 / P99)", fontsize=9, y=1.01)
    fig.tight_layout()
    out = os.path.join(plots_dir, "fig3_latency_timeseries.png")
    fig.savefig(out); plt.close(fig)
    print(f"  [OK] {out}")


# ── Fig 4: Scheduler probability over time ────────────────────────────────────
def fig_p_tid1(comp_dir, ops, plots_dir, seed=1):
    op_items = sorted(ops.items())
    n_ops = len(op_items)
    fig, axes = plt.subplots(1, n_ops, figsize=(2.6 * n_ops, 2.8), sharey=True)

    for col, (pct, info) in enumerate(op_items):
        load = info["load"]
        ax = axes[col]

        # Adaptive scheduler probability
        df1 = load_sched_csv(comp_dir, 1, load, seed)
        if df1 is not None and "p_tid1" in df1.columns:
            ax.plot(df1["time_s"], df1["p_tid1"],
                    color=C["Adaptive"], lw=1.4, label="Adaptive (OA)")

        # Greedy: constant p=1.0
        df0 = load_sched_csv(comp_dir, 0, load, seed)
        if df0 is not None:
            ax.axhline(1.0, color=C["STR"], lw=1.2, ls="--", label="MLO-STR (fixed 1.0)")

        ax.set_ylim(0, 1.1)
        ax.set_xlabel("Time (s)")
        ax.set_title(f"~{int(pct*100)}% target\n({int(load)} Mbps)", fontsize=8)
        if col == 0:
            ax.set_ylabel("p_tid1 (fraction to Link1 via TID3)")
        if col == n_ops - 1:
            ax.legend(fontsize=7)

    fig.suptitle("Scheduler Probability (p_tid1) Over Time", fontsize=9)
    fig.tight_layout()
    out = os.path.join(plots_dir, "fig4_p_tid1_timeseries.png")
    fig.savefig(out); plt.close(fig)
    print(f"  [OK] {out}")


# ── Fig 5: Paper-style latency percentile bars (like Fig 1b / Fig 2b) ────────
def fig_summary_bars(comp_df, comp_dir, ops, plots_dir, seeds=(1, 2, 3)):
    """
    One subplot per CO operating point.
    X-axis: percentile groups (90th, 95th, 99th, 100th).
    Y-axis: E2E latency (ms), log scale.
    Two bars per group: MLO-STR (blue) vs Adaptive OA (red).
    P99 bars carry 95% bootstrap CI error bars.
    Matches style of Fig 1(b)/2(b) in Kulshrestha et al. IFIP 2024.
    """
    op_items  = sorted(ops.items())
    n_ops     = len(op_items)
    pct_names = ["90th", "95th", "99th", "100th"]
    x         = np.arange(len(pct_names))
    w         = 0.32

    op_loads = [info["load"] for _, info in op_items]

    # p90, p99, max from summary (mean over seeds)
    grp_sum = comp_df[comp_df["load_mbps"].isin(op_loads)].groupby(
        ["sched", "load_mbps"])[["p90_ms", "p99_ms", "max_ms"]].mean().reset_index()

    # p95 + bootstrap CI on p99 from pooled samples across all seeds
    stats_records = []
    for sched in [0, 1]:
        for load in op_loads:
            vals = []
            for seed in seeds:
                df = load_samples_csv(comp_dir, sched, load, seed, max_rows=100000)
                if df is not None and "latency_ms" in df.columns:
                    vals.extend(df["latency_ms"].tolist())
            if vals:
                p95_val = float(np.percentile(vals, 95))
                ci_lo, ci_hi = bootstrap_ci(vals, np.percentile, (99,))
                stats_records.append({
                    "sched": sched, "load_mbps": load,
                    "p95_ms": p95_val,
                    "p99_ci_lo": ci_lo, "p99_ci_hi": ci_hi,
                })
    stats_df = pd.DataFrame(stats_records) if stats_records else None

    fig, axes = plt.subplots(1, n_ops, figsize=(2.8 * n_ops, 3.8), sharey=False)
    if n_ops == 1:
        axes = [axes]

    for col, (pct, info) in enumerate(op_items):
        load = info["load"]
        ax   = axes[col]

        for i, (sched, name) in enumerate([(0, "MLO-STR"), (1, "Adaptive (OA)")]):
            row_sum = grp_sum[(grp_sum["sched"] == sched) & (grp_sum["load_mbps"] == load)]
            if row_sum.empty:
                continue

            p90  = float(row_sum["p90_ms"].values[0])
            p99  = float(row_sum["p99_ms"].values[0])
            p100 = float(row_sum["max_ms"].values[0])

            p95 = np.nan
            p99_err_lo = p99_err_hi = 0.0
            if stats_df is not None and not stats_df.empty:
                rs = stats_df[(stats_df["sched"] == sched) & (stats_df["load_mbps"] == load)]
                if not rs.empty:
                    p95 = float(rs["p95_ms"].values[0])
                    p99_err_lo = max(0.0, p99 - float(rs["p99_ci_lo"].values[0]))
                    p99_err_hi = max(0.0, float(rs["p99_ci_hi"].values[0]) - p99)

            heights = [p90, p95, p99, p100]
            offset  = (i - 0.5) * w
            ax.bar(x + offset, heights, w,
                   color=SCHED_COLOR[sched], alpha=0.85,
                   label=name, zorder=3)
            # P99 error bar (index 2)
            ax.errorbar(x[2] + offset, p99,
                        yerr=[[p99_err_lo], [p99_err_hi]],
                        fmt="none", color="black", capsize=3, lw=1.2, zorder=4)

        ax.set_yscale("log")
        ax.yaxis.set_major_formatter(mticker.FuncFormatter(
            lambda v, _: f"{v:g}"))
        ax.set_xticks(x)
        ax.set_xticklabels(pct_names, fontsize=8)
        ax.set_xlabel("Percentiles", fontsize=8)
        if col == 0:
            ax.set_ylabel("E2E Latency (ms)", fontsize=9)
        ax.set_title(f"~{int(pct*100)}% CO\n({int(load)} Mbps)", fontsize=8)
        ax.grid(True, which="both", axis="y", alpha=0.3, lw=0.5)
        ax.grid(False, axis="x")
        ax.set_axisbelow(True)
        if col == 0:
            ax.legend(fontsize=7, loc="upper left")

    fig.suptitle("E2E Latency at CO Operating Points -- MLO-STR vs Adaptive OA",
                 fontsize=10)
    fig.tight_layout()
    out = os.path.join(plots_dir, "fig5_latency_percentiles.png")
    fig.savefig(out); plt.close(fig)
    print(f"  [OK] {out}")


def print_mannwhitney(comp_dir, ops, seeds):
    """Print Mann-Whitney U p-values for each operating point."""
    print("\nMann-Whitney U test (per-packet latency, STR vs Adaptive OA):")
    print(f"  {'Load (Mbps)':<14} {'Target CO':<12} {'p-value':<12} {'Significant'}")
    for pct, info in sorted(ops.items()):
        load = info["load"]
        p = mann_whitney_p99(comp_dir, load, seeds)
        sig = "YES (p<0.05)" if (not np.isnan(p) and p < 0.05) else "no"
        p_str = f"{p:.3e}" if not np.isnan(p) else "N/A"
        print(f"  {int(load):<14} {int(pct*100):>3}%         {p_str:<12} {sig}")


# ── Fig 6: CO overlay ─────────────────────────────────────────────────────────
def fig_co_overlay(comp_dir, ops, plots_dir, seed=1):
    op_items = sorted(ops.items())
    n_ops = len(op_items)
    fig, axes = plt.subplots(1, n_ops, figsize=(2.6 * n_ops, 2.8), sharey=True)

    for col, (pct, info) in enumerate(op_items):
        load = info["load"]
        ax = axes[col]
        for sched, ls_style, name in [(0, "--", "STR"), (1, "-", "OA")]:
            df = load_sched_csv(comp_dir, sched, load, seed)
            if df is None:
                continue
            co0 = "co0_ap" if "co0_ap" in df.columns else "co0"
            co1 = "co1_ap" if "co1_ap" in df.columns else "co1"
            ax.plot(df["time_s"], df[co0] * 100, color=C["link0"],
                    lw=1.3, ls=ls_style, label=f"Link0 ({name})")
            ax.plot(df["time_s"], df[co1] * 100, color=C["link1"],
                    lw=1.3, ls=ls_style, label=f"Link1 ({name})")
        ax.axhline(pct * 100, color="0.5", lw=0.7, ls=":", label=f"Target {int(pct*100)}%")
        ax.set_title(f"~{int(pct*100)}%  ({int(load)} Mbps)", fontsize=8)
        ax.set_xlabel("Time (s)")
        ax.set_ylim(0, 105)
        ax.yaxis.set_major_formatter(mticker.PercentFormatter(xmax=100))

    axes[0].set_ylabel("CO -- AP view (%)")
    handles, labels = axes[-1].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=5,
               fontsize=7, bbox_to_anchor=(0.5, -0.15))
    fig.suptitle("CO: MLO-STR vs Adaptive OA at each operating point", fontsize=9)
    fig.tight_layout()
    out = os.path.join(plots_dir, "fig6_co_comparison.png")
    fig.savefig(out, bbox_inches="tight"); plt.close(fig)
    print(f"  [OK] {out}")


# ── Fig 7: Latency CDFs ───────────────────────────────────────────────────────
def fig_latency_cdf(comp_dir, ops, plots_dir, seeds=(1, 2, 3)):
    op_items = sorted(ops.items())
    n_ops = len(op_items)
    fig, axes = plt.subplots(1, n_ops, figsize=(2.6 * n_ops, 3.0), sharey=True)

    for col, (pct, info) in enumerate(op_items):
        load = info["load"]
        ax = axes[col]

        for sched in [0, 1]:
            all_samples = []
            for seed in seeds:
                df = load_samples_csv(comp_dir, sched, load, seed, max_rows=20000)
                if df is not None and "latency_ms" in df.columns:
                    all_samples.append(df["latency_ms"].values)
            if not all_samples:
                continue
            samples = np.concatenate(all_samples)
            sorted_s = np.sort(samples)
            cdf = np.arange(1, len(sorted_s) + 1) / len(sorted_s)
            ax.plot(sorted_s, cdf * 100,
                    color=SCHED_COLOR[sched], lw=1.5,
                    label=SCHED_NAME[sched])

        # Mark key percentiles
        for pct_line, ls in [(50, ":"), (90, "--"), (99, "-.")]:
            ax.axhline(pct_line, color="0.6", lw=0.7, ls=ls, alpha=0.7)
            ax.annotate(f"P{pct_line}", xy=(ax.get_xlim()[1] if col > 0 else 0, pct_line),
                        xytext=(2, 1), textcoords="offset points",
                        fontsize=6, color="0.5")

        ax.set_xlabel("Latency (ms)")
        ax.set_title(f"~{int(pct*100)}% CO\n({int(load)} Mbps)", fontsize=8)
        if col == 0:
            ax.set_ylabel("CDF (%)")
            ax.yaxis.set_major_formatter(mticker.PercentFormatter(xmax=100))
        ax.set_ylim(0, 101)
        ax.set_xlim(left=0)
        if col == n_ops - 1:
            ax.legend(fontsize=7)

    fig.suptitle("Latency CDFs -- MLO-STR vs Adaptive OA", fontsize=9)
    fig.tight_layout()
    out = os.path.join(plots_dir, "fig7_latency_cdf.png")
    fig.savefig(out); plt.close(fig)
    print(f"  [OK] {out}")


# ── Fig 8: Boxplots ───────────────────────────────────────────────────────────
def fig_boxplots(comp_dir, ops, plots_dir, seeds=(1, 2, 3)):
    op_items = sorted(ops.items())
    n_ops = len(op_items)
    fig, axes = plt.subplots(1, n_ops, figsize=(2.6 * n_ops, 3.5), sharey=False)

    for col, (pct, info) in enumerate(op_items):
        load = info["load"]
        ax = axes[col]

        box_data = []
        box_labels = []
        box_colors = []

        for sched in [0, 1]:
            all_samples = []
            for seed in seeds:
                df = load_samples_csv(comp_dir, sched, load, seed, max_rows=30000)
                if df is not None and "latency_ms" in df.columns:
                    all_samples.append(df["latency_ms"].values)
            if all_samples:
                box_data.append(np.concatenate(all_samples))
                box_labels.append(SCHED_NAME[sched])
                box_colors.append(SCHED_COLOR[sched])

        if box_data:
            bp = ax.boxplot(box_data,
                            patch_artist=True,
                            notch=False,
                            showfliers=True,
                            flierprops=dict(marker=".", markersize=1.5,
                                            alpha=0.3, linestyle="none"),
                            medianprops=dict(color="white", lw=1.5),
                            whiskerprops=dict(lw=1.0),
                            capprops=dict(lw=1.0),
                            widths=0.5)
            for patch, color in zip(bp["boxes"], box_colors):
                patch.set_facecolor(color)
                patch.set_alpha(0.75)
            for flier, color in zip(bp["fliers"], box_colors):
                flier.set(markerfacecolor=color, markeredgecolor=color)

        ax.set_xticks([1, 2])
        ax.set_xticklabels(["MLO-STR", "Adaptive\n(OA)"], fontsize=7)
        ax.set_title(f"~{int(pct*100)}% CO\n({int(load)} Mbps)", fontsize=8)
        if col == 0:
            ax.set_ylabel("E2E Latency (ms)")
        ax.set_ylim(bottom=0)

    fig.suptitle("Latency Distribution Boxplots -- MLO-STR vs Adaptive OA", fontsize=9)
    fig.tight_layout()
    out = os.path.join(plots_dir, "fig8_boxplots.png")
    fig.savefig(out); plt.close(fig)
    print(f"  [OK] {out}")


# ── Summary table ─────────────────────────────────────────────────────────────
def print_summary(comp_df, ops):
    op_loads = sorted([info["load"] for info in ops.values()])
    op_pcts  = {info["load"]: pct for pct, info in ops.items()}

    grp = comp_df[comp_df["load_mbps"].isin(op_loads)].groupby(
        ["sched", "load_mbps"])[
        ["mean_ms", "p50_ms", "p90_ms", "p99_ms",
         "tput_mbps", "avg_co0_ap", "avg_co1_ap"]].mean().reset_index()

    print("\nSUMMARY (averaged over seeds):")
    hdr = f"  {'Load':>6}  {'Sched':<16}  {'co0':>5}  {'co1':>5}  " \
          f"{'Mean':>6}  {'P50':>6}  {'P90':>6}  {'P99':>6}  {'Tput':>7}"
    print(hdr)
    print("  " + "-" * (len(hdr) - 2))
    for _, r in grp.sort_values(["load_mbps", "sched"]).iterrows():
        if r["load_mbps"] not in op_loads:
            continue
        name = SCHED_NAME[int(r["sched"])]
        print(f"  {int(r['load_mbps']):>6}  {name:<16}  {r['avg_co0_ap']:.3f}  "
              f"{r['avg_co1_ap']:.3f}  "
              f"{r['mean_ms']:>6.2f}  {r['p50_ms']:>6.2f}  "
              f"{r['p90_ms']:>6.2f}  {r['p99_ms']:>6.2f}  "
              f"{r['tput_mbps']:>7.1f}")

    print("\nOperating points (Adaptive OA, avg CO):")
    for pct, info in sorted(ops.items()):
        print(f"  avg_co~{int(pct*100)}%  ->  {int(info['load'])} Mbps  "
              f"(co0={info['co0']:.3f}, co1={info['co1']:.3f})")


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--calib-dir",  default="scratch/mlo_results_v4/calibration")
    ap.add_argument("--comp-dir",   default="scratch/mlo_results_v4/comparative")
    ap.add_argument("--plots-dir",  default="scratch/mlo_results_v4/plots_v4")
    ap.add_argument("--op-loads",   default=None,
                    help="Space-separated loads (e.g. '80 310 390 450 600 700')")
    ap.add_argument("--seeds",      default=None,
                    help="Space-separated seeds (default: all seeds found in summary)")
    args = ap.parse_args()

    setup_style()
    os.makedirs(args.plots_dir, exist_ok=True)

    calib_df = load_summary(os.path.join(args.calib_dir, "summary.csv"))
    if calib_df is None or calib_df.empty:
        print("ERROR: calibration data missing"); return
    print(f"Calibration rows: {len(calib_df)}")

    comp_df = load_summary(os.path.join(args.comp_dir, "summary.csv"))
    if comp_df is None or comp_df.empty:
        print("ERROR: comparative data missing"); return
    print(f"Comparative rows: {len(comp_df)}")

    # Determine seeds: explicit list or all unique seeds in the data
    if args.seeds:
        seeds = tuple(int(s) for s in args.seeds.split())
    else:
        seeds = tuple(sorted(comp_df["seed"].dropna().astype(int).unique()))
    print(f"Seeds used: {seeds}")

    op_loads_list = [float(x) for x in args.op_loads.split()] if args.op_loads else None
    LOAD_TO_TARGET = {80: 0.50, 310: 0.75, 390: 0.82, 450: 0.88,
                      600: 0.95, 700: 0.99}
    op_target_labels = (
        [LOAD_TO_TARGET.get(int(l), None) for l in op_loads_list]
        if op_loads_list else None
    )
    if op_target_labels and any(t is None for t in op_target_labels):
        op_target_labels = None
    ops = find_operating_points(calib_df,
                                targets=(0.50, 0.75, 0.82, 0.88),
                                op_loads_override=op_loads_list,
                                op_target_labels=op_target_labels)
    print_summary(comp_df, ops)

    print("\nGenerating figures...")
    fig_calibration(calib_df, ops, args.plots_dir)
    fig_co_timeseries(args.comp_dir, ops, args.plots_dir, seed=seeds[0])
    fig_latency_timeseries(args.comp_dir, ops, args.plots_dir, seed=seeds[0])
    fig_p_tid1(args.comp_dir, ops, args.plots_dir, seed=seeds[0])
    fig_summary_bars(comp_df, args.comp_dir, ops, args.plots_dir, seeds=seeds)
    fig_co_overlay(args.comp_dir, ops, args.plots_dir, seed=seeds[0])
    fig_latency_cdf(args.comp_dir, ops, args.plots_dir, seeds=seeds)
    fig_boxplots(args.comp_dir, ops, args.plots_dir, seeds=seeds)

    print_mannwhitney(args.comp_dir, ops, seeds)
    print("\nDone.")


if __name__ == "__main__":
    main()
