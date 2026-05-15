"""
boxplots_v4.py -- Boxplots and tabular summary for mlo-eval-v4 results.

Reads per-packet _samples.csv files (subsampled) for true latency
distribution boxplots. Also prints + saves a tabular summary from
summary.csv.

Usage:
    python3 boxplots_v4.py [--datadir PATH] [--outdir PATH] [--n-sample N]

Outputs:
    fig_box_w250.png        -- latency boxplots, window=250ms, all 3 scheds x 6 loads
    fig_box_window_oa.png   -- window size effect boxplots, Adaptive OA
    fig_box_window_mcab.png -- window size effect boxplots, MCAB
    table_summary.csv       -- tabular summary (mean+-CI, p99, tput per group)
    table_summary.txt       -- human-readable table
"""

import argparse
import glob
import os
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

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
})

SCHED_LABELS  = {0: "MLO-STR", 1: "Adaptive OA", 2: "MCAB"}
SCHED_COLORS  = {0: "#e41a1c", 1: "#377eb8", 2: "#4daf4a"}
LOADS         = [80, 310, 390, 450, 600, 700]
WINDOWS       = [250, 125, 100]


# ── Data loading ─────────────────────────────────────────────────────────────

def load_samples(datadir: str, sched: int, load: int, window: int,
                 seeds=(1, 2, 3), n_sample: int = 1000) -> np.ndarray:
    """Pool n_sample latency values per seed from _samples.csv files."""
    chunks = []
    for seed in seeds:
        pat = os.path.join(datadir,
                           f"s{sched}_l{load}_w{window}_sd{seed}_samples.csv")
        files = glob.glob(pat)
        if not files:
            continue
        try:
            df = pd.read_csv(files[0], usecols=["latency_ms"])
            vals = df["latency_ms"].dropna().values
            if len(vals) > n_sample:
                idx = np.random.choice(len(vals), n_sample, replace=False)
                vals = vals[idx]
            chunks.append(vals)
        except Exception:
            continue
    return np.concatenate(chunks) if chunks else np.array([])


# ── Boxplot helpers ──────────────────────────────────────────────────────────

def grouped_boxplot(ax, data_dict, group_labels, colors, group_spacing=1.0,
                    box_width=0.22):
    """
    data_dict: {sched: [array_per_load, ...]}  ordered by group_labels
    """
    scheds = sorted(data_dict.keys())
    n_groups = len(group_labels)
    n_scheds = len(scheds)
    offsets  = np.linspace(-(n_scheds-1)/2, (n_scheds-1)/2, n_scheds) * box_width * 1.25

    for si, sched in enumerate(scheds):
        positions = [i * group_spacing + offsets[si] for i in range(n_groups)]
        arrays    = data_dict[sched]
        valid_pos = [p for p, a in zip(positions, arrays) if len(a) > 0]
        valid_arr = [a for a in arrays if len(a) > 0]
        if not valid_arr:
            continue
        bp = ax.boxplot(valid_arr,
                        positions=valid_pos,
                        widths=box_width,
                        patch_artist=True,
                        showfliers=True,
                        flierprops=dict(marker=".", markersize=1.5,
                                        alpha=0.3, color=colors[sched]),
                        medianprops=dict(color="black", linewidth=1.2),
                        boxprops=dict(facecolor=colors[sched], alpha=0.7),
                        whiskerprops=dict(color=colors[sched]),
                        capprops=dict(color=colors[sched]))
        # Invisible line for legend
        ax.plot([], [], color=colors[sched], linewidth=4,
                alpha=0.7, label=SCHED_LABELS[sched])

    ax.set_xticks([i * group_spacing for i in range(n_groups)])
    ax.set_xticklabels(group_labels)
    ax.grid(True, axis="y", linewidth=0.4, alpha=0.5)
    return ax


def save(fig, path: str):
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {path}")


# ── Tabular summary ──────────────────────────────────────────────────────────

def build_table(summary_csv: str) -> pd.DataFrame:
    df = pd.read_csv(summary_csv)
    df["sched"]     = df["sched"].astype(int)
    df["window_ms"] = df["window_ms"].astype(int)

    rows = []
    for (sched, load, window), g in df.groupby(["sched", "load_mbps", "window_ms"]):
        n    = len(g)
        ci   = lambda s: 1.96 * s / np.sqrt(n) if n > 1 else 0.0
        rows.append({
            "Scheduler":  SCHED_LABELS[int(sched)],
            "Load_Mbps":  int(load),
            "Window_ms":  int(window),
            "Seeds":      n,
            "Mean_ms":    round(g["mean_ms"].mean(), 3),
            "Mean_CI":    round(ci(g["mean_ms"].std()), 3),
            "P50_ms":     round(g["p50_ms"].mean(), 3),
            "P90_ms":     round(g["p90_ms"].mean(), 3),
            "P99_ms":     round(g["p99_ms"].mean(), 3),
            "P99_CI":     round(ci(g["p99_ms"].std()), 3),
            "P999_ms":    round(g["p999_ms"].mean(), 3),
            "Tput_Mbps":  round(g["tput_mbps"].mean(), 1),
            "Tput_CI":    round(ci(g["tput_mbps"].std()), 1),
        })
    return pd.DataFrame(rows).sort_values(["Window_ms", "Scheduler", "Load_Mbps"])


def print_table(tbl: pd.DataFrame, txt_path: str):
    header = (
        f"{'Scheduler':<14} {'Load':>6} {'Win':>5} {'Seeds':>5} "
        f"{'Mean(ms)':>10} {'±CI':>6} "
        f"{'P50':>7} {'P90':>7} {'P99':>7} {'±CI':>6} "
        f"{'P999':>8} {'Tput(Mbps)':>11} {'±CI':>6}"
    )
    sep = "-" * len(header)
    lines = [sep, header, sep]

    prev_win = None
    for _, r in tbl.iterrows():
        if r["Window_ms"] != prev_win:
            if prev_win is not None:
                lines.append("")
            lines.append(f"  [ Window = {r['Window_ms']} ms ]")
            prev_win = r["Window_ms"]
        lines.append(
            f"{r['Scheduler']:<14} {r['Load_Mbps']:>6} {r['Window_ms']:>5} {r['Seeds']:>5} "
            f"{r['Mean_ms']:>10.3f} {r['Mean_CI']:>6.3f} "
            f"{r['P50_ms']:>7.3f} {r['P90_ms']:>7.3f} {r['P99_ms']:>7.3f} {r['P99_CI']:>6.3f} "
            f"{r['P999_ms']:>8.3f} {r['Tput_Mbps']:>11.1f} {r['Tput_CI']:>6.1f}"
        )
    lines.append(sep)

    text = "\n".join(lines)
    print(text)
    with open(txt_path, "w") as f:
        f.write(text + "\n")
    print(f"  Saved: {txt_path}")


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--datadir",  default="/home/mynavajha/ns-3-dev/scratch/mlo_results_v5/comparative")
    parser.add_argument("--summary",  default=None,
                        help="Path to summary.csv (defaults to datadir/summary.csv)")
    parser.add_argument("--outdir",   default="/home/mynavajha/wifi7-mlo-scheduler/results/v4/plots")
    parser.add_argument("--n-sample", type=int, default=1500,
                        help="Packets to sample per seed for boxplots")
    args = parser.parse_args()

    if args.summary is None:
        args.summary = os.path.join(args.datadir, "summary.csv")

    if not os.path.exists(args.summary):
        sys.exit(f"ERROR: {args.summary} not found.")

    os.makedirs(args.outdir, exist_ok=True)
    np.random.seed(42)

    # ── Tabular summary ─────────────────────────────────────────────────────
    print("\n=== Tabular Summary ===")
    tbl = build_table(args.summary)
    tbl.to_csv(f"{args.outdir}/table_summary.csv", index=False)
    print_table(tbl, f"{args.outdir}/table_summary.txt")

    # ── Boxplot 1: all 3 schedulers vs load at window=250ms ─────────────────
    print("\nBuilding boxplot: all schedulers, window=250ms ...")
    fig, ax = plt.subplots(figsize=(8, 3.5))
    data = {}
    for sched in [0, 1, 2]:
        data[sched] = [
            load_samples(args.datadir, sched, load, 250,
                         n_sample=args.n_sample)
            for load in LOADS
        ]
    grouped_boxplot(ax, data, [str(l) for l in LOADS], SCHED_COLORS)
    ax.set_xlabel("Offered Load (Mbps)")
    ax.set_ylabel("Latency (ms)")
    ax.set_title("Latency Distribution: STR vs Adaptive OA vs MCAB  (window = 250 ms)")
    ax.legend(loc="upper left")
    # Cap y-axis at 99th percentile of all data to suppress extreme outliers
    all_vals = np.concatenate([v for vlist in data.values() for v in vlist if len(v)])
    if len(all_vals):
        ax.set_ylim(bottom=0, top=min(np.percentile(all_vals, 99.5) * 1.1, ax.get_ylim()[1]))
    fig.tight_layout()
    save(fig, f"{args.outdir}/fig_box_w250.png")

    # ── Boxplot 2: window size effect on Adaptive OA ─────────────────────────
    print("Building boxplot: window effect, Adaptive OA ...")
    fig, axes = plt.subplots(1, 3, figsize=(9, 3.2), sharey=True)
    for wi, window in enumerate(WINDOWS):
        ax = axes[wi]
        data_w = {1: [load_samples(args.datadir, 1, load, window,
                                   n_sample=args.n_sample)
                      for load in LOADS]}
        grouped_boxplot(ax, data_w, [str(l) for l in LOADS],
                        SCHED_COLORS, box_width=0.4)
        ax.set_title(f"Window = {window} ms")
        ax.set_xlabel("Load (Mbps)")
        if wi == 0:
            ax.set_ylabel("Latency (ms)")
        all_vals = np.concatenate([v for v in data_w[1] if len(v)])
        if len(all_vals):
            axes[0].set_ylim(bottom=0,
                             top=min(np.percentile(all_vals, 99.5) * 1.15,
                                     axes[0].get_ylim()[1]))
    fig.suptitle("Adaptive OA: Effect of Window Size on Latency", fontsize=10)
    fig.tight_layout()
    save(fig, f"{args.outdir}/fig_box_window_oa.png")

    # ── Boxplot 3: window size effect on MCAB ───────────────────────────────
    print("Building boxplot: window effect, MCAB ...")
    fig, axes = plt.subplots(1, 3, figsize=(9, 3.2), sharey=True)
    for wi, window in enumerate(WINDOWS):
        ax = axes[wi]
        data_w = {2: [load_samples(args.datadir, 2, load, window,
                                   n_sample=args.n_sample)
                      for load in LOADS]}
        grouped_boxplot(ax, data_w, [str(l) for l in LOADS],
                        SCHED_COLORS, box_width=0.4)
        ax.set_title(f"Window = {window} ms")
        ax.set_xlabel("Load (Mbps)")
        if wi == 0:
            ax.set_ylabel("Latency (ms)")
        all_vals = np.concatenate([v for v in data_w[2] if len(v)])
        if len(all_vals):
            axes[0].set_ylim(bottom=0,
                             top=min(np.percentile(all_vals, 99.5) * 1.15,
                                     axes[0].get_ylim()[1]))
    fig.suptitle("MCAB: Effect of Window Size on Latency", fontsize=10)
    fig.tight_layout()
    save(fig, f"{args.outdir}/fig_box_window_mcab.png")

    # ── Boxplot 4: OA vs MCAB side-by-side at each window ───────────────────
    print("Building boxplot: OA vs MCAB per window ...")
    fig, axes = plt.subplots(1, 3, figsize=(9, 3.2), sharey=True)
    for wi, window in enumerate(WINDOWS):
        ax = axes[wi]
        data_w = {}
        for sched in [1, 2]:
            data_w[sched] = [
                load_samples(args.datadir, sched, load, window,
                             n_sample=args.n_sample)
                for load in LOADS
            ]
        grouped_boxplot(ax, data_w, [str(l) for l in LOADS], SCHED_COLORS)
        ax.set_title(f"Window = {window} ms")
        ax.set_xlabel("Load (Mbps)")
        if wi == 0:
            ax.set_ylabel("Latency (ms)")
            ax.legend(loc="upper left")
        all_vals = np.concatenate([v for vlist in data_w.values()
                                   for v in vlist if len(v)])
        if len(all_vals):
            axes[0].set_ylim(bottom=0,
                             top=min(np.percentile(all_vals, 99.5) * 1.15,
                                     axes[0].get_ylim()[1]))
    fig.suptitle("Adaptive OA vs MCAB: Latency by Window Size", fontsize=10)
    fig.tight_layout()
    save(fig, f"{args.outdir}/fig_box_oa_vs_mcab.png")

    print("\nAll done.")


if __name__ == "__main__":
    main()
