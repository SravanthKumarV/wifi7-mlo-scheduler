# Wi-Fi 7 MLO Scheduler Evaluation (v4)

**Comparing MLO-STR (Greedy/EDCA) against Adaptive OA (CO-equalizing)  
under asymmetric dual-link 802.11be EHT at controlled channel-occupancy operating points.**

---

## What This Evaluates

Wi-Fi 7 (IEEE 802.11be) introduces Multi-Link Operation (MLO), letting an AP and STA
transmit simultaneously over two frequency bands.  This experiment asks:

> *Does a channel-occupancy-aware TID scheduler outperform plain EDCA-based
> greedy MLO-STR for downlink UDP latency, and by how much, at increasing load?*

Two schedulers are compared across six channel-occupancy (CO) operating points:
50 %, 75 %, 82 %, 88 %, and attempted 95 % / 99 % (which saturate at the
~89 % protocol ceiling).

---

## Network Topology

```
        ┌───────────────────────────────────┐
        │              Access Point          │
        │   Link 0: 5 GHz / 40 MHz          │
        │   Link 1: 6 GHz / 80 MHz          │
        └────────────┬──────────────────────┘
                     │ (downlink UDP, DL only)
        ┌────────────┴──────────────────────┐
        │             Station (STA)          │
        └───────────────────────────────────┘
```

| Parameter        | Link 0              | Link 1              |
|------------------|---------------------|---------------------|
| Band             | 5 GHz               | 6 GHz               |
| Channel width    | 40 MHz              | 80 MHz              |
| MCS (EHT)        | EhtMcs5             | EhtMcs11            |
| PHY capacity     | ~108 Mbps (MAC)     | ~361 Mbps (MAC)     |
| Guard interval   | 800 ns              | 800 ns              |

---

## Schedulers

### Scheduler 0 — MLO-STR (Greedy / EDCA)
TID mapping: `"0 0; 3 0,1; 1,2,4,5,6,7 1"`  
All traffic sent as TID 3; EDCA picks whichever link wins contention first.
`p_tid1 = 1.0` (fixed).

### Scheduler 1 — Adaptive OA (CO-equalizing)
TID mapping: `"0 0; 1,2,3,4,5,6,7 1"`  
A proportional controller runs every 500 ms:

```
p ← clamp(p + 0.2 × (co0 − co1), 0.05, 0.95)
```

`p` fraction of packets are sent as TID 3 → Link 1; `(1−p)` as TID 0 → Link 0.  
This explicitly drives CO(Link 0) = CO(Link 1).

---

## Key Design Decisions

| Decision | Rationale |
|----------|-----------|
| **No separate BG flow** | BG hardwired to Link 0 (via `pTid1=0.0`) bypassed the scheduler entirely, making Greedy always win. All traffic now goes through the scheduler. |
| **Proportional controller** (not MCAB) | MCAB formula `p = co0/(co0+co1)` converges to `co0/co1 = p/(1-p)`, NOT `co0 = co1`, on asymmetric links. Only the proportional controller guarantees equalization. |
| **Initial condition p = 0.75** | Capacity-weighted steady state for 40/80 MHz links. Starting at 0.5 overloads Link 0 at high loads → startup queue buildup → inflated p99. |
| **CO ceiling at ~89 %** | Protocol overhead (SIFS 16 µs, DIFS 34 µs, backoff, Block ACK) makes 100 % CO physically impossible. 95 % / 99 % targets are shown on calibration as "unachievable". |

---

## Operating Points

| Target CO | Load (Mbps) | Actual CO (Adaptive OA) | Note |
|-----------|-------------|------------------------|------|
| ~50 %     | 80          | 50 %                   | Light load |
| ~75 %     | 310         | 75 %                   | Medium load |
| ~82 %     | 390         | 82 %                   | High load |
| ~88 %     | 450         | 88 %                   | Near saturation |
| ~95 %     | 600         | 89 %* | Saturated — unachievable |
| ~99 %     | 700         | 89 %* | Saturated — unachievable |

\* Protocol ceiling — CO cannot exceed ~89 % regardless of offered load.

---

## Results Summary

| CO target | MLO-STR P99 (ms) | Adaptive OA P99 (ms) | Improvement |
|-----------|-----------------|----------------------|-------------|
| ~50 %     | 0.33            | 0.53                 | STR wins (light load) |
| ~75 %     | 1.56            | 0.96                 | **1.6× lower** |
| ~82 %     | 2.71            | 1.37                 | **2.0× lower** |
| ~88 %     | 4.52            | 2.37                 | **1.9× lower** |
| ~95 %     | 10.26           | 9.42                 | **1.1× lower** |
| ~99 %     | 10.35           | 9.55                 | **1.1× lower** |

At light load (50 % CO) EDCA wins slightly because per-packet overhead dominates.  
At ≥ 75 % CO, CO-equalization consistently reduces tail latency by 1.6–2.0×.  
In the saturation regime (≥ 95 % target), Adaptive OA still wins on latency because
it keeps both links equally loaded (89 %/89 %) while Greedy drives Link 0 to 96 %.

---

## File Structure

```
scratch/
├── mlo-eval-v3.cc                  # ns-3 simulation (C++)
├── analyze_mlo_v3.py               # Analysis and plotting (Python)
└── mlo_results_v4/
    ├── calibration/
    │   └── summary.csv             # CO vs load (Adaptive OA, seed=1)
    ├── comparative/
    │   └── summary.csv             # Both schedulers × 6 loads × 3 seeds
    └── plots_v4/
        ├── fig1_calibration.png    # CO curve with operating points
        ├── fig2_co_timeseries.png  # CO evolution (2×6 panel)
        ├── fig3_latency_timeseries.png  # Latency over time (2×6 panel)
        ├── fig4_p_tid1_timeseries.png   # Scheduler probability over time
        ├── fig5_latency_percentiles.png # Paper-style bar chart (1×6 panel)
        ├── fig6_co_comparison.png  # CO overlay both schedulers
        ├── fig7_latency_cdf.png    # Latency CDFs (1×6 panel)
        └── fig8_boxplots.png       # Raw latency boxplots (1×6 panel)
```

---

## How to Reproduce

### 1. Build
```bash
cd /home/mynavajha/ns-3-dev
./ns3 build mlo-eval-v3
```

### 2. Run Calibration (Adaptive OA, seed=1, loads 20–700 Mbps)
```bash
for load in 20 30 40 50 60 70 80 90 100 110 120 130 140 150 160 170 180 190 200 \
            210 220 230 240 250 260 270 280 290 300 310 320 330 340 350 360 370 \
            380 390 400 410 420 430 440 450 500 600 700; do
  ./ns3 run "mlo-eval-v3 --sched=1 --load=${load} --simTime=25 --seed=1 \
             --outDir=scratch/mlo_results_v4/calibration"
done
```

### 3. Run Comparative (both schedulers, 6 loads, 3 seeds)
```bash
for sched in 0 1; do
  for load in 80 310 390 450 600 700; do
    for seed in 1 2 3; do
      ./ns3 run "mlo-eval-v3 --sched=${sched} --load=${load} --simTime=25 \
                 --seed=${seed} --outDir=scratch/mlo_results_v4/comparative" &
    done
  done
done
wait
```

### 4. Generate All Plots
```bash
python3 scratch/analyze_mlo_v3.py \
    --calib-dir scratch/mlo_results_v4/calibration \
    --comp-dir  scratch/mlo_results_v4/comparative \
    --plots-dir scratch/mlo_results_v4/plots_v4 \
    --op-loads  "80 310 390 450 600 700"
```

---

## Dependencies

- ns-3-dev with 802.11be / EHT MLO support (version ≥ 3.40)
- Python 3 with: `numpy`, `pandas`, `matplotlib`
- IITH EE24MTECH11033 — Wi-Fi 7 MLO Scheduler Evaluation Project
