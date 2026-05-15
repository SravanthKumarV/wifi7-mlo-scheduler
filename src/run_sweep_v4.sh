#!/usr/bin/env bash
# Parallel sweep: 3 schedulers x 6 loads x 3 windows x 3 seeds = 162 runs
set -euo pipefail

OUTDIR="/home/mynavajha/ns-3-dev/scratch/mlo_results_v5/comparative"
RUN_SCRIPT="/home/mynavajha/wifi7-mlo-scheduler/src/run_one_sim_v4.sh"
LOG="${OUTDIR}/sweep.log"
SCHEDS="0 1 2"
LOADS="80 310 390 450 600 700"
WINDOWS="250 125 100"
SEEDS="1 2 3"
JOBS=28

mkdir -p "${OUTDIR}"
chmod +x "${RUN_SCRIPT}"
echo "[$(date)] Sweep v4 starting (J=${JOBS})" | tee "${LOG}"

ARGFILE=$(mktemp)
for SCHED in $SCHEDS; do
    for LOAD in $LOADS; do
        for WINDOW in $WINDOWS; do
            for SEED in $SEEDS; do
                echo "${SCHED} ${LOAD} ${WINDOW} ${SEED}"
            done
        done
    done
done > "${ARGFILE}"

TOTAL=$(wc -l < "${ARGFILE}")
echo "[$(date)] Total runs: ${TOTAL}" | tee -a "${LOG}"

xargs -a "${ARGFILE}" -P "${JOBS}" -L 1 bash "${RUN_SCRIPT}"

rm -f "${ARGFILE}"
echo "[$(date)] Sweep complete: ${TOTAL} runs -> ${OUTDIR}/summary.csv" | tee -a "${LOG}"
