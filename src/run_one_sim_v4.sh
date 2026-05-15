#!/usr/bin/env bash
# Called by xargs: run_one_sim_v4.sh <sched> <load> <window> <seed>
BINARY="/home/mynavajha/ns-3-dev/build/scratch/ns3-dev-mlo-eval-v4-default"
OUTDIR="/home/mynavajha/ns-3-dev/scratch/mlo_results_v5/comparative"
LOG="${OUTDIR}/sweep.log"
SCHED="$1"; LOAD="$2"; WINDOW="$3"; SEED="$4"
OUT=$("${BINARY}" --sched="${SCHED}" --load="${LOAD}" --window="${WINDOW}" \
      --simTime=20 --seed="${SEED}" --outDir="${OUTDIR}" 2>&1)
echo "[$(date +%T)] sched=${SCHED} load=${LOAD} window=${WINDOW}ms seed=${SEED} | ${OUT}" >> "${LOG}"
