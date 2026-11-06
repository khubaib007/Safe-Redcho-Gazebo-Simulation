#!/usr/bin/env bash
# Source this file; the location of the checkout is unrestricted.
REDCHO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export REDCHO_ROOT
export PYTHONNOUSERSITE=1
export PYTHONDONTWRITEBYTECODE=1
export MPLBACKEND=Agg
export MPLCONFIGDIR="${MPLCONFIGDIR:-$REDCHO_ROOT/results/.matplotlib}"
mkdir -p "$MPLCONFIGDIR"
export PYTHONPATH="$REDCHO_ROOT/src/safe_redcho_distributed:$REDCHO_ROOT/src/safe_redcho_swarm:$REDCHO_ROOT/src/safe_redcho_scene${PYTHONPATH:+:$PYTHONPATH}"

export PYTHONPATH="$REDCHO_ROOT/src/safe_redcho_single_sim${PYTHONPATH:+:$PYTHONPATH}"
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-1}"
