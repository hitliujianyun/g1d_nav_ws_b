#!/usr/bin/env bash
# Starts guard -> lio -> nav in the foreground (bring-up without systemd). Ctrl-C stops nav and
# lio first and the guard last (the guard brings the base to rest on its own).
#
#   deploy/run_all.sh [--guard-only] [--no-lio] [--no-web]
# The web UI (g1d_web) starts right after the guard: http://<robot-ip>:8080
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set -a
# shellcheck disable=SC1091
source "$ROOT/deploy/g1d.env"
set +a
G1D_ROOT="$ROOT"
G1D_CONFIG="${G1D_CONFIG_OVERRIDE:-$ROOT/config/g1d_nav.yaml}"
export CYCLONEDDS_URI="file://$ROOT/deploy/cyclonedds.xml"
export PYTHONPATH="$ROOT/python:$ROOT/third_party/unitree_sdk2_python${PYTHONPATH:+:$PYTHONPATH}"
BIN="$ROOT/install/bin"
[ -x "$BIN/g1d_nav" ] || BIN="$ROOT/build/src/apps"
LOGS="$ROOT/logs"
mkdir -p "$LOGS"
STAMP="$(date +%Y%m%d-%H%M%S)"
cd "$ROOT"

GUARD_ONLY=0
NO_LIO=0
NO_WEB=0
for a in "$@"; do
  case "$a" in
    --guard-only) GUARD_ONLY=1 ;;
    --no-lio) NO_LIO=1 ;;
    --no-web) NO_WEB=1 ;;
    *) echo "unknown option $a" >&2; exit 2 ;;
  esac
done

pids=()
cleanup() {
  for ((i=${#pids[@]}-1; i>=0; i--)); do
    kill -INT "${pids[$i]}" 2>/dev/null || true
    wait "${pids[$i]}" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

python3 -m g1d_nav_py.guard_node --config "$G1D_CONFIG" --log-dir "$LOGS" \
  > >(tee "$LOGS/guard-$STAMP.log") 2>&1 &
pids+=($!)
sleep 2
if [ "$NO_WEB" = 0 ]; then
  python3 -m g1d_web --config "$G1D_CONFIG" > >(tee "$LOGS/web-$STAMP.log") 2>&1 &
  pids+=($!)
fi
if [ "$GUARD_ONLY" = 0 ]; then
  if [ "$NO_LIO" = 0 ]; then
    echo "[run_all] starting g1d_lio: keep the robot still for 2 s"
    taskset -c 2,3 "$BIN/g1d_lio" "$G1D_CONFIG" > >(tee "$LOGS/lio-$STAMP.log") 2>&1 &
    pids+=($!)
    sleep 3
  fi
  taskset -c 4,5 "$BIN/g1d_nav" "$G1D_CONFIG" > >(tee "$LOGS/nav-$STAMP.log") 2>&1 &
  pids+=($!)
fi
echo "[run_all] running (logs in $LOGS); Ctrl-C to stop"
wait
