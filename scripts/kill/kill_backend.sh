#!/usr/bin/env bash
set -euo pipefail

TARGET_NODE="${TARGET_NODE:-hyfleet_ring_roslibrust}"
BACKEND_PATTERN="${BACKEND_PATTERN:-$TARGET_NODE}"
TIMEOUT="${TIMEOUT:-5}"
FORCE="${FORCE:-0}"

find_pids() {
  if command -v pgrep >/dev/null 2>&1; then
    pgrep -f "$BACKEND_PATTERN" 2>/dev/null | grep -v "$$" | sort -u
  fi
}

alive_pids() {
  local pid
  local alive=()
  for pid in "$@"; do
    if kill -0 "$pid" 2>/dev/null; then
      alive+=("$pid")
    fi
  done
  echo "${alive[@]}"
}

pids=( $(find_pids || true) )
if [[ ${#pids[@]} -eq 0 ]]; then
  echo "No backend processes matching '${BACKEND_PATTERN}'"
  exit 0
fi

echo "Killing backend processes matching '${BACKEND_PATTERN}': ${pids[*]}"
kill -INT "${pids[@]}" 2>/dev/null || true

start=$(date +%s)
while true; do
  sleep 0.2
  remaining=( $(alive_pids "${pids[@]}") )
  if [[ ${#remaining[@]} -eq 0 ]]; then
    exit 0
  fi
  if (( $(date +%s) - start >= TIMEOUT )); then
    break
  fi
  pids=("${remaining[@]}")
  kill -INT "${pids[@]}" 2>/dev/null || true
done

if [[ "$FORCE" == "1" ]]; then
  echo "Force killing remaining backend processes: ${remaining[*]}"
  kill -9 "${remaining[@]}" 2>/dev/null || true
fi
