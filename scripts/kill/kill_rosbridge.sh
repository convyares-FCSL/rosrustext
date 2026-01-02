#!/usr/bin/env bash
set -euo pipefail

BRIDGE_URL="${BRIDGE_URL:-ws://localhost:9090}"
BRIDGE_PORT="${BRIDGE_PORT:-}"
TIMEOUT="${TIMEOUT:-5}"
FORCE="${FORCE:-0}"

if [[ "$BRIDGE_URL" =~ ^ws://[^:/]+:([0-9]+)$ ]]; then
  if [[ -z "$BRIDGE_PORT" ]]; then
    BRIDGE_PORT="${BASH_REMATCH[1]}"
  fi
else
  echo "BRIDGE_URL must be ws://host:port (got: $BRIDGE_URL)" >&2
  exit 1
fi

find_pids() {
  if command -v lsof >/dev/null 2>&1; then
    lsof -iTCP:"$BRIDGE_PORT" -sTCP:LISTEN -n -P -t 2>/dev/null | sort -u
  elif command -v ss >/dev/null 2>&1; then
    ss -ltnp 2>/dev/null | awk -v port=":$BRIDGE_PORT" '$4 ~ port {print $0}' \
      | sed -n 's/.*pid=\([0-9][0-9]*\).*/\1/p' | sort -u
  elif command -v netstat >/dev/null 2>&1; then
    netstat -ltnp 2>/dev/null | awk -v port=":$BRIDGE_PORT" '$4 ~ port {print $7}' \
      | sed -n 's#/\([0-9][0-9]*\)$#\1#p' | sort -u
  fi
}

find_named_pids() {
  if command -v pgrep >/dev/null 2>&1; then
    pgrep -f "rosbridge_websocket" 2>/dev/null | grep -v "$$" | sort -u
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
  pids=( $(find_named_pids || true) )
  if [[ ${#pids[@]} -eq 0 ]]; then
    echo "No listeners on port $BRIDGE_PORT"
    exit 0
  fi
  echo "No listeners on port $BRIDGE_PORT; killing rosbridge_websocket processes: ${pids[*]}"
else
  echo "Killing listeners on port $BRIDGE_PORT: ${pids[*]}"
fi

kill "${pids[@]}" 2>/dev/null || true

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
  kill "${pids[@]}" 2>/dev/null || true
done

if [[ "$FORCE" == "1" ]]; then
  echo "Force killing remaining rosbridge processes: ${remaining[*]}"
  kill -9 "${remaining[@]}" 2>/dev/null || true
fi
