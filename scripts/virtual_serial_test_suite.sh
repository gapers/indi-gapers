#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DRIVER_BIN="$ROOT_DIR/indi_gapers"

USE_EMULATOR=0
if [[ "${1:-}" == "--with-emulator" ]]; then
  USE_EMULATOR=1
fi

for cmd in socat indiserver indi_getprop indi_setprop python3; do
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo "[FAIL] Missing required command: $cmd" >&2
    exit 2
  fi
done

if [[ ! -x "$DRIVER_BIN" ]]; then
  echo "[INFO] Build not found, running make"
  (cd "$ROOT_DIR" && make -j"$(nproc)")
fi

TMP_DIR="$(mktemp -d)"
HOME_ISO="$TMP_DIR/home"
mkdir -p "$HOME_ISO"
SOCAT_LOG="$TMP_DIR/socat.log"
SERVER_LOG="$TMP_DIR/indiserver.log"
EMU_LOG="$TMP_DIR/emulator.log"

SERVER_PID=""
SOCAT_PID=""
EMU_PID=""

cleanup() {
  [[ -n "$EMU_PID" ]] && kill "$EMU_PID" 2>/dev/null || true
  [[ -n "$SERVER_PID" ]] && kill "$SERVER_PID" 2>/dev/null || true
  [[ -n "$SOCAT_PID" ]] && kill "$SOCAT_PID" 2>/dev/null || true
  rm -rf "$TMP_DIR"
}
trap cleanup EXIT

wait_for_prop() {
  local query="$1"
  local timeout_s="${2:-10}"
  local waited=0
  while (( waited < timeout_s )); do
    if indi_getprop -t 1 "$query" >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
    waited=$((waited + 1))
  done
  return 1
}

read_state() {
  indi_getprop "GAPers Telescope.CONNECTION._STATE" | awk -F= '{print $2}'
}

assert_equals() {
  local expected="$1"
  local got="$2"
  local label="$3"
  if [[ "$expected" == "$got" ]]; then
    echo "[PASS] $label: $got"
  else
    echo "[FAIL] $label: expected '$expected', got '$got'"
    return 1
  fi
}

# Create PTY pair for virtual serial testing
socat -d -d pty,raw,echo=0 pty,raw,echo=0 >"$SOCAT_LOG" 2>&1 &
SOCAT_PID=$!

for _ in {1..20}; do
  [[ $(grep -c "PTY is" "$SOCAT_LOG" || true) -ge 2 ]] && break
  sleep 0.2
done

PTY_A="$(grep -m1 "PTY is" "$SOCAT_LOG" | awk '{print $NF}')"
PTY_B="$(grep -m2 "PTY is" "$SOCAT_LOG" | tail -n1 | awk '{print $NF}')"
if [[ -z "$PTY_A" || -z "$PTY_B" ]]; then
  echo "[FAIL] Could not create PTY pair"
  exit 1
fi

echo "[INFO] PTY pair: $PTY_A <-> $PTY_B"

if (( USE_EMULATOR == 1 )); then
  python3 "$ROOT_DIR/scripts/pty_plc_emulator.py" "$PTY_B" --echo >"$EMU_LOG" 2>&1 &
  EMU_PID=$!
  echo "[INFO] Emulator started on $PTY_B (pid=$EMU_PID)"
fi

HOME="$HOME_ISO" indiserver -v "$DRIVER_BIN" >"$SERVER_LOG" 2>&1 &
SERVER_PID=$!

if ! wait_for_prop "GAPers Telescope.DRIVER_INFO.DRIVER_NAME" 12; then
  echo "[FAIL] INDI driver did not become available"
  echo "----- indiserver log -----"
  cat "$SERVER_LOG"
  exit 1
fi

# Try to reset any persisted config before running checks.
indi_setprop "GAPers Telescope.CONFIG_PROCESS.CONFIG_PURGE=On" >/dev/null 2>&1 || true
indi_setprop "GAPers Telescope.CONNECTION.DISCONNECT=On" >/dev/null 2>&1 || true

# Test 1: simulation mode must connect cleanly
indi_setprop "GAPers Telescope.CONNECTION.DISCONNECT=On"
indi_setprop "GAPers Telescope.SIMULATION.ENABLE=On"
indi_setprop "GAPers Telescope.CONNECTION.CONNECT=On"
sleep 1
STATE_SIM="$(read_state)"
assert_equals "Ok" "$STATE_SIM" "Simulation connect"

# Test 2: virtual serial real-mode should be handled by INDI (Alert or Ok)
indi_setprop "GAPers Telescope.CONNECTION.DISCONNECT=On"
indi_setprop "GAPers Telescope.DEVICE_AUTO_SEARCH.INDI_DISABLED=On"
indi_setprop "GAPers Telescope.SIMULATION.DISABLE=On"
indi_setprop "GAPers Telescope.DEVICE_PORT.PORT=$PTY_A"
indi_setprop "GAPers Telescope.CONNECTION.CONNECT=On" || true
sleep 2
STATE_REAL="$(read_state)"
SIM_REAL="$(indi_getprop "GAPers Telescope.SIMULATION.ENABLE" | awk -F= '{print $2}')"

if [[ "$STATE_REAL" == "Ok" ]]; then
  echo "[PASS] Virtual serial real-mode connect: Ok"
elif [[ "$STATE_REAL" == "Alert" ]]; then
  echo "[PASS] Virtual serial real-mode managed failure: Alert"
else
  echo "[FAIL] Unexpected connection state in real-mode: $STATE_REAL"
  exit 1
fi

if [[ "$SIM_REAL" == "On" ]]; then
  echo "[WARN] Driver ended in simulation mode after real-mode test (likely config auto-load)."
fi

echo "[INFO] Final properties:"
indi_getprop "GAPers Telescope.CONNECTION._STATE" \
             "GAPers Telescope.CONNECTION.*" \
             "GAPers Telescope.SIMULATION.*" \
             "GAPers Telescope.DEVICE_PORT.PORT" \
             "GAPers Telescope.DEVICE_AUTO_SEARCH.*"

echo "[DONE] virtual serial test suite completed"
