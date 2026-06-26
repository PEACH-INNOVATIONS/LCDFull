#!/usr/bin/env bash
# run_sim.sh — build (if needed) and run the LCDFull PC simulator.
#
# Usage:
#   ./run_sim.sh [-s <scale%>] [-d <session.bin>] [--gui]
#
#   -s <pct>        Window scale percentage (default 100). e.g. -s 75
#   -d <file.bin>   Session recording to replay via FakePIC.
#                   If omitted the sim opens with no live data.
#   --gui           Launch the FakePIC GUI instead of the headless replayer.
#                   Requires 'socat'. The GUI port is pre-set to /tmp/fakepic.
#
# Examples:
#   ./run_sim.sh                                           # GUI only, no data
#   ./run_sim.sh -s 75                                     # 75% scale, no data
#   ./run_sim.sh -d DataFiles/session_55_packets.bin       # headless replay
#   ./run_sim.sh -s 75 -d DataFiles/session_55_packets.bin
#   ./run_sim.sh --gui                                     # FakePIC GUI + sim
#   ./run_sim.sh -s 75 --gui                               # 75% scale + GUI

set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"
FAKEPIC_DIR="${FAKEPIC_DIR:-/home/joe/Peach/WiFiHaLow/FakePIC}"

SCALE=""
SESSION=""
USE_GUI=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        -s)    SCALE="$2";   shift 2 ;;
        -d)    SESSION="$2"; shift 2 ;;
        --gui) USE_GUI=true; shift   ;;
        *)     echo "Unknown option: $1"; exit 1 ;;
    esac
done

# ── Build if binary is missing or sources are newer ────────────────────────
if [[ ! -f "$BUILD_DIR/lcdfull_sim" ]] || \
   find "$SCRIPT_DIR" -maxdepth 1 -name "*.c" -newer "$BUILD_DIR/lcdfull_sim" | grep -q .; then
    echo "[run_sim] Building…"
    mkdir -p "$BUILD_DIR"
    cmake -S "$SCRIPT_DIR" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Debug -Wno-dev >/dev/null
    cmake --build "$BUILD_DIR" --parallel "$(nproc)"
fi

# ── socat / FakePIC setup ───────────────────────────────────────────────────
SOCAT_PID=""
FAKEPIC_PID=""
SIM_PID=""
CONFIG_BAK=""

cleanup() {
    [[ -n "$SIM_PID"     ]] && kill "$SIM_PID"     2>/dev/null
    [[ -n "$FAKEPIC_PID" ]] && kill "$FAKEPIC_PID" 2>/dev/null
    [[ -n "$SOCAT_PID"   ]] && kill "$SOCAT_PID"   2>/dev/null
    # Restore FakePIC config if we patched it
    if [[ -n "$CONFIG_BAK" && -f "$CONFIG_BAK" ]]; then
        mv "$CONFIG_BAK" "$FAKEPIC_DIR/fake_pic_config.json"
    fi
    rm -f /tmp/fakepic /tmp/simport
}
trap cleanup EXIT INT TERM

need_socat() {
    if ! command -v socat &>/dev/null; then
        echo "[run_sim] socat not found — installing…"
        sudo apt install -y socat
    fi
}

if $USE_GUI; then
    # ── FakePIC GUI mode ──────────────────────────────────────────────────
    need_socat
    GUI_SCRIPT="$FAKEPIC_DIR/fake_pic_gui.py"
    if [[ ! -f "$GUI_SCRIPT" ]]; then
        echo "[run_sim] ERROR: fake_pic_gui.py not found at $FAKEPIC_DIR"
        echo "         Set FAKEPIC_DIR env var to the FakePIC directory and retry."
        exit 1
    fi

    echo "[run_sim] Starting PTY bridge (socat)…"
    socat PTY,link=/tmp/fakepic,rawer PTY,link=/tmp/simport,rawer &
    SOCAT_PID=$!
    sleep 0.3

    # Patch config to pre-select /tmp/fakepic in the GUI's port field
    CFG="$FAKEPIC_DIR/fake_pic_config.json"
    CONFIG_BAK="$(mktemp)"
    cp "$CFG" "$CONFIG_BAK"
    python3 - "$CFG" <<'EOF'
import sys, json
p = sys.argv[1]
with open(p) as f: cfg = json.load(f)
cfg["port"] = "/tmp/fakepic"
with open(p, "w") as f: json.dump(cfg, f, indent=2)
EOF
    echo "[run_sim] Launching FakePIC GUI (port pre-set to /tmp/fakepic)…"
    python3 "$GUI_SCRIPT" &
    FAKEPIC_PID=$!

    # Give the GUI a moment to start before opening the sim window
    sleep 0.5
    echo "[run_sim] Launching lcdfull_sim${SCALE:+ at $SCALE%}…"
    SIM_ARGS=()
    [[ -n "$SCALE" ]] && SIM_ARGS+=(-s "$SCALE")
    "$BUILD_DIR/lcdfull_sim" "${SIM_ARGS[@]}" &
    SIM_PID=$!

    # Wait for either process to exit, then clean up both
    echo "[run_sim] Running — close the FakePIC GUI or the sim window to stop."
    wait "$FAKEPIC_PID" "$SIM_PID"

elif [[ -n "$SESSION" ]]; then
    # ── Headless replay mode ──────────────────────────────────────────────
    need_socat
    if [[ ! -f "$SESSION" ]]; then
        echo "[run_sim] ERROR: session file not found: $SESSION"
        exit 1
    fi
    FAKEPIC_SCRIPT="$FAKEPIC_DIR/fake_pic.py"
    if [[ ! -f "$FAKEPIC_SCRIPT" ]]; then
        echo "[run_sim] ERROR: fake_pic.py not found at $FAKEPIC_DIR"
        echo "         Set FAKEPIC_DIR env var to the FakePIC directory and retry."
        exit 1
    fi

    echo "[run_sim] Starting PTY bridge (socat)…"
    socat PTY,link=/tmp/fakepic,rawer PTY,link=/tmp/simport,rawer &
    SOCAT_PID=$!
    sleep 0.3

    echo "[run_sim] Replaying $SESSION via FakePIC…"
    python3 "$FAKEPIC_SCRIPT" "$SESSION" /tmp/fakepic &
    FAKEPIC_PID=$!

    SIM_ARGS=()
    [[ -n "$SCALE" ]] && SIM_ARGS+=(-s "$SCALE")
    echo "[run_sim] Launching lcdfull_sim ${SIM_ARGS[*]}"
    "$BUILD_DIR/lcdfull_sim" "${SIM_ARGS[@]}"

else
    # ── No-data mode ──────────────────────────────────────────────────────
    echo "[run_sim] No session file — running in GUI-only mode (no live data)."
    SIM_ARGS=()
    [[ -n "$SCALE" ]] && SIM_ARGS+=(-s "$SCALE")
    echo "[run_sim] Launching lcdfull_sim ${SIM_ARGS[*]}"
    "$BUILD_DIR/lcdfull_sim" "${SIM_ARGS[@]}"
fi
