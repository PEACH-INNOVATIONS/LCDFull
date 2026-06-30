#!/usr/bin/env bash
# run_sim.sh — build (if needed) and run the LCDFull PC simulator.
#
# Usage:
#   ./run_sim.sh [-s <scale%>] [-d <session.bin>] [--gui] [--multi N]
#
#   -s <pct>        Window scale percentage (default 100). e.g. -s 75
#   -d <file.bin>   Session recording to replay via FakePIC (headless mode).
#   --gui           Launch a single FakePIC GUI instance.
#   --multi N       Launch N FakePIC GUI instances (one per boat) with unique
#                   PTY pairs and logger IDs so the Fleet View shows real data.
#                   Default recordings cycle through the available .bin files;
#                   you can change them inside each GUI window.
#
# Examples:
#   ./run_sim.sh                                           # no data
#   ./run_sim.sh -s 75                                     # 75% scale, no data
#   ./run_sim.sh -d DataFiles/session_55_packets.bin       # headless replay
#   ./run_sim.sh --gui                                     # single FakePIC GUI
#   ./run_sim.sh --multi 3                                 # 3-boat GUI simulation
#   ./run_sim.sh --multi 3 -s 75                           # 75% scale, 3 boats

set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"
FAKEPIC_DIR="${FAKEPIC_DIR:-/home/joe/Peach/WiFiHaLow/FakePIC}"

SCALE=""
SESSION=""
USE_GUI=false
MULTI=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        -s)      SCALE="$2";   shift 2 ;;
        -d)      SESSION="$2"; shift 2 ;;
        --gui)   USE_GUI=true; shift   ;;
        --multi) MULTI="$2";   shift 2 ;;
        *)       echo "Unknown option: $1"; exit 1 ;;
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
SOCAT_PIDS=()
FAKEPIC_PIDS=()
TEMP_FILES=()

cleanup() {
    [[ -n "$SIM_PID"     ]] && kill "$SIM_PID"     2>/dev/null || true
    [[ -n "$FAKEPIC_PID" ]] && kill "$FAKEPIC_PID" 2>/dev/null || true
    [[ -n "$SOCAT_PID"   ]] && kill "$SOCAT_PID"   2>/dev/null || true
    for pid in "${FAKEPIC_PIDS[@]}"; do kill "$pid" 2>/dev/null || true; done
    for pid in "${SOCAT_PIDS[@]}";   do kill "$pid" 2>/dev/null || true; done
    # Restore FakePIC config if we patched it
    if [[ -n "$CONFIG_BAK" && -f "$CONFIG_BAK" ]]; then
        mv "$CONFIG_BAK" "$FAKEPIC_DIR/fake_pic_config.json"
    fi
    # Remove temp files and PTY symlinks
    for f in "${TEMP_FILES[@]}"; do rm -f "$f" 2>/dev/null || true; done
    rm -f /tmp/fakepic /tmp/simport
    rm -f /tmp/fakepic_sim[0-9]* /tmp/simport_sim[0-9]*
}
trap cleanup EXIT INT TERM

need_socat() {
    if ! command -v socat &>/dev/null; then
        echo "[run_sim] socat not found — installing…"
        sudo apt install -y socat
    fi
}

if [[ $MULTI -ge 1 ]]; then
    # ── Multi-boat GUI mode ────────────────────────────────────────────────
    need_socat
    GUI_SCRIPT="$FAKEPIC_DIR/fake_pic_gui.py"
    if [[ ! -f "$GUI_SCRIPT" ]]; then
        echo "[run_sim] ERROR: fake_pic_gui.py not found at $FAKEPIC_DIR"
        echo "         Set FAKEPIC_DIR env var to the FakePIC directory and retry."
        exit 1
    fi

    # Default recordings to cycle through (each boat gets a different one)
    RECORDINGS=(
        "/home/joe/Peach/Data/RowingData/Binary/2x/alistairs2x.bin"
        "/home/joe/Peach/Data/RowingData/Binary/8+/session_55_packets.bin"
        "/home/joe/Peach/Data/RowingData/Binary/8+/session_296_packets.bin"
    )
    N_RECS=${#RECORDINGS[@]}

    SIM_PORTS_VAL=""

    for i in $(seq 0 $((MULTI - 1))); do
        FPIC="/tmp/fakepic_sim${i}"
        SPORT="/tmp/simport_sim${i}"

        echo "[run_sim] Starting PTY bridge ${i} (socat)…"
        socat "PTY,link=${FPIC},rawer" "PTY,link=${SPORT},rawer" &
        SOCAT_PIDS+=($!)

        # Unique logger ID file for this boat slot
        LID_FILE="/tmp/sim_lid_${i}.txt"
        echo "$((i + 1))" > "$LID_FILE"
        TEMP_FILES+=("$LID_FILE")

        # Recording: cycle through available files
        REC="${RECORDINGS[$((i % N_RECS))]}"

        # Per-boat config: clone the base config, patch port + logger_id + recording
        CFG_FILE="/tmp/sim_config_${i}.json"
        TEMP_FILES+=("$CFG_FILE")
        python3 - "$FAKEPIC_DIR/fake_pic_config.json" "$CFG_FILE" \
                   "$FPIC" "$LID_FILE" "$REC" <<'PYEOF'
import sys, json, pathlib
src, dst, port, lid_file, rec = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5]
with open(src) as f:
    cfg = json.load(f)
cfg["port"]      = port
cfg["logger_id"] = lid_file
cfg["recording"] = rec
with open(dst, "w") as f:
    json.dump(cfg, f, indent=2)
PYEOF

        SIM_PORTS_VAL="${SIM_PORTS_VAL}${SIM_PORTS_VAL:+:}${SPORT}"
    done

    # Let socat pairs settle
    sleep 0.4

    echo "[run_sim] Launching $MULTI FakePIC GUI instance(s)…"
    for i in $(seq 0 $((MULTI - 1))); do
        python3 "$GUI_SCRIPT" --config "/tmp/sim_config_${i}.json" &
        FAKEPIC_PIDS+=($!)
    done

    sleep 0.5

    echo "[run_sim] Launching lcdfull_sim with SIM_PORTS=$SIM_PORTS_VAL"
    SIM_ARGS=()
    [[ -n "$SCALE" ]] && SIM_ARGS+=(-s "$SCALE")
    SIM_PORTS="$SIM_PORTS_VAL" "$BUILD_DIR/lcdfull_sim" "${SIM_ARGS[@]}" &
    SIM_PID=$!

    echo "[run_sim] Running — close the sim window or Ctrl-C to stop."
    wait "$SIM_PID" "${FAKEPIC_PIDS[@]}"

elif $USE_GUI; then
    # ── Single FakePIC GUI mode ───────────────────────────────────────────
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

    sleep 0.5
    echo "[run_sim] Launching lcdfull_sim${SCALE:+ at $SCALE%}…"
    SIM_ARGS=()
    [[ -n "$SCALE" ]] && SIM_ARGS+=(-s "$SCALE")
    "$BUILD_DIR/lcdfull_sim" "${SIM_ARGS[@]}" &
    SIM_PID=$!

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
