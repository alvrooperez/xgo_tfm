#!/usr/bin/env bash
# ==============================================================================
# Lanzador de la Flota RASTREO en Gazebo Harmonic 3D:
# 3 Robots (R1 Naranja, R2 Azul, R3 Verde) + Bridge + 3 Behavior Trees + Misiones
# ==============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TFM_DIR="$(dirname "$SCRIPT_DIR")"

echo "======================================================================"
echo "    INICIANDO GAZEBO HARMONIC 3D CON 3 ROBOTS Y 3 BEHAVIOR TREES      "
echo "======================================================================"

cleanup() {
    echo -e "\n[CLEANUP] Deteniendo simulación Gazebo y Behavior Trees..."
    kill "$SIM_PID" 2>/dev/null || true
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "parameter_bridge" 2>/dev/null || true
    pkill -f "bt_executor_node" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

export DISPLAY="${DISPLAY:-:0}"

source /opt/ros/jazzy/setup.bash
source "$TFM_DIR/xgo_sim/install/setup.bash"
source "$TFM_DIR/xgo_behavior_tree/install/setup.bash"

HEADLESS_ARG="false"
LAUNCH_MISSIONS=""
RUN_FULL_CYCLE="false"

for arg in "$@"; do
    case "$arg" in
        --headless)
            HEADLESS_ARG="true"
            ;;
        --with-missions|-m)
            LAUNCH_MISSIONS="demo"
            ;;
        --full-cycle|-c)
            RUN_FULL_CYCLE="true"
            ;;
    esac
done

echo "[1/2] Lanzando Gazebo Harmonic + 3 Cuadrúpedos (R1, R2, R3) + 3 Behavior Trees..."
ros2 launch xgo_sim fleet_sim.launch.py headless:="$HEADLESS_ARG" launch_bt:=true &
SIM_PID=$!

echo "[2/2] Esperando a que Gazebo spawnee los 3 modelos y active el bridge (6s)..."
sleep 6

if [ "$RUN_FULL_CYCLE" = "true" ]; then
    echo "[MISIÓN] Ejecutando ciclo completo de misiones (Tránsito -> SWEEP_AREA -> RTB)..."
    python3 "$SCRIPT_DIR/run_full_mission_cycle.py"
elif [ "$LAUNCH_MISSIONS" = "demo" ]; then
    echo "[MISIÓN] Despachando misiones coordinadas a los 3 robots..."
    python3 "$SCRIPT_DIR/dispatch_missions.py" --fleet
fi

echo "======================================================================"
echo "    ¡FLOTA EN GAZEBO 3D ACTIVA Y OPERANDO!                            "
echo "    - Robot 1 (Naranja): /robot1 (Groot2 puerto 1667)                 "
echo "    - Robot 2 (Azul):    /robot2 (Groot2 puerto 1668)                 "
echo "    - Robot 3 (Verde):   /robot3 (Groot2 puerto 1669)                 "
echo "    - Despachar misiones: python3 scripts/dispatch_missions.py --fleet"
echo "    - Ciclo completo:     python3 scripts/run_full_mission_cycle.py   "
echo "======================================================================"

wait "$SIM_PID"
