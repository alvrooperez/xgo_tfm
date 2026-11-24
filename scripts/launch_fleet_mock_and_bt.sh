#!/usr/bin/env bash
# ==============================================================================
# Lanzador de la Flota RASTREO: Mock GUI + Drivers C++ + Behavior Trees
# ==============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TFM_DIR="$(dirname "$SCRIPT_DIR")"

echo "======================================================================"
echo "    INICIANDO SIMULADOR VISUAL MULTI-ROBOT CON 3 BEHAVIOR TREES      "
echo "======================================================================"

cleanup() {
    echo -e "\n[CLEANUP] Deteniendo procesos..."
    kill "$BT_PID" 2>/dev/null || true
    kill "$DRIVERS_PID" 2>/dev/null || true
    kill "$MOCK_PID" 2>/dev/null || true
    rm -f /tmp/tty_robot1 /tmp/tty_robot2 /tmp/tty_robot3 /tmp/tty_robot
}
trap cleanup EXIT INT TERM

# 1. Arrancar el Mock Multi-Robot en Tkinter
echo "[1/3] Lanzando la arena gráfica con los 3 perros (Mock ESP32)..."
export DISPLAY="${DISPLAY:-:0}"
python3 "$TFM_DIR/xgo_tfm/mock/mock_esp32.py" --multi &
MOCK_PID=$!
sleep 2

# 2. Arrancar los 3 drivers de bajo nivel
echo "[2/3] Conectando los 3 drivers C++ (/robot1, /robot2, /robot3)..."
source /opt/ros/jazzy/setup.bash
source "$TFM_DIR/xgo_tfm/install/setup.bash"

ros2 launch xgo_driver_cpp fleet_drivers.launch.py &
DRIVERS_PID=$!
sleep 2

# 3. Arrancar los 3 Behavior Trees
echo "[3/3] Despertando los 3 cerebros Behavior Tree en paralelo..."
source "$TFM_DIR/xgo_behavior_tree/install/setup.bash"

ros2 launch xgo_behavior_tree fleet_behavior_trees.launch.py &
BT_PID=$!

if [ "$1" = "--with-missions" ] || [ "$1" = "-m" ]; then
    echo "[MISIÓN] Se despacharán misiones automáticas en 4 segundos..."
    (sleep 4 && python3 "$SCRIPT_DIR/dispatch_missions.py" --fleet) &
fi

echo "======================================================================"
echo "    ¡TODO EL SISTEMA ESTÁ VIVO Y CORRIENDO!                           "
echo "    - Ventana visual abierta con los 3 perros: R1, R2, R3             "
echo "    - Drivers conectados en /tmp/tty_robot1, 2 y 3                    "
echo "    - 3 Behavior Trees supervisando a los perros                      "
if [ "$1" = "--with-missions" ] || [ "$1" = "-m" ]; then
echo "    - Misiones autónomas activadas para los 3 perros                  "
fi
echo "======================================================================"

# Esperar a que el usuario cierre la ventana del simulador
wait "$MOCK_PID"
