#!/usr/bin/env bash

if [[ $# -ne 1 ]]; then
    echo "Usage: $0 <record_name>"
    echo "Example: $0 plug_fwd_11"
    exit 1
fi

set -m

RECORD_NAME="$1"
DATA_PID=""
TELEOP_PID=""

stop_children() {
    trap - INT

    echo
    echo "Stopping teleop.py..."
    kill -INT -- "-${TELEOP_PID}" 2>/dev/null || true

    echo "Stopping data_collect.py..."
    kill -INT -- "-${DATA_PID}" 2>/dev/null || true

    wait "${TELEOP_PID}" 2>/dev/null || true
    wait "${DATA_PID}" 2>/dev/null || true
    exit 130
}

trap stop_children INT

echo "Starting data_collect.py..."
bash -c 'exec python "$@"' bash data_collect.py robot=franka demo_num="${RECORD_NAME}" &
DATA_PID=$!

sleep 0.2

echo "Starting teleop.py..."
bash -c 'exec python "$@"' bash teleop.py robot=franka control-mode=absolute_joint record="${RECORD_NAME}" &
TELEOP_PID=$!

echo "Started:"
echo "  data_collect.py PID: ${DATA_PID}"
echo "  teleop.py      PID: ${TELEOP_PID}"
echo
echo "Press Ctrl+C to stop teleop.py, then data_collect.py."

wait
