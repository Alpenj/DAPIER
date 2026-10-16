#!/bin/bash
# start one work6 grid run detached; $1 = output json, $2 = processes
cd "$(dirname "$0")"
HOME=/home/robot PYTHONDONTWRITEBYTECODE=1 OMP_NUM_THREADS=1 setsid nohup /home/robot/DAPIER/so101_imitation_learning/.venv/bin/python run_parallel.py "$1" "$2" > "${1%.json}.log" 2>&1 < /dev/null &
echo "started $!"
