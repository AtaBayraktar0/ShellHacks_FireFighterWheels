#!/usr/bin/env bash
# Created by OpenAI Codex: launch with the desktop's Isaac Sim 6.1 interpreter.
set -euo pipefail
if [[ -z "${ISAAC_SIM_PATH:-}" ]]; then
  echo 'Set ISAAC_SIM_PATH to your Isaac Sim 6.1 installation directory.' >&2
  exit 1
fi
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "${ISAAC_SIM_PATH}/python.sh" "${script_dir}/../run_isaac.py" "$@"
