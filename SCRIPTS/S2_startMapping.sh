#!/usr/bin/env bash
# Мир для построения карты и записи точек: лидар включён, метки выключены.
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "${SCRIPT_DIR}/S2_startSim.sh" mode:=mapping "$@"
