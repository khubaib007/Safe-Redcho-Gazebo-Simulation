#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/environment.sh"
exec /usr/bin/python3 "$REDCHO_ROOT/scripts/native_test.py" "${1:-smoke}"
