#!/usr/bin/env bash
# Build only. No programmer or serial operations are provided.
set -euo pipefail
BOARD=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec python3 "$BOARD/gowin/build.py" "$@"
