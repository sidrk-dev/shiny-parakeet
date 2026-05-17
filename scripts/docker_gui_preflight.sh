#!/usr/bin/env bash
set -euo pipefail

if [[ -z "${DISPLAY:-}" ]]; then
  cat >&2 <<'EOF'
ERROR: DISPLAY is not set inside Docker.
Run this script without -Gui for the safe headless check, or launch from a WSLg/X11 environment.
EOF
  exit 42
fi

display_number="$(printf '%s' "$DISPLAY" | sed -E 's/^.*:([0-9]+).*$/\1/')"
if [[ -n "$display_number" && -S "/tmp/.X11-unix/X${display_number}" ]]; then
  echo "Docker GUI display appears available: DISPLAY=$DISPLAY"
  exit 0
fi

cat >&2 <<EOF
ERROR: DISPLAY is set to $DISPLAY, but Docker cannot see the X11 socket.
Run without -Gui, or configure WSLg/X11 forwarding before launching RViz.
EOF

exit 42
