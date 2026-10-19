#!/usr/bin/env bash
# Generates .env with host-specific values (not committed to git)
cd "$(dirname "$0")/.."
cat > .env << EOF
HOST_UID=$(id -u)
HOST_GID=$(id -g)
VIDEO_GID=$(getent group video | cut -d: -f3)
RENDER_GID=$(getent group render | cut -d: -f3)
DISPLAY=${DISPLAY:-:0}
XAUTH_FILE=${XAUTHORITY:-$HOME/.Xauthority}
EOF
echo "Wrote .env:"; cat .env
