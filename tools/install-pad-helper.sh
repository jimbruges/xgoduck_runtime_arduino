#!/bin/sh
# Install the Bluetooth gamepad helper as a systemd user service for the current user.
# Needs no sudo. User services only start at boot (and over ADB) with lingering on, which a
# fresh board does not have; a user can enable it for itself.
set -e
here=$(cd "$(dirname "$0")/.." && pwd)
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
if [ "$(loginctl show-user "$(id -un)" -p Linger --value 2>/dev/null)" != yes ]; then
    loginctl enable-linger "$(id -un)"
    for _ in 1 2 3 4 5 6 7 8 9 10; do [ -S "$XDG_RUNTIME_DIR/bus" ] && break; sleep 1; done
fi
mkdir -p "$HOME/.config/systemd/user"
cp "$here/host/xgoduck-pad.service" "$HOME/.config/systemd/user/"
systemctl --user daemon-reload
systemctl --user enable xgoduck-pad.service
systemctl --user restart xgoduck-pad.service
systemctl --user --no-pager status xgoduck-pad.service | head -5
