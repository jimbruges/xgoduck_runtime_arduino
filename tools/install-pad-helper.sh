#!/bin/sh
# Install the Bluetooth gamepad helper as a systemd user service for the current user.
# Needs no sudo: user services start at boot because lingering is enabled for `arduino`.
set -e
here=$(cd "$(dirname "$0")/.." && pwd)
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
mkdir -p "$HOME/.config/systemd/user"
cp "$here/host/xgoduck-pad.service" "$HOME/.config/systemd/user/"
systemctl --user daemon-reload
systemctl --user enable xgoduck-pad.service
systemctl --user restart xgoduck-pad.service
systemctl --user --no-pager status xgoduck-pad.service | head -5
