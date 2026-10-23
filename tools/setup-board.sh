#!/bin/sh
# Set up a new or replacement Uno Q for this app. Run on the board, as `arduino`, from the
# app directory (~/ArduinoApps/xgoduck):
#   sh tools/setup-board.sh
# Safe to re-run. The one step that needs root (the router baud rate) is run with sudo
# when it is missing, so the script may ask for the arduino password once.
set -u
here=$(cd "$(dirname "$0")/.." && pwd)
cd "$here"
ok() { echo "  ok    $*"; }
todo() { echo "  TODO  $*"; pending=1; }
pending=0

echo "XGO Duck board setup in $here"

# Internet and clock: the first build downloads the pinned board core and Python packages,
# and TLS fails while the clock is wrong (a new board can boot days behind until NTP syncs).
# Headers only: the full index is many MB and a short timeout fails mid-download.
if curl -sfI -m 10 -o /dev/null https://pypi.org/simple/pip/; then
    ok "internet reachable"
else
    todo "no internet: connect Wi-Fi (App Lab, or: sudo nmcli --ask dev wifi connect <network>)"
fi
if timedatectl show -p NTPSynchronized --value 2>/dev/null | grep -q yes; then
    ok "clock synchronised ($(date -u '+%Y-%m-%d %H:%M UTC'))"
else
    todo "clock not synchronised yet ($(date -u '+%Y-%m-%d %H:%M UTC')); it syncs once online"
fi

# User services (the Bluetooth helper) only run at boot, and over ADB, with lingering on.
# A fresh board does not have it; a user can enable it for itself.
if [ "$(loginctl show-user "$(id -un)" -p Linger --value 2>/dev/null)" != yes ]; then
    loginctl enable-linger "$(id -un)" && ok "lingering enabled" || todo "loginctl enable-linger $(id -un) failed"
    for _ in 1 2 3 4 5 6 7 8 9 10; do [ -S "/run/user/$(id -u)/bus" ] && break; sleep 1; done
else
    ok "lingering on"
fi

# Router at 2 Mbps (root).
if [ -f /etc/systemd/system/arduino-router.service.d/90-xgoduck-baud.conf ]; then
    ok "router set to 2 Mbps"
else
    echo "  ...   setting the router to 2 Mbps (needs sudo)"
    if sudo bash tools/router-2m.sh; then ok "router set to 2 Mbps"; else todo "sudo bash tools/router-2m.sh"; fi
fi

# Bridge library patches. Needs the libraries in the App CLI cache; on a board that has
# never built an app, start the app once (it resolves them), then run this script again.
if python3 tools/prepare-bridge.py; then
    ok "Bridge libraries patched"
else
    todo "Bridge libraries not in the cache yet: run 'arduino-app-cli app start \"$here\"' once, then this script again"
fi

# Bluetooth gamepad helper (systemd user service).
if sh tools/install-pad-helper.sh >/dev/null 2>&1 &&
   XDG_RUNTIME_DIR="/run/user/$(id -u)" systemctl --user is-active --quiet xgoduck-pad.service; then
    ok "Bluetooth helper running"
else
    todo "Bluetooth helper did not start: sh tools/install-pad-helper.sh"
fi

# Start this app at boot.
if arduino-app-cli properties set default "$here" >/dev/null 2>&1; then
    ok "default (boot) app: $here"
else
    todo "arduino-app-cli properties set default \"$here\""
fi

# Per-robot data. Calibration lives in the servos, but these files hold the robot's zeros,
# joint directions, tuning and controller mapping. Moving a robot to a new board: restore
# them with tools/robot-data.sh restore (on the computer, over ADB). A new robot starts
# without them: set up the servos, calibrate, then run the Check page.
for f in zero_pos.json joint_signs.json tuning.json controller.json; do
    [ -f "data/$f" ] || missing="${missing:-} $f"
done
if [ -n "${missing:-}" ]; then
    echo "  note  no robot data ($missing ): fine for a new robot; for a moved one, restore it"
else
    ok "robot data present"
fi

if [ "$pending" = 0 ]; then
    echo "Ready. Start the app: arduino-app-cli app start \"$here\""
else
    echo "Finish the TODO items above, then run this script again."
fi
