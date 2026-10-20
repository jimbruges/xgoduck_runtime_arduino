#!/bin/sh
# Back up or restore a robot's per-robot settings over ADB, run from the computer:
#   sh tools/robot-data.sh backup [dir]    copy them off the board (default checkpoints/robot-data-<date>)
#   sh tools/robot-data.sh restore <dir>   copy them onto a (new) board, then restart the app
# These files belong to the robot, not the board: back them up before replacing an Uno Q.
# Servo calibration itself is stored in each servo and moves with the robot.
set -eu
APP=/home/arduino/ArduinoApps/xgoduck
FILES="zero_pos.json joint_signs.json tuning.json controller.json servo_offsets.json"
cmd=${1:-}
case "$cmd" in
backup)
    dir=${2:-checkpoints/robot-data-$(date +%Y%m%d-%H%M%S)}
    mkdir -p "$dir"
    for f in $FILES; do
        if adb shell "test -f $APP/data/$f"; then adb pull -q "$APP/data/$f" "$dir/$f"; echo "saved $f"
        else echo "skip  $f (not on the board)"; fi
    done
    echo "backup in $dir"
    ;;
restore)
    dir=${2:?usage: robot-data.sh restore <dir>}
    adb shell "mkdir -p $APP/data"
    for f in $FILES; do
        [ -f "$dir/$f" ] || continue
        adb push -q "$dir/$f" "$APP/data/$f"; echo "restored $f"
    done
    echo "restart the app to load them: adb shell \"cd $APP && arduino-app-cli app stop user:xgoduck; arduino-app-cli app start $APP\""
    ;;
*)
    echo "usage: $0 backup [dir] | restore <dir>" >&2
    exit 2
    ;;
esac
