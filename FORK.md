# What this fork changes

Based on [LuwuDynamics/xgoduck_runtime_arduino](https://github.com/LuwuDynamics/xgoduck_runtime_arduino) at `9dc3678` ("Add servo scanning functionality"). Everything else (policies, control loop, web control page, Bridge setup) is unchanged.

The changes come from bringing up an assembled XGO Duck, where the upstream calibration flow tripped the battery and several joints could not be calibrated or reach the Default pose.

## Calibration no longer starts every servo at once

Upstream, **Start calibration** enabled torque on all 15 servos and sent each one straight to raw 2047 from wherever it was. On an assembled robot the combined inrush tripped the 8.4 V pack and reset the shield.

Now:

- Start leaves torque off. Each slider follows its joint's encoder until it is dragged.
- Dragging a slider powers only that joint (KP = 3, KD = 0), starting from its present position. The MCU energizes at most one new joint per 10 ms tick.
- The MCU slews every powered joint toward its target at most 4 counts per tick (about 35°/s), so a slider jump does not snap the servo.
- Goals are sync-written only to powered joints (`syncWriteGoalsMasked`). Writing a goal to an unpowered servo can still drag it.
- A joint whose encoder reads outside 0–4095 is never powered, so its clamped goal cannot make it jump. Its slider is locked.

`/api/cal/targets` accepts `null` for a joint that should stay unpowered, and only sends the MCU a mask of the joints that have targets.

## Finish re-centres the servo so every zero reads 2048

Upstream, **Finish** stored the joint's raw encoder reading as its zero. When a horn is fitted so the joint's zero lies near the encoder wrap, that zero ends up near 0 or 4095 (or reads outside the range entirely). The joint then cannot reach part of its travel: on the test robot the right ankle's zero was 3935, so the Default pose's -23° saturated at -14° and the robot would not stand.

Now **Finish**:

1. Reads the servo's position offset (register 31).
2. Writes a new offset so the joint's present pose reads 2048, with that joint's torque off. These servos report `(encoder - offset) mod 4096`.
3. Re-holds the joint at 2048 if it was powered, so it does not move.
4. Saves 2048 as the zero only after the joint reports about 2048; otherwise it returns an error and leaves the zero unchanged.

Every calibrated joint therefore sits mid-range, about ±180° from the wrap, regardless of how its horn was fitted. Joints reading outside 0–4095 can be finished too. `data/zero_pos.json` keeps the same format; after calibration every entry is 2048.

## Zero-pose guide on the calibration page

The hardware docs have no per-joint mechanical-zero drawings. The calibration page now has a **Zero pose** panel:

- A render of the XGO-Duck CAD assembly for each joint (`assets/cal/j<ID>.png`). The part that joint moves is teal and an orange bar marks its 0° line. The assembly is posed at Microduck's model zero (all joints 0°): legs straight and vertical, feet flat and forward, neck vertical, head level and forward, beak closed. The bars were modelled into the CAD at the joint axes, so they line up with the real pivots.
- The selected joint's encoder reading, slider and **Finish** button, plus previous/next, so calibration needs no scrolling. Finish moves on to the next joint.
- Each row shows a thumbnail of its render.

Joint text and image paths are in `assets/calguide.js`.

## Move all to zero

A **Move all to zero** button sends every responding joint to its saved zero and holds it at KP = 3, using the same one-joint-per-tick, slew-limited path. It is meant for after calibration: fine-tune a joint with its slider and press Finish again. It skips joints that are missing or out of range, and warns before running.

## Servo register commands

New MCU servo-setup ops (`sketch/protocol.h`, `python/wire.py`), available through `/api/servo` in Servo setup mode:

| Op | Host action | Effect |
| --- | --- | --- |
| `SERVO_SET_MIDDLE` (8) | `set_middle` | Writes 128 to the torque register (the STS midpoint command). It had no effect on the HD-1910 servos tested, so no button exposes it. |
| `SERVO_READ_REG` (9) | `read_reg` (`new_id` = address) | Reads a 16-bit register. |
| `SERVO_WRITE_LIMIT` (10) | `write_limit` (registers 9, 11), `write_offset` (register 31) | Unlocks EEPROM, writes the word, relocks, reads it back. Torque is switched off first. Only the angle limits and the position offset are writable. |

`write_offset` takes the raw register value (0–4095). Writing 0 removes an offset.

## Bluetooth controller

A Bluetooth gamepad can drive the robot, configured from a new **Controller** page. The App Lab container cannot reach BlueZ or `/dev/input`, so `host/pad_helper.py` runs on the host as a systemd user service (`tools/install-pad-helper.sh`). It pairs controllers through `bluetoothctl`, reads input events, and posts the state to the app at 50 Hz (`POST /api/pad`). The app proxies `/api/bt/*` to it.

- **Pair new controller** pairs the first gamepad that starts advertising, so controllers that change address per mode still pair. Pairing turns on `pairable`, so BlueZ stores the bond and accepts the HID connection.
- Buttons map to mode changes, pick, roulade, hold-to-open mouth and half speed. Sticks and triggers map to forward, strafe, turn, the four head channels and the mouth, with invert, trigger and deadzone options. Saved in `data/controller.json`.
- While the controller sends input it overrides the Control page's motion and counts as the heartbeat. If it stops, motion zeroes within 0.3 s and the usual 1 s heartbeat disarms. Calibration and Servo setup ignore it.
- The Control page mirrors the controller's commands on its joysticks and sliders.
- Only generic HID modes work. The kernel lacks PlayStation and Nintendo drivers, so DualShock 4 or Switch Pro imitation modes send no input. On the GameSir Nova Lite use Android mode (A + Home, yellow).

## Head channels in Default pose

Upstream, the four head channels only feed the walk policy, so in Default pose they did nothing. Now in Default pose they drive the neck and head joints (IDs 30–33) directly: ±1 gives ±30°, ±35°, ±60° and ±20°, slewed at 100°/s. Walking is unchanged. The Control page labels them by joint.

## Walk tuning and battery readout

A **Walk tuning** section on the Control page, saved in `data/tuning.json`:

- **Home pose: Table or Model.** Upstream uses one hand-tuned home (hip/ankle 23°) for every policy. The models' own `default_joint_pos` metadata says walk, get-up and pick were trained around 20° and roulade around 24°. **Model** uses each policy's trained home for observations and actions. Default: Table (upstream behaviour).
- **Command smoothing:** an optional low-pass on walk and head commands.
- **IMU level:** measures and cancels the IMU's mounting roll and pitch, with the body held level against a spirit level. Levelling in Default pose hid a real backward lean on the development robot. It replaces the hard-coded `IMU_BIAS_DEG`, which was zero.
- **Battery and temperature:** the MCU's per-tick servo status read is extended from 6 to 8 bytes (adding voltage and temperature, registers 62–63). They travel in the raw packet (`DQR2`, 96 bytes), and the page shows the lowest servo voltage and the hottest servo.

## Joint direction check

A **Check** page tests every joint's direction and the IMU axes against the Microduck model the policies were trained on, with torque off. Upstream hard-codes direction −1 for all 15 servos. Here a reversed joint is fixed with **Flip direction**, saved in `data/joint_signs.json`, and applied by the MCU at runtime: a new `CAL_SET_SIGNS` message, ignored while armed. The MCU echoes the active directions in the raw packet (`posSignMask`, which grows the packet to 98 bytes), and the host re-sends them whenever they differ. On the development robot, neck pitch and head pitch were reversed. That fits the walk failing within two steps, because the policy moves the heavy head to balance.

## Notes from bring-up

These are observations from one robot, not guarantees:

- The HD-1910 servos' angle limits read 0/4095 (single-turn), yet their position readings can run past 4095 or below 0 (bit 15 set) until the servo is power-cycled.
- The MCU control thread's stack is 8 KB (upstream 4 KB), because the servo register, calibration and Finish ops now run on it.
- After a sketch upload the MCU sometimes stayed silent, or received no host commands, until the Uno Q was fully power-cycled. App restarts did not clear it. Several of these coincided with a low or tripped battery; recharging and power-cycling cleared the last one.
- The App CLI default app was set with `arduino-app-cli properties set default /home/arduino/ArduinoApps/xgoduck` so the app starts on boot. That is board configuration, not part of this repo.
- `tools/test-servo-setup.py` still passes. The new firmware paths were tested on the robot (offset writes on all 15 servos, and Finish on one unpowered joint); Finish on a powered joint and on an out-of-range joint has not been exercised yet.
