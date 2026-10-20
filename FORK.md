# What this fork changes

Based on [LuwuDynamics/xgoduck_runtime_arduino](https://github.com/LuwuDynamics/xgoduck_runtime_arduino) at `9dc3678` ("Add servo scanning functionality"). Everything else (policies, control loop, web control page, Bridge setup) is unchanged.

The changes come from bringing up an assembled XGO Duck, where the upstream calibration flow tripped the battery and several joints could not be calibrated or reach the Default pose.

Companion forks for the same UNO Q build:
- [jimbruges/xgoduck_hardware](https://github.com/jimbruges/xgoduck_hardware): the clip-on UNO Q cover and the whole-robot CAD assembly (UNO Q, expansion board, 2 × 18650 pack, cover). The calibration page's zero-pose images and the retrained policy's mass placement come from it.
- [jimbruges/xgoduck_rl](https://github.com/jimbruges/xgoduck_rl), branch `cad-battery`: the walk policy retrained with the real mass and centre of mass (see below).

## Walking: what was wrong and what this fork changes

Getting the robot to walk uncovered five problems. Each has a fix or a setting, and each setting on the Control page explains itself (**What / Why / Tune**). The README's *Walk tuning* section has the full evidence.

| Problem found | Evidence | What this fork does |
| --- | --- | --- |
| Neck pitch and head pitch mounted reversed | The Check page showed them mirrored against the model the policies were trained on; every other joint and the IMU matched | Per-robot joint directions, set on the **Check** page |
| Calibration zeros lost at startup | The MCU kept factory zeros; a second calibration message overwrote the first before the MCU read it | The host resends zeros and directions until the MCU confirms; Walk waits for that |
| IMU dead after a shield brownout | The accelerometer read exactly zero while the IMU reported OK, so the policy walked blind | The firmware re-initialises the sensor; the host refuses zero IMU data |
| Servos follow about 40–50% of each commanded swing, about 85 ms late | Flight recorder (**Last walk**); training assumed a 4 ms delay and P = 5 | Adjustable servo KP/KD, plus the follow-ratio summary to tune them |
| Whole robot loses power on backward falls, needing the switch cycled | Black box: supply 8.0 V at rest, 7.5 V walking, 5.8 V in the last 0.1 s of a fall with every servo still answering; replacing the Uno Q did not help | Supply guard: the MCU caps servo torque at 50 % below 6.8 V and 20 % below 6.2 V. The real fix is a stiffer supply (high-drain cells, a 10 A BMS) |
| Centre of mass about 7–11 mm behind the trained one | The XGO training model has no Uno Q, shield or battery at the back; backward walking needed 9–10° of lean | Home lean, with a separate backward value, as a stop-gap; the real fixes are moving the battery forward or retraining |

Also: the home pose now follows each model's training metadata (20°, not the hand-tuned 23°); the IMU is levelled against a spirit level (about 4° board tilt); and command smoothing is available. The controller, calibration and setup tools below support all of this.

## Retrained walk policy (real mass and centre of mass)

The original walk policy was trained on a model without the Uno Q, shield and battery at the back. A retrained policy comes from [jimbruges/xgoduck_rl](https://github.com/jimbruges/xgoduck_rl), branch `real-mass-com`, which changes three things from Luwu's model:
- the real total mass (825 g), with the trunk's centre of mass moved back;
- current xgoduck_rl's ±15 mm centre-of-mass randomisation;
- an 8–24 ms randomised command delay, matching this runtime.

There are two retrains, differing only in where the extra mass sits:

| Model | Branch | Mass placement | Whole-robot CoM vs upstream |
|---|---|---|---|
| `xgoduck_walk_realcom.onnx` | `real-mass-com` | battery, Uno Q, shield and cover (~143 g) all behind the body at x −55 mm | 8 mm back |
| `xgoduck_walk_cadbattery.onnx` | `cad-battery` | positions from the [CAD assembly](https://github.com/jimbruges/xgoduck_hardware/blob/master/structure/assembly/xgoduck_full_assembly.solidcraft): battery (~95 g) inside the body at x −20 mm, Uno Q, shield, cover and connectors (~48 g) behind it at x −51 mm | 3.7 mm back |

The CAD check found the first retrain over-corrected: the 18650 cells sit in the body, in front of the Uno Q, not behind it. The CAD has no real masses (every part uses one default density), so the 95/48 g split is an estimate. Weighing the battery pack and the Uno Q stack would pin it down.

It was trained with Hugging Face Jobs (`--hf-jobs`, L4 GPU, about 3 h and $3) and exported with `scripts/export.py`. **Walk policy** in Walk tuning switches between `xgoduck_walk.onnx` and any other `python/xgoduck_walk*.onnx` for A/B testing.

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

Two bugs made Finish unreliable after replacing two hip servos, both fixed:

- **A Finish could be lost.** The MCU kept one pending calibration message, and the page's 250 ms keep-alive (slider targets) could replace a Finish before the MCU applied it. Calibration messages now queue in order (two slots; eight overflowed the MCU's RAM), the keep-alive goes through the queue Finish waits on, and the host holds slider targets back while it confirms a Finish from three fresh readings.
- **The offset reached RAM only.** Finish switched the joint's torque off with a write addressed to that servo, then sent the EEPROM unlock immediately; the unlock collided with the servo's status reply and was lost. The offset read back correctly but reverted at the next servo power cycle, so Default pose drove the joint to its old frame (a replaced hip roll, about 72°, which browned out the shield). Finish now waits for the reply before unlocking, and reads the written offset back. Verified across a shield power cycle.

### Offset check

The host now saves each servo's offset (`data/servo_offsets.json`) when Calibration or Servo setup closes, and re-reads them at start and whenever a servo's power returns. If any differs, Default pose and Walk are refused with the ID, until that joint is recalibrated. See the README's Offset check section.

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

A **Walk tuning** section on the Control page, saved in `data/tuning.json`. Every setting carries its own What / Why / Tune explanation on the page. The settings are:

- **Home pose: Table or Model.** Upstream uses one hand-tuned home (hip/ankle 23°) for every policy. The models' own `default_joint_pos` metadata says walk, get-up and pick were trained around 20° and roulade around 24°. **Model** uses each policy's trained home for observations and actions. Default: Table (upstream behaviour).
- **Command smoothing:** an optional low-pass on walk and head commands.
- **IMU level:** measures and cancels the IMU's mounting roll and pitch, with the body held level against a spirit level. Levelling in Default pose hid a real backward lean on the development robot. It replaces the hard-coded `IMU_BIAS_DEG`, which was zero.
- **Home lean** (standing/forward and backward values, blended by the backward command and slewed at 20°/s): moves the body forward over the feet to compensate the rearward centre of mass.
- **Battery and temperature** (amber below the training's 7.4 V minimum): the MCU's per-tick servo status read is extended from 6 to 8 bytes (adding voltage and temperature, registers 62–63). They travel in the raw packet (`DQR2`, 96 bytes), and the page shows the lowest servo voltage and the hottest servo.

## Joint direction check

A **Check** page tests every joint's direction and the IMU axes against the Microduck model the policies were trained on, with torque off. Upstream hard-codes direction −1 for all 15 servos. Here a reversed joint is fixed with **Flip direction**, saved in `data/joint_signs.json`, and applied by the MCU at runtime: a new `CAL_SET_SIGNS` message, ignored while armed. The MCU echoes the active directions in the raw packet (`posSignMask`, which grows the packet to 98 bytes), and the host re-sends them whenever they differ. On the development robot, neck pitch and head pitch were reversed. That fits the walk failing within two steps, because the policy moves the heavy head to balance.

## IMU recovery after a shield power loss

Upstream marks the IMU healthy on any successful I2C read. After a shield brownout, the QMI8658 resets to power-down but still answers I2C, so it reported "OK" with an all-zero accelerometer and gyro, and the walk policy ran with no balance feedback. The firmware now checks the sensor's enable register once a second and treats repeated all-zero readings as dead; either one re-initialises the sensor, which recovered in about 1 s on the test robot without rebooting the Uno Q. The host independently refuses to arm, and disarms, on an all-zero accelerometer, and the Control page shows "no data".

## Calibration resync and flight recorder

The host now confirms the saved zeros and joint directions on the MCU, resending them after an MCU reset or a lost message. It sends one calibration message at a time, because the MCU holds only one. Default pose and Walk wait until both are confirmed. A flight recorder keeps the last 20 s of tilt, gyro, joint targets and measured angles while position control is on (`GET /api/log`).

## Adjustable servo gains and walk summary

The run gains (servo temporary KP/KD, registers 50/51) were firmware constants (6/20). They are now Walk tuning settings saved in `data/tuning.json`. A new `CAL_SET_GAINS` message applies them live, the MCU echoes the gains in use (the raw packet is now 100 bytes), and the host resends them on mismatch. The flight recorder gains a summary (`GET /api/log/summary`, shown as **Last walk**): per command type, the legs' follow ratio (achieved over commanded swing) and lag. It measured about 0.4–0.46 at 85 ms on the development robot at the default gains.

## Notes from bring-up

These are observations from one robot, not guarantees:

- The HD-1910 servos' angle limits read 0/4095 (single-turn), yet their position readings can run past 4095 or below 0 (bit 15 set) until the servo is power-cycled.
- The MCU control thread's stack is 8 KB (upstream 4 KB), because the servo register, calibration and Finish ops now run on it.
- After a sketch upload the MCU sometimes stayed silent, or received no host commands, until the Uno Q was fully power-cycled. App restarts did not clear it. Several of these coincided with a low or tripped battery; recharging and power-cycling cleared the last one.
- The App CLI default app was set with `arduino-app-cli properties set default /home/arduino/ArduinoApps/xgoduck` so the app starts on boot. That is board configuration, not part of this repo.
- `tools/test-servo-setup.py` still passes. The new firmware paths were tested on the robot (offset writes on all 15 servos, and Finish on one unpowered joint); Finish on a powered joint and on an out-of-range joint has not been exercised yet.
