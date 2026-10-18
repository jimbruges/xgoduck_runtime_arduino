# XGO Duck

> **Fork:** this fork adds safer, repeatable zero calibration and servo register tools. See [FORK.md](FORK.md) for what differs from [LuwuDynamics/xgoduck_runtime_arduino](https://github.com/LuwuDynamics/xgoduck_runtime_arduino).

This project runs walking, get-up, pick, and roulade policies on an XGODuck built around the Arduino Uno Q.

It is based on [Microduck](https://github.com/pollen-robotics/microduck) by [Pollen Robotics](https://pollen-robotics.com/microduck/).

The policies here follow that project: 15 servos, a 50 Hz neural controller, and the same joint order.

## What runs where

The Uno Q is two computers.

| | Host | MCU |
| --- | --- | --- |
| Silicon | Qualcomm Linux | STM32U585, Zephyr |
| Rate | 50 Hz policy | 100 Hz bus loop |
| Work | ONNX Runtime, one thread; web UI on port 9527 | 15-servo bus, QMI8658, Bridge report thread |
| Link | Official Arduino App container | `Bridge` at 2 Mbps |

The MCU loop does not wait for Linux. It publishes a 240-byte `duck_state` snapshot at 100 Hz. The host filters each snapshot on arrival and the 50 Hz thread reads the latest one. A late cycle is skipped. There is no queued backlog of actions.

```text
browser :9527
    |
Linux  50 Hz ONNX  ---- duck_command (80 bytes) ---->  STM32 100 Hz
    ^                                                      |
    +------------- duck_state (240 bytes) -----------------+
                                                           |
                                              Serial1 1 Mbps, IDs 10-14, 20-24, 30-34
                                              Wire, QMI8658
```

Joint angles, rates, and targets are float32 degrees. Servo positions are saturated to the encoder range 0–4095. Non-finite targets are rejected. A finite angle outside ±180° is not rejected.

## Servo map

IDs follow Microduck. The bus always addresses all 15, in this order:

| IDs | Role |
| --- | --- |
| 10–14 | Leg |
| 20–24 | Leg |
| 30–33 | Neck and head. Policy order inside the 14-D action is 10–14, 30–33, 20–24. |
| 34 | Mouth. It is not part of the policy action. The web UI commands it from 0° to 30°. |

The home pose in `sketch/duck_config.h` and `python/rl_core.py` is hip/ankle ±23°, knee 0°, hip roll ±5°, neck and head pitch 20°, mouth 0°. `action_scale` is 1. Home is this table, not an ONNX metadata field.

Factory encoder zeros are `ZERO_POS` in `sketch/duck_config.h`. A calibration overwrites them in `data/zero_pos.json` and the host copies that file into MCU RAM at startup. `data/zero_pos.json` is local to the robot and is gitignored.

## First-time servo setup

**Scan all IDs** checks every address from **0 through 253** at the configured servo bus rate (1 Mbps). **Stop scan** cancels the scan and keeps the IDs already found. Click a result, or type any valid address into **Current / target ID**. **Read** also accepts any address in this range. Addresses 254 (broadcast) and 255 are not assignable servo IDs.

To rename a servo, enter its current ID and the desired **Change ID to** value, then click **Change ID**. The app checks that the source responds and the destination is free, writes the ID, and verifies replies at the new and old addresses. A Bridge timeout aborts scanning; it is not reported as a missing servo. The normal Control page still monitors the robot's configured 15 joint IDs.

Deploy `assets/`, `python/`, and `sketch/` together for this feature; updating only the page or Python is insufficient. Restart the app through App Lab / App CLI so the updated sketch is compiled and uploaded.

Do this before the servos are installed in the frame. Connect **one servo at a time**. Two servos with the same ID on the bus will both accept a write.

1. Open **Servo setup** (`/servo.html`) and enter setup.
2. Assign IDs **10–14** and **20–24** for the legs. Assign **30–34** for the neck, head, and mouth. The ID write unlocks register `0x37` and then writes `0x05`.
3. Center that servo. **Move to** raw position **2047** and write it. Temporary run gains for this move are KP = 6 and KD = 20. ID 34 uses temporary KP = 10.
4. Write stored KP/KD. The page defaults are **KP = 5** and **KD = 20**. The write unlocks `0x37` and stores both bytes at `0x15`.
5. Disconnect that servo and repeat until every ID exists exactly once.
6. Install the servos in the frame and finish the wiring: servo bus on D0/D1 (`Serial1`, 1 Mbps), QMI8658 on D20/D21 (`Wire`, 400 kHz, address `0x6A` then `0x6B`).

After power-up the controller also writes the temporary torque register (address 40) and temporary KP/KD (addresses 50 and 51): KP = 6, KD = 20, and KP = 10 on ID 34. Torque off clears torque and does not write a goal position. Writing a goal after torque-off still drags a connected servo, so position writes are suppressed while disarmed.

## Calibration

Calibration records each joint's **zero**: the encoder reading when that joint is at the model's 0°. The policies, Default pose and walking all measure joint angles from these zeros, so a wrong zero shows up as a robot that leans, binds or will not stand.

Zero is the pose of the XGO-Duck CAD assembly, which is Microduck's model with every joint at 0°:

- legs straight and vertical,
- feet flat and pointing straight forward,
- neck vertical,
- head level and facing forward,
- beak closed.

This is **not** the Default pose. Default pose bends the hips, ankles and neck away from zero.

### Before you start

- The harness is wired and all 15 IDs answer. Servo setup's **Scan all IDs** should find 10–14, 20–24 and 30–34.
- The 8.4 V pack is charged and the servo supply is on.
- The robot is held or supported so that a released joint cannot let it fall.

### Zero pose for each joint

The calibration page shows these renders of the CAD assembly in its **Zero pose** panel. In each one the part moved by that joint is **teal**, and an **orange bar**, modelled into the CAD at the joint's axis, marks its 0° line. Line the real part up with the orange bar.

**Left leg (IDs 10–14)**

<table>
<tr>
<td align="center"><img src="assets/cal/j10.png" width="170" alt="Left hip yaw at zero, bottom view"><br><b>10 · hip yaw</b><br>Bottom view, front to the right. Foot points straight forward, parallel to the line.</td>
<td align="center"><img src="assets/cal/j11.png" width="170" alt="Left hip roll at zero, front view"><br><b>11 · hip roll</b><br>Front view. Leg hangs straight down, not splayed out or tucked in.</td>
<td align="center"><img src="assets/cal/j12.png" width="170" alt="Left hip pitch at zero, side view"><br><b>12 · hip pitch</b><br>Left side view. Thigh vertical, not swung forward or back.</td>
</tr>
<tr>
<td align="center"><img src="assets/cal/j13.png" width="170" alt="Left knee at zero, side view"><br><b>13 · knee</b><br>Left side view. Knee straight; shank in line with the thigh.</td>
<td align="center"><img src="assets/cal/j14.png" width="170" alt="Left ankle at zero, side view"><br><b>14 · ankle</b><br>Left side view. Sole flat along the horizontal line, square to the shank.</td>
<td></td>
</tr>
</table>

**Right leg (IDs 20–24)**

<table>
<tr>
<td align="center"><img src="assets/cal/j20.png" width="170" alt="Right hip yaw at zero, bottom view"><br><b>20 · hip yaw</b><br>Bottom view, front to the right. Foot points straight forward.</td>
<td align="center"><img src="assets/cal/j21.png" width="170" alt="Right hip roll at zero, front view"><br><b>21 · hip roll</b><br>Front view. Leg hangs straight down.</td>
<td align="center"><img src="assets/cal/j22.png" width="170" alt="Right hip pitch at zero, side view"><br><b>22 · hip pitch</b><br>Right side view. Thigh vertical.</td>
</tr>
<tr>
<td align="center"><img src="assets/cal/j23.png" width="170" alt="Right knee at zero, side view"><br><b>23 · knee</b><br>Right side view. Knee straight.</td>
<td align="center"><img src="assets/cal/j24.png" width="170" alt="Right ankle at zero, side view"><br><b>24 · ankle</b><br>Right side view. Sole flat, square to the shank.</td>
<td></td>
</tr>
</table>

**Neck, head and mouth (IDs 30–34)**

<table>
<tr>
<td align="center"><img src="assets/cal/j30.png" width="170" alt="Neck pitch at zero, side view"><br><b>30 · neck pitch</b><br>Side view. Neck vertical; both neck pivots on the line.</td>
<td align="center"><img src="assets/cal/j31.png" width="170" alt="Head pitch at zero, side view"><br><b>31 · head pitch</b><br>Side view. With the neck vertical, the head's underside is level.</td>
<td align="center"><img src="assets/cal/j32.png" width="170" alt="Head yaw at zero, top view"><br><b>32 · head yaw</b><br>Top view, front to the right. Head faces straight forward.</td>
</tr>
<tr>
<td align="center"><img src="assets/cal/j33.png" width="170" alt="Head roll at zero, front view"><br><b>33 · head roll</b><br>Front view. Head level side to side.</td>
<td align="center"><img src="assets/cal/j34.png" width="170" alt="Mouth at zero, side view"><br><b>34 · mouth</b><br>Side view. Beak fully closed. The jaw is not a separate CAD part, so the whole head is teal; the line marks the beak seam.</td>
<td></td>
</tr>
</table>

### Step by step

1. **Open Calibration** (`/calibrate.html`) and press **Start calibration**. Policy and pose control stop. Torque stays **off**, so nothing moves yet, and each slider follows its joint's encoder.
2. **Select a joint.** Click its row, or use **←** and **→** in the **Zero pose** panel. The panel shows that joint's render and instruction, its encoder reading, its slider and its **Finish** button, so you do not need to scroll.
3. **Put the joint at zero**, matching the orange line in the render. Either:
   - move it by hand while it is unpowered, or
   - drag its slider. This powers **only that joint**, gently (KP = 3, KD = 0), starting from where it is. The MCU moves it toward the slider at about 35°/s and powers at most one new joint every 10 ms, so the servos never all start at once.
4. **Press Finish.** The servo's stored position offset is shifted so this pose reads **2048**, and 2048 is saved as the joint's zero (see [What Finish does](#what-finish-does)). A powered joint stays where it is. The row turns green and the panel moves on to the next joint.
5. **Repeat for all 15 joints.** Finishing a joint again replaces its zero with its current pose, so only press Finish when the joint is actually at zero.
6. **Fine-tune (optional, after every joint is calibrated).** Press **Move all to zero**. Every responding joint is powered in turn, moved slowly to its saved zero and held there at KP = 3. Check the whole robot against the renders, nudge any joint with its slider, and press **Finish** on it again. Joints that are missing or out of range are skipped and listed.
7. **Exit.** Torque turns off. Then, with the robot supported, test **Default pose** from the Control page before trying to walk.

A green row only marks joints finished during this visit. The saved zeros are in `data/zero_pos.json`, which is local to the robot and gitignored. After a full calibration every entry is 2048.

### What Finish does

Each servo reports its position as `(encoder - offset) mod 4096`, where the offset is stored in the servo (register 31). The horn can be fitted with a joint's zero anywhere on the encoder, including right at the wrap between 4095 and 0. A zero there leaves almost no room in one direction: the joint either cannot reach its target, or crosses the wrap and jumps.

So **Finish**, on the MCU:

1. reads the servo's current offset,
2. switches that joint's torque off and writes a new offset so the present pose reads 2048,
3. re-holds the joint at 2048 if it was powered, so it does not move.

The host then waits for the joint to report about 2048 before saving that as its zero. If the servo does not confirm within 0.8 s, Finish shows an error and the zero is left unchanged. Every calibrated joint therefore sits mid-range, about ±180° from the wrap, however its horn was fitted.

To shift an offset by hand instead, enter Servo setup and post `{"action": "write_offset", "target_id": ID, "raw_pos": OFFSET}` to `/api/servo`, with `OFFSET = (reading + current offset - 2048) mod 4096`. `read_reg` with `new_id: 31` returns the current offset. Writing 0 removes the offset. The `set_middle` action (writing 128 to address 40, the STS midpoint command) did not change the reading on the HD-1910 servos tested.

### Troubleshooting

| Symptom | What to do |
| --- | --- |
| A row shows **encoder … out of range** and its slider is locked | The joint is past the encoder wrap. Move it to zero by hand and press **Finish**; that re-centres it. |
| **Finish** reports that the joint *did not confirm the offset change* | The servo did not answer. Check its cable and the servo supply. The zero was not changed. |
| The Control page shows IMU **no data** | The IMU answers but reads zero, usually after a shield power loss. The firmware re-initialises it within about a second; Default pose and Walk are refused until it reports real data. |
| Encoder readings show **—** or rows stop updating | No servo replies. Check the battery, the servo supply and the cables; a loose head cable drops IDs 30–34. If the page shows no readings at all after an app update, the MCU may not have started: fully power-cycle the Uno Q. |
| The shield resets or the battery cuts out | Do not power many joints at once. Calibration only powers the joints you drag, one at a time. Recharge the pack if it has been cutting out. |
| Default pose does not stand | Check each joint against the renders with **Move all to zero**, and re-finish any that are off. |

## Joint direction check

A servo fitted the other way round still calibrates and can even hold Default pose: a hip pitch and ankle that are both reversed still give flat feet, just leaning the wrong way. The walk policy then gets mirrored feedback and falls within a step or two. The **Check** page (`/check.html`) finds this, with torque off.

1. Open **Check** and press **Torque off**.
2. For each IMU row, press **Start** and tilt or turn the whole robot as described. For each joint row, press **Start** and move only that joint 20–30° as described.
3. A row turns green when the robot measures the motion with the sign the policies were trained with, and red ("reversed") when it is mirrored.
4. On a red joint row press **Flip direction**, then run the row again to confirm it turns green.

The expected directions come from the Microduck model the policies were trained on (robot frame x forward, y left, z up; "left" and "right" are the robot's own):

| ID | Joint | Positive direction |
| --- | --- | --- |
| 10 / 20 | hip yaw | toes turn to the robot's right (left foot inward, right foot outward) |
| 11 / 21 | hip roll | feet move to the robot's left (left foot outward, right foot inward) |
| 12 / 22 | hip pitch | left foot swings backward, right foot swings forward |
| 13 / 23 | knee | left foot moves forward, right foot moves backward |
| 14 / 24 | ankle | left toe down, right toe up |
| 30 / 31 | neck / head pitch | neck tilts back / head tips nose-down |
| 32 / 33 | head yaw / roll | head turns left / left side goes down |
| 34 | mouth | beak opens |

IMU: nose-down gives positive gravity x; left side down gives positive gravity y. Turning left gives positive gyro z.

Flips are stored on the robot in `data/joint_signs.json` (gitignored), as the list of IDs that use direction +1 instead of the firmware default of −1. The host sends them to the MCU at startup, re-sends them if the MCU reports anything different (for example after an MCU reset), and refuses changes while torque is on. Calibration zeros are raw encoder values, so flipping a direction does not invalidate them. **Reset all directions** returns every joint to the default. A reversed IMU row cannot be flipped here; check how the board is mounted.

On the robot this fork was developed on, neck pitch (30) and head pitch (31) were reversed. Every other joint and the IMU matched.

## Running

Pinned stack: App CLI with Arduino Zephyr core 1.0.0, RouterBridge 0.4.3, RPClite 0.3.1, MsgPack 0.4.2. `sketch/sketch.yaml` lists those libraries, including the transitive ones. Python packages are installed by the App build from `python/requirements.txt`.

On the Uno Q, from this directory:

```bash
python3 tools/prepare-bridge.py
sudo bash tools/router-2m.sh
arduino-app-cli app start "$PWD"
```

`prepare-bridge.py` patches the resolved library cache and keeps the original headers as `.h.original`:

- RouterBridge 0.4.3: RPC thread stack 500 → 4096 bytes.
- RPClite 0.3.1: a bounded MessagePack scanner accepts a complete frame before `Unpacker::feed()`, and rejects a frame larger than the RPC buffer.

Re-run the script after a library-cache wipe or on a new board. If it cannot find the libraries, let the App CLI resolve `sketch.yaml` once, then run it again.

`router-2m.sh` sets the router serial rate to 2 Mbps in `/etc/systemd/system/arduino-router.service.d/90-xgoduck-baud.conf` and leaves the board GPIO ready-hook in place. That rate is global. Before starting another app that calls `Bridge.begin()` at 115200, move the drop-in aside, reload systemd, and restart `arduino-router`. Both ends must use the same rate. Restarting the router recreates its socket and resets the MCU, so stop this app first, restart the router, then start the app again. A normal app restart does not need a router restart.

If an isolated CLI build tries to re-download a toolchain that is already on the board and GitHub is unreachable, run `python3 tools/reuse-toolchain.py` and start the app again.

GCC 14 can tail-call `k_thread_name_set` at the end of `setup()` with a Thumb `JUMP24` from the loadable sketch into flash. The Uno Q loader then returns ENOEXEC (−8) and the MCU never starts. The `asm volatile("nop")` at the end of `setup()` keeps a long call. `python3 tools/verify-elf.py` checks the built ELF for the same class of short branch to an unresolved firmware symbol.

The web UI is served on port 9527. App Lab starts the same app under the name in `app.yaml`.

The app boots in **inference only**: the network runs, torque stays off. From the control page:

| Control | Effect |
| --- | --- |
| Inference only | Policy runs. Torque off. |
| Default pose | Holds the home pose. Requires a fresh MCU and IMU sample. |
| Walk / get up | Enables position control and the recovery state machine. |
| Pick | While upright in walk, runs `xgoduck_pick.onnx` for 4 s, then returns to walk. |
| Roulade | While upright in walk, runs `xgoduck_roulade.onnx` for 1.9 s, then returns to walk. |
| Torque off | Disables position control. |

Closing the page, or leaving it in the background for more than 1 second, turns position control off. Calibration and servo setup use a 2 second heartbeat and then exit and turn torque off.

Walk commands are vx ±0.4, vy ±0.3, ωz ±1.0, with a joystick dead zone of 0.1. The four head channels are ±1; see [Head in Default pose](#head-in-default-pose). Keyboard: W/S, A/D, Q/E, Space. A Bluetooth gamepad can also drive the robot; see [Bluetooth controller](#bluetooth-controller).

## Walk tuning

The **Walk tuning** section of the Control page holds settings for A/B testing how well the policies transfer to the robot. Changes apply immediately and are saved on the robot in `data/tuning.json` (gitignored). The defaults reproduce upstream behaviour.

| Setting | Default | What it does |
| --- | --- | --- |
| Home pose | Table | The pose that observations are measured from and that actions are added to. **Table** is the hand-tuned `HOME_DEG` (hip pitch and ankle ±23°, knee 0°) for every policy. **Model** uses each ONNX policy's own `default_joint_pos` metadata: walk, get-up and pick were trained around ±20° (knee ∓0.29°), roulade around ±24°. Switch only while not walking. |
| Command smoothing | off | A first-order low-pass on the walk and head commands, as a time constant (0–0.5 s). Stick and key steps reach the policy as ramps. Upstream Microduck smooths commands the same way. |
| IMU level | 0°, 0° | The IMU's mounting roll and pitch, cancelled before the gravity filter. With torque off, hold the robot so its **body** is level: a spirit level (or phone level app) on the body's flat top should read level front-to-back and side-to-side. Keep it still and press **Level IMU**. It averages the accelerometer for 1 s and refuses if the robot moves, isn't at ~1 g, or is tilted more than 30°. **Reset** clears it. Don't level in Default pose: the head's weight and servo sag can tilt the body a few degrees, and levelling would hide that real lean. On the development robot that gave −5.4° and made it fall backwards. |

| Home lean | 0° | Stands the hip pitch and ankle home angles this many degrees more upright, in every policy's home and in Default pose. That moves the body forward over the feet by about 1.3 mm per degree, to compensate for a centre of mass behind the trained one. The XGO training model does not place the battery and Uno Q at the back, which puts the real CoM an estimated 7–11 mm behind. Change it while not walking. |
| Servo KP | 6 | Proportional gain of every joint servo in Default pose and Walk (the servos' temporary KP register, 50). Higher follows the policy more closely; too high buzzes, oscillates or runs hot. Applied live and confirmed on the MCU. |
| Servo KD | 20 | Derivative gain (register 51). Lower responds faster but can overshoot; upstream Microduck runs its servos with none. |

**Last walk** summarises the flight recorder for the most recent Walk run. For each command type (forward, backward, turn, still) it shows the **follow ratio**, how much of each commanded leg swing the servos achieved (1.0 = all of it), and the delay. It also shows any fall and the resting body pitch. On the development robot at KP 6 / KD 20, legs followed about 0.46 forward and 0.40 backward, roughly 85 ms late. The policy then over-corrects and walking backward falls. To tune: **Clear**, walk the same sequence, **Refresh**, then change one gain in small steps and repeat, watching the servo temperatures in the header.

The section header shows the **battery** voltage (the lowest servo supply reading, register 62) and the hottest servo temperature (register 63). They are read with the 100 Hz status and reported at 5 Hz. The policies were trained with supply voltages of roughly 6.5–8.2 V, so torque and walking degrade as the pack runs down; it turns amber below 7.0 V.

### Why these three

- **Home pose:** the policies' metadata says walk, get-up and pick were trained around 20°, but the runtime uses 23° for all of them (upstream started at 24°, which matches roulade, then hand-tuned to 23°). With Table, the walk policy sees every hip and ankle 3° away from where it is, and its targets are shifted by 3°. Upstream Microduck's runtime notes that this kind of mismatch "looks like a tuning or timing problem". Table may still walk better if the 23° compensates for something physical, which is what the A/B switch is for.
- **IMU level:** projected gravity is a policy input. It must be zeroed against a truly level body, not a standing pose. Training randomised the IMU mounting by up to 6° about a random axis, and upstream corrects a systematic board pitch offset at the source. The QMI8658 on this shield has never been levelled.
- **Command smoothing:** policies are trained on smoothly varying commands; a gamepad gives steps.

### Suggested A/B procedure

1. Charge the battery (check the header reads ≥ 8.0 V) and use the same flat floor each time.
2. **Level IMU** once, holding the body level against a spirit level on its flat top (torque off), not standing in Default pose.
3. With **Table**, then with **Model**, walk the same short sequence: stand, walk forward at half stick for 5 s, turn in place each way, stop. Note falls, drift, foot scuffing and how quickly it recovers.
4. Repeat step 3 with command smoothing at 0.15 s.
5. Keep whichever combination is steadier. Leave the others at their defaults while testing one change.

Other things that affect walking but aren't settings yet: servo run gains (KP 6, KD 20 in `sketch/duck_config.h`; upstream Microduck runs its servos with D = 0), action smoothing (the **Action smoothing** sliders; upstream disagrees on whether its policies expect filtering), mechanical backlash (training assumed about ±1° per joint), and foot friction.

## Bluetooth controller

A Bluetooth gamepad can drive the robot, and the **Controller** page (`/controller.html`) pairs it and sets what each button and stick does.

### How it is wired

The App Lab container cannot reach BlueZ or `/dev/input`. App Lab generates its Docker settings without the `input` group, the input device class or the D-Bus socket, and regenerates them on every start. So a small helper runs on the Uno Q host instead:

```text
gamepad ── Bluetooth ──> BlueZ ──> /dev/input/eventN
                                        │
              host/pad_helper.py  (user service xgoduck-pad, port 9528)
                │  POST /api/pad at 50 Hz              ▲ /bt/* pairing calls
                ▼                                      │
              app container  ── /api/bt/* proxy ───────┘  (msgpack-rpc-router = host gateway)
```

`host/pad_helper.py` uses only the standard library. It pairs, trusts, connects and forgets devices through `bluetoothctl`, reads the first connected gamepad's input events, and posts the button and stick state to the app. It accepts HTTP only from loopback and Docker bridge addresses.

Install it once on the board (no sudo needed: user services start at boot because lingering is enabled for `arduino`):

```bash
tools/install-pad-helper.sh
```

### Pairing

1. Open **Controller** and press **Pair new controller**.
2. Within 60 s, put the controller in pairing mode. The page pairs the first gamepad that starts advertising, then trusts and connects it.
3. A paired controller reconnects by itself whenever it is switched on.

**Scan** lists nearby named devices; **Pair & connect**, **Connect**, **Disconnect** and **Forget** act on one. Pairing switches the adapter to pairable first. Without that, BlueZ pairs without storing a bond and then rejects the controller's HID connection ("Rejected connection from !bonded device").

Use the controller's **Android** or generic HID mode. This board's kernel has no `hid-playstation`, `hid-sony` or `hid-nintendo` driver, and `/dev/hidraw*` is root-only. Modes that imitate a DualShock 4 (often labelled PC or PS4) or a Switch Pro controller pair, but send no input. Controllers can use a different Bluetooth address in each mode.

| GameSir Nova Lite | Combo, from off | LED | Works |
| --- | --- | --- | --- |
| Android | A + Home | yellow, fast blink | yes |
| PC | B + Home | blue | no (DualShock 4 imitation) |
| Switch | Y + Home | red | no (Switch Pro imitation) |

The Nova Lite leaves pairing mode after about a minute if nothing connects.

### Mapping

The **Live input** panel shows each button's name as you press it, and each axis value as you move a stick or trigger. The matching mapping row lights up. Mappings are saved on the robot in `data/controller.json`.

| Button action | Effect |
| --- | --- |
| Torque off, Inference only, Default pose, Walk / get up | Switch mode, once per press. |
| Pick, Roulade | Start that move, once per press. Same rules as the Control page. |
| Open mouth (hold) | Mouth fully open while held. |
| Half speed (hold) | Halve forward, strafe and turn while held. |

| Axis target | Range |
| --- | --- |
| Forward / back, Strafe, Turn | Full stick = vx ±0.4, vy ±0.3, ωz ±1.0. |
| Neck pitch, Head pitch, Head yaw, Head roll | ±1, see below. |
| Mouth | 0–30°, linear over the trigger's travel. |

Stick axes run −1 to +1, with up and left negative. Analog triggers reported as `gas` or `brake` run 0 (released) to 1. Tick **Invert** if an axis moves the robot the wrong way, and **Trigger** for an axis that rests at −1 (some pads report triggers as `lz` or `rz`). The deadzone applies to sticks.

Default mapping:

| Input | Action |
| --- | --- |
| south (A) | Walk / get up |
| east (B) | Torque off |
| west, north | Pick, Roulade |
| start | Default pose |
| select | Inference only |
| l1 | Open mouth (hold) |
| r1 | Half speed (hold) |
| left stick | Forward / back and strafe |
| right stick, horizontal | Turn |

### Behaviour and safety

- While a controller is sending input, its sticks drive motion and the Control page's joysticks and head sliders are ignored. The page mirrors the controller's commands instead.
- A connected controller also counts as the heartbeat, so Default pose and Walk stay enabled without the page open.
- If the controller disconnects, or the helper stops, motion zeroes within 0.3 s. Without a page open, position control turns off after 1 s.
- Calibration and Servo setup ignore the controller.

### Head in Default pose

The four head channels are neck pitch, head pitch, head yaw and head roll (IDs 30–33).

- **Walk:** they are inputs to the walk policy, which decides how far the head follows.
- **Default pose:** there is no policy, so they move those joints directly from the default angles. ±1 gives ±30°, ±35°, ±60° and ±20° respectively, at most 2° per 20 ms tick (100°/s).

This applies whether the head is set from the Control page or the controller.

## Policies

Models in `python/`:

- `xgoduck_walk.onnx`
- `xgoduck_getup.onnx`
- `xgoduck_pick.onnx`
- `xgoduck_roulade.onnx`

Each expects observation `[1, 61]` and emits action `[1, 14]`. All four are loaded and warmed at startup. Spare time in the 50 Hz loop runs one dummy pass on an idle session so a switch does not pay a cold start.

Observation layout: gyro (rad/s), gravity, 14 joint positions relative to home, 14 joint velocities, previous action, twist (3), head (4), and six zeros.

Recovery, evaluated on the 50 Hz clock:

- Tilt above 55° for 0.15 s → get-up model, action forced to 0 for 1 s, then the get-up action. Twist and head are zero.
- Tilt below 15° for 1 s → walk, and the web twist/head/mouth commands apply again.
- Pick, only from upright walk: command `[cos(2πφ), sin(2πφ), 0]` plus ten zeros, `φ` advances by `dt/4` (a 4 s cycle). The mouth opens to 30° and closes when φ ≥ 0.4.
- Roulade, only from upright walk: runs for 1.9 s with twist, head, and the six trailing command inputs at 0. Gyro, gravity, joint position, joint velocity, and the previous action stay on the usual path. Leg and neck smoothing drop to α = 0.15 for that window, then return to the previous values. Tilt does not select get-up until the 1.9 s ends.

Action smoothing uses α = 0.45 on slices `[0:5]`, `[5:10]`, and `[10:14]`. `prev_action` stores the raw network output and is cleared on a policy switch. The 1 s fallen hold sends the home pose directly and does not keep a filtered walk action. Gyro smoothing α = 0.5, joint-velocity α = 0.4, gravity complementary-filter τ = 0.3 s. Accelerometer correction falls off when the norm leaves 1 g by more than about 20%. There is no gyro bias subtraction. The QMI axes are mapped once, in firmware; the host undoes the reference device mapping so the filter matches the Microduck math without applying that mapping twice.

Position control stops if commands are absent for 250 ms or the IMU is absent for 100 ms. Enabling again takes an explicit mode change. Host feedback older than 150 ms also disarms; that matches the MCU source-age limit.

## Layout

```text
app.yaml                 App Lab manifest
assets/                  Web UI
assets/cal/              CAD zero-pose renders for calibration (j<ID>.png)
assets/calguide.js       Zero-pose text and image per joint
python/                  Host policy, Bridge client, ONNX models
sketch/                  STM32 firmware
data/zero_pos.json       Created on the robot by calibration (not in git)
tools/                   Bridge patch, router rate, ELF check, read-only capture
host/                    Bluetooth gamepad helper and its systemd user unit (runs on the host)
data/controller.json     Controller mapping, saved from the Controller page (not in git)
data/joint_signs.json    Joints mounted reversed, set on the Check page (not in git)
data/tuning.json         Walk tuning (home pose, command smoothing, IMU level), saved from the Control page (not in git)
FORK.md                  What this fork changes from upstream
```

## Checks

```bash
python tools/measure.py --seconds 60
```

`tools/measure.py` only reads `http://127.0.0.1:9527/api/status`. It does not enable motors. Pass `--output` to save a JSON capture.

## References

- [Microduck](https://github.com/pollen-robotics/microduck)
- [Arduino RouterBridge](https://github.com/arduino-libraries/Arduino_RouterBridge)
- [Arduino Router](https://github.com/arduino/arduino-router)
