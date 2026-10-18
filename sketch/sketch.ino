#include <Arduino_RouterBridge.h>
#include <zephyr/kernel.h>
#include <math.h>
#include <string.h>
#include "duck_config.h"
#include "protocol.h"
#include "scs_bus.h"
#include "qmi8658.h"
#include "uart_fifo.h"

Motor motors[NUM_MOTORS];
struct k_mutex motorsMu;
// App sketches are Zephyr loadable extensions: initialize kernel objects at runtime.
// Avoid iterable kernel-object sections in the loadable sketch.
static struct k_mutex commandMu, stateMu, hostMu;
static struct k_sem reportReady;
K_THREAD_STACK_DEFINE(controlStack, 4096);
K_THREAD_STACK_DEFINE(reportStack, 4096);
static struct k_thread controlThread, reportThread;
static StatePacket snapshot = {};
static CommandPacket latest = {};
static bool pending = false;
static uint32_t invalidCommands = 0, reportDrops = 0;

static HostBusMode hostMode = HOST_NORMAL;
static CalPacket pendingCal = {};
static bool calPending = false;
static ServoPacket pendingServo = {};
static bool servoPending = false;
static ServoReplyPacket servoReply = {};
static bool servoReplyReady = false;
static bool rawNotifyEveryTick = false;
// Calibration energizes a joint only after the host first sends it a target.
static uint16_t calGoalMask = 0, calActiveMask = 0;
static uint16_t calGoal[NUM_MOTORS] = {};

static void receiveCommand(MsgPack::bin_t<uint8_t> bytes) {
  CommandPacket cmd;
  bool valid = bytes.size() == sizeof(cmd);
  if (valid) {
    memcpy(&cmd, bytes.data(), sizeof(cmd));
    valid = memcmp(cmd.magic, "DQC1", 4) == 0 && cmd.mode <= HOLD &&
            !cmd.reserved && !cmd.reserved2 &&
            cmd.ttlUs > 0 && cmd.ttlUs <= COMMAND_TTL_US;
    for (unsigned i = 0; i < NUM_MOTORS; ++i)
      valid = valid && isfinite(cmd.target[i]);
  }
  k_mutex_lock(&commandMu, K_FOREVER);
  if (valid) { latest = cmd; pending = true; }
  else ++invalidCommands;
  k_mutex_unlock(&commandMu);
}

static void receiveCal(MsgPack::bin_t<uint8_t> bytes) {
  CalPacket pkt;
  bool valid = bytes.size() == sizeof(pkt);
  if (valid) {
    memcpy(&pkt, bytes.data(), sizeof(pkt));
    valid = memcmp(pkt.magic, "DQL1", 4) == 0 && pkt.op <= CAL_SET_ZEROS;
    if (pkt.op == CAL_FINISH_ONE) valid = valid && pkt.index < NUM_MOTORS;
    if (pkt.op == CAL_SET_RAW || pkt.op == CAL_SET_ZEROS) {
      for (unsigned i = 0; i < NUM_MOTORS; ++i) {
        if (pkt.mask & (1u << i))
          valid = valid && pkt.raw[i] <= POS_MAX;
      }
    }
  }
  k_mutex_lock(&hostMu, K_FOREVER);
  if (valid) { pendingCal = pkt; calPending = true; }
  else {
    k_mutex_lock(&commandMu, K_FOREVER); ++invalidCommands; k_mutex_unlock(&commandMu);
  }
  k_mutex_unlock(&hostMu);
}

static void receiveServo(MsgPack::bin_t<uint8_t> bytes) {
  ServoPacket pkt;
  bool valid = bytes.size() == sizeof(pkt);
  if (valid) {
    memcpy(&pkt, bytes.data(), sizeof(pkt));
    valid = memcmp(pkt.magic, "DQV1", 4) == 0 && pkt.op <= SERVO_WRITE_LIMIT &&
            pkt.targetId <= 253;
    if (pkt.op == SERVO_GOTO) valid = valid && pkt.rawPos <= POS_MAX;
    if (pkt.op == SERVO_SET_ID) valid = valid && pkt.newId <= 253;
    if (pkt.op == SERVO_WRITE_LIMIT)
      valid = valid && (pkt.newId == ADDR_MIN_ANGLE || pkt.newId == ADDR_MAX_ANGLE ||
                        pkt.newId == ADDR_POS_OFFSET);
  }
  k_mutex_lock(&hostMu, K_FOREVER);
  if (valid) { pendingServo = pkt; servoPending = true; }
  else {
    k_mutex_lock(&commandMu, K_FOREVER); ++invalidCommands; k_mutex_unlock(&commandMu);
  }
  k_mutex_unlock(&hostMu);
}

static void fillRawPacket(RawPacket &r) {
  memcpy(r.magic, "DQR1", 4);
  r.mask = servoRxMask;
  k_mutex_lock(&motorsMu, K_FOREVER);
  for (unsigned i = 0; i < NUM_MOTORS; ++i) {
    r.pos[i] = motors[i].fbPos;
    r.zero[i] = motors[i].zeroPos;
  }
  k_mutex_unlock(&motorsMu);
}

static void calFinishOne(unsigned i);

static void applyCalOp(const CalPacket &pkt) {
  switch (pkt.op) {
    case CAL_ENTER:
      hostMode = HOST_CAL;
      rawNotifyEveryTick = true;
      setTorqueEnable(false);
      setTempKpKd(TEMP_KP_CAL, TEMP_KD_CAL);
      calGoalMask = calActiveMask = 0;
      break;
    case CAL_EXIT:
      hostMode = HOST_NORMAL;
      rawNotifyEveryTick = false;
      calGoalMask = calActiveMask = 0;
      setTorqueEnable(false);
      setRunKpKd();
      break;
    case CAL_SET_RAW:
      if (hostMode != HOST_CAL) break;
      for (unsigned i = 0; i < NUM_MOTORS; ++i) {
        if (pkt.mask & (1u << i)) calGoal[i] = satPos(pkt.raw[i]);
      }
      calGoalMask |= pkt.mask & ALL_MOTORS_MASK;
      break;
    case CAL_FINISH_ONE:
      if (hostMode != HOST_CAL || pkt.index >= NUM_MOTORS) break;
      calFinishOne(pkt.index);
      break;
    case CAL_SET_ZEROS:
      k_mutex_lock(&motorsMu, K_FOREVER);
      for (unsigned i = 0; i < NUM_MOTORS; ++i) {
        if (pkt.mask & (1u << i)) motors[i].zeroPos = satPos(pkt.raw[i]);
      }
      k_mutex_unlock(&motorsMu);
      break;
  }
}

static void readOneForReply(uint8_t id, ServoReplyPacket &reply) {
  drainServoRx();
  servoReadId = id;
  servoReadSeen = false;
  servoReadPos = 0;
  readSinglePresent(id);
  Serial1.flush();
  const uint32_t start = micros();
  while ((uint32_t)(micros() - start) < SERVO_REPLY_US) {
    parseServoRx();
    k_sleep(K_USEC(100));
  }
  parseServoRx();
  const int idx = motorIndexById(id);
  reply.ok = servoReadSeen ? 1 : 0;
  reply.rawPos = servoReadPos;
  servoReadId = 0xFF;
  if (idx >= 0) {
    k_mutex_lock(&motorsMu, K_FOREVER);
    reply.zeroPos = motors[idx].zeroPos;
    k_mutex_unlock(&motorsMu);
  }
}

static void readRegForReply(uint8_t id, uint8_t addr, ServoReplyPacket &reply) {
  drainServoRx();
  servoRegId = id;
  servoRegSeen = false;
  servoRegValue = 0;
  const uint8_t data[2] = {addr, 2};
  scsWriteBuf(id, 0x02, data, 2);
  Serial1.flush();
  const uint32_t start = micros();
  while ((uint32_t)(micros() - start) < SERVO_REPLY_US) {
    parseServoRx();
    k_sleep(K_USEC(100));
  }
  parseServoRx();
  reply.ok = servoRegSeen ? 1 : 0;
  reply.rawPos = servoRegValue;
  servoRegId = 0xFF;
}

// Store the joint's present position as its zero by shifting the servo's position
// offset so it reads CAL_ZERO_POS there. Zero then sits mid-range, ±180° from the
// encoder wrap. These servos report (encoder - offset) mod 4096.
static void calFinishOne(unsigned i) {
  const uint8_t id = motors[i].id;
  ServoReplyPacket off = {};
  readRegForReply(id, ADDR_POS_OFFSET, off);
  if (!off.ok) return;  // zero unchanged; the host sees no re-centred reading
  k_mutex_lock(&motorsMu, K_FOREVER);
  const uint16_t raw = motors[i].fbPos;
  k_mutex_unlock(&motorsMu);
  // Bit 15 marks a reading below 0 (sign-magnitude).
  const int32_t present = (raw & 0x8000u) ? -(int32_t)(raw & 0x7FFFu) : (int32_t)raw;
  const uint16_t newOffset = (uint16_t)(((int32_t)off.rawPos + present - CAL_ZERO_POS) & 0x0FFF);
  const bool held = calActiveMask & (1u << i);
  // Release the joint while its frame changes so its old goal cannot drive it.
  scsWriteByte(id, ADDR_TORQUE, 0);
  Serial1.flush();
  servoWriteLimit(id, ADDR_POS_OFFSET, newOffset);
  k_mutex_lock(&motorsMu, K_FOREVER);
  motors[i].zeroPos = CAL_ZERO_POS;
  motors[i].fbPos = CAL_ZERO_POS;
  motors[i].desPos = CAL_ZERO_POS;
  k_mutex_unlock(&motorsMu);
  calGoal[i] = CAL_ZERO_POS;
  if (held) {
    writeSingleGoalRaw(id, CAL_ZERO_POS);
    scsWriteByte(id, ADDR_TORQUE, 1);
    Serial1.flush();
  }
}

static void applyServoOp(const ServoPacket &pkt) {
  ServoReplyPacket reply = {};
  memcpy(reply.magic, "DQS2", 4);
  reply.op = pkt.op;
  reply.targetId = pkt.targetId;
  reply.ok = 1;
  switch (pkt.op) {
    case SERVO_ENTER:
      hostMode = HOST_SERVO_DEBUG;
      rawNotifyEveryTick = true;
      setTorqueEnable(false);
      setRunKpKd();
      break;
    case SERVO_EXIT:
      hostMode = HOST_NORMAL;
      rawNotifyEveryTick = false;
      setTorqueEnable(false);
      setRunKpKd();
      break;
    case SERVO_UNLOCK:
      if (hostMode != HOST_SERVO_DEBUG) { reply.ok = 0; break; }
      servoUnlock(pkt.targetId);
      break;
    case SERVO_SET_ID:
      if (hostMode != HOST_SERVO_DEBUG) { reply.ok = 0; break; }
      setTorqueEnable(false);
      servoUnlock(pkt.targetId);
      k_msleep(2);
      servoSetId(pkt.targetId, pkt.newId);
      break;
    case SERVO_GOTO:
      if (hostMode != HOST_SERVO_DEBUG) { reply.ok = 0; break; }
      setRunKpKd();
      setTorqueEnable(true);
      writeSingleGoalRaw(pkt.targetId, pkt.rawPos);
      Serial1.flush();
      break;
    case SERVO_SET_PERM_KP_KD:
      if (hostMode != HOST_SERVO_DEBUG) { reply.ok = 0; break; }
      servoUnlock(pkt.targetId);
      k_msleep(2);
      servoSetPermKpKd(pkt.targetId, pkt.kp, pkt.kd);
      break;
    case SERVO_PING: {
      if (hostMode != HOST_SERVO_DEBUG) { reply.ok = 0; break; }
      drainServoRx();
      servoPingId = pkt.targetId;
      servoPingSeen = false;
      scsWriteBuf(pkt.targetId, 0x01, nullptr, 0);
      Serial1.flush();
      const uint32_t start = micros();
      while ((uint32_t)(micros() - start) < SERVO_REPLY_US) {
        parseServoRx();
        k_sleep(K_USEC(100));
      }
      parseServoRx();
      reply.ok = servoPingSeen ? 1 : 0;
      servoPingId = 0xFF;
      break;
    }
    case SERVO_SET_MIDDLE:
      if (hostMode != HOST_SERVO_DEBUG) { reply.ok = 0; break; }
      // Torque off on every servo first so the offset change cannot move anything.
      setTorqueEnable(false);
      servoSetMiddle(pkt.targetId);
      readOneForReply(pkt.targetId, reply);
      break;
    case SERVO_READ_REG:
      if (hostMode != HOST_SERVO_DEBUG) { reply.ok = 0; break; }
      readRegForReply(pkt.targetId, pkt.newId, reply);
      break;
    case SERVO_WRITE_LIMIT:
      if (hostMode != HOST_SERVO_DEBUG) { reply.ok = 0; break; }
      setTorqueEnable(false);
      servoWriteLimit(pkt.targetId, pkt.newId, pkt.rawPos);
      readRegForReply(pkt.targetId, pkt.newId, reply);
      break;
    case SERVO_READ:
      if (hostMode != HOST_SERVO_DEBUG) { reply.ok = 0; break; }
      readOneForReply(pkt.targetId, reply);
      break;
  }
  k_mutex_lock(&hostMu, K_FOREVER);
  servoReply = reply;
  servoReplyReady = true;
  k_mutex_unlock(&hostMu);
}

static void controlTask(void*, void*, void*) {
  Serial1.begin(SERVO_BAUD);
  enableUartFifo(reinterpret_cast<USART_TypeDef*>(DT_REG_ADDR(DT_PHANDLE_BY_IDX(DT_PATH(zephyr_user), serials, 0))));
  for (unsigned i=0; i<3; ++i) { setTorqueEnable(false); k_msleep(5); }
  setRunKpKd();
  // Read temporary KP/KD once at boot (address 50, 2 bytes). Missing IDs stay unset.
  readAllRegisters(ADDR_TEMP_KP, 2); Serial1.flush();
  const uint32_t pdStart=micros();
  while ((uint32_t)(micros()-pdStart)<20000) { parseServoRx(); k_sleep(K_USEC(100)); }
  initImu();
  uint32_t deadline = micros(), previous = deadline, seq = 0, lastCommandUs = 0;
  uint32_t acceptedSeq = 0, ttl = COMMAND_TTL_US, overruns = 0, lastImuUs = 0;
  uint32_t lastHealthyUs = 0;
  bool enabled = false, timedOut = false, haveCommand = false;
  for (;;) {
    const uint32_t start = micros(), period = start - previous;
    previous = start;

    CalPacket calPkt;
    ServoPacket servoPkt;
    bool gotCal = false, gotServo = false;
    k_mutex_lock(&hostMu, K_FOREVER);
    if (calPending) { calPkt = pendingCal; calPending = false; gotCal = true; }
    if (servoPending) { servoPkt = pendingServo; servoPending = false; gotServo = true; }
    k_mutex_unlock(&hostMu);
    if (gotCal) applyCalOp(calPkt);
    if (gotServo) applyServoOp(servoPkt);

    CommandPacket cmd;
    k_mutex_lock(&commandMu, K_FOREVER);
    const bool got = pending;
    if (got) { cmd = latest; pending = false; }
    const uint32_t bad = invalidCommands;
    k_mutex_unlock(&commandMu);

    const bool busTaken = hostMode != HOST_NORMAL;
    if (got && !busTaken) {
      const bool fresh = cmd.mode == DISARM || (uint32_t)(start - cmd.sourceUs) <= MAX_SOURCE_AGE_US;
      const bool ordered = !haveCommand || cmd.mode == DISARM || (int32_t)(cmd.seq - acceptedSeq) > 0;
      if (fresh && ordered) {
        acceptedSeq = cmd.seq; haveCommand = true; lastCommandUs = start; ttl = cmd.ttlUs;
        if (cmd.mode == DISARM) { enabled = false; timedOut = false; }
        if (cmd.mode == ARM && !enabled && imu.ok &&
            (uint32_t)(start-lastImuUs) < 50000) {
          enabled = true; timedOut = false;
          lastHealthyUs = start;
          setRunKpKd();
        }
        k_mutex_lock(&motorsMu, K_FOREVER);
        for (unsigned i = 0; i < NUM_MOTORS; ++i) motors[i].desAngleDeg = cmd.target[i];
        k_mutex_unlock(&motorsMu);
      } else {
        k_mutex_lock(&commandMu, K_FOREVER); ++invalidCommands; k_mutex_unlock(&commandMu);
      }
    }
    if (busTaken) {
      enabled = false;
      timedOut = false;
    } else if (enabled && ((uint32_t)(start-lastCommandUs) > ttl ||
                           (uint32_t)(start-lastHealthyUs) > 100000)) {
      enabled = false; timedOut = true;
    }
    static bool previousEnabled = false;
    if (hostMode == HOST_CAL) {
      k_mutex_lock(&motorsMu, K_FOREVER);
      // Energize at most one new joint per tick, starting at its present position,
      // so the bus never sees a simultaneous inrush from every servo.
      const uint16_t waiting = calGoalMask & ~calActiveMask & servoRxMask;
      for (unsigned i = 0; i < NUM_MOTORS; ++i) {
        // A reading past the encoder range would be clamped and the joint would jump.
        if (!(waiting & (1u << i)) || motors[i].fbPos > POS_MAX) continue;
        motors[i].desPos = motors[i].fbPos;
        writeSingleGoalRaw(motors[i].id, motors[i].desPos);
        scsWriteByte(motors[i].id, ADDR_TORQUE, 1);
        calActiveMask |= 1u << i;
        break;
      }
      for (unsigned i = 0; i < NUM_MOTORS; ++i) {
        if (!(calActiveMask & (1u << i))) continue;
        const int32_t d = (int32_t)calGoal[i] - (int32_t)motors[i].desPos;
        const int32_t step = d > CAL_STEP_PER_TICK ? CAL_STEP_PER_TICK :
                             d < -(int32_t)CAL_STEP_PER_TICK ? -(int32_t)CAL_STEP_PER_TICK : d;
        motors[i].desPos = satPos((int32_t)motors[i].desPos + step);
      }
      sendMotorRawPos(calActiveMask);
      k_mutex_unlock(&motorsMu);
      Serial1.flush();
    } else if (enabled) {
      k_mutex_lock(&motorsMu, K_FOREVER); sendMotorDesPos(); k_mutex_unlock(&motorsMu);
      Serial1.flush();
    }
    if (!busTaken) {
      if (enabled != previousEnabled || !enabled) setTorqueEnable(enabled);
      previousEnabled = enabled;
    } else {
      previousEnabled = false;
    }
    drainServoRx();
    servoRxMask = 0;
    readAllState(); Serial1.flush();
    const uint32_t readStart = micros();
    if (updateImu()) lastImuUs = micros();
    while ((uint32_t)(micros()-readStart) < SERVO_REPLY_US) {
      parseServoRx(); k_sleep(K_USEC(100));
    }
    parseServoRx();
    if (imu.ok) lastHealthyUs = micros();
    StatePacket s = {};
    memcpy(s.magic, "DQS1", 4); s.version = 1;
    s.flags = (imu.ok ? 1 : 0) | (enabled ? 2 : 0) | (timedOut ? 4 : 0) |
              ((hostMode == HOST_CAL) ? 8 : 0) | ((hostMode == HOST_SERVO_DEBUG) ? 16 : 0);
    s.servoMask = servoRxMask; s.seq = ++seq; s.sampleUs = start; s.imuUs = lastImuUs;
    s.commandSeq = acceptedSeq; s.commandAgeUs = haveCommand ? start-lastCommandUs : UINT32_MAX;
    s.periodUs = period; s.overruns = overruns; s.invalidCommands = bad;
    memcpy(s.acc, imu.accRaw, sizeof(s.acc)); memcpy(s.gyro, imu.gyroRaw, sizeof(s.gyro));
    k_mutex_lock(&motorsMu, K_FOREVER);
    for (unsigned i = 0; i < NUM_MOTORS; ++i) {
      s.angle[i] = motors[i].fbAngleDeg; s.velocity[i] = motors[i].fbVelDeg;
      s.target[i] = motors[i].desAngleDeg;
    }
    k_mutex_unlock(&motorsMu);
    s.workUs = micros()-start;
    k_mutex_lock(&stateMu, K_FOREVER);
    if (k_sem_count_get(&reportReady)) ++reportDrops;
    s.reportDrops = reportDrops; snapshot = s;
    k_mutex_unlock(&stateMu);
    k_sem_give(&reportReady);
    deadline += TICK_US;
    const uint32_t now = micros();
    if ((int32_t)(deadline-now) <= 0) {
      const uint32_t missed = (now-deadline)/TICK_US+1;
      overruns += missed; deadline += missed*TICK_US;
    }
    const int32_t remaining = (int32_t)(deadline-micros());
    if (remaining > 0) k_sleep(K_USEC(remaining));
  }
}

static void reportTask(void*, void*, void*) {
  MsgPack::bin_t<uint8_t> bytes;
  unsigned reports = 0;
  bytes.resize(sizeof(StatePacket));
  for (;;) {
    k_sem_take(&reportReady, K_FOREVER);
    k_mutex_lock(&stateMu, K_FOREVER);
    memcpy(bytes.data(), &snapshot, sizeof(snapshot));
    k_mutex_unlock(&stateMu);
    Bridge.notify("duck_state", bytes);
    const bool wantRaw = rawNotifyEveryTick || ((reports % 20) == 0);
    if (wantRaw) {
      RawPacket raw = {};
      fillRawPacket(raw);
      MsgPack::bin_t<uint8_t> rawBytes;
      rawBytes.resize(sizeof(raw));
      memcpy(rawBytes.data(), &raw, sizeof(raw));
      Bridge.notify("duck_raw", rawBytes);
    }
    bool replyReady = false;
    ServoReplyPacket reply = {};
    k_mutex_lock(&hostMu, K_FOREVER);
    if (servoReplyReady) { reply = servoReply; servoReplyReady = false; replyReady = true; }
    k_mutex_unlock(&hostMu);
    if (replyReady) {
      MsgPack::bin_t<uint8_t> rb;
      rb.resize(sizeof(reply));
      memcpy(rb.data(), &reply, sizeof(reply));
      Bridge.notify("duck_servo_reply", rb);
    }
    if ((++reports % 100) == 0) {
      MsgPack::bin_t<uint8_t> pd;
      pd.resize(32);
      pd[0]=servoPdMask & 255; pd[1]=servoPdMask >> 8;
      memcpy(pd.data()+2, servoBootP, 15); memcpy(pd.data()+17, servoBootD, 15);
      Bridge.notify("duck_boot_pd", pd);
    }
  }
}

void setup() {
  k_mutex_init(&motorsMu);
  k_mutex_init(&commandMu);
  k_mutex_init(&stateMu);
  k_mutex_init(&hostMu);
  k_sem_init(&reportReady, 0, 1);
  for (unsigned i = 0; i < NUM_MOTORS; ++i) {
    motors[i] = {}; motors[i].id = MOTOR_IDS[i]; motors[i].zeroPos = ZERO_POS[i];
    motors[i].fbPos = ZERO_POS[i];
    motors[i].sign = MOTOR_SIGN[i]; motors[i].desAngleDeg = DEFAULT_DEG[i];
    motors[i].desPos = CAL_CENTER_POS;
  }
  // Start the watchdog/hardware loop even if Linux Bridge startup stalls.
  k_thread_create(&controlThread, controlStack, K_THREAD_STACK_SIZEOF(controlStack),
                  controlTask, nullptr, nullptr, nullptr, 4, 0, K_NO_WAIT);
  k_thread_name_set(&controlThread, "duck_control100");
  ARDUINO_ROUTER_SERIAL.begin(BRIDGE_BAUD);
  enableUartFifo(reinterpret_cast<USART_TypeDef*>(DT_REG_ADDR(ARDUINO_ROUTER_PHANDLE)));
  Bridge.begin(BRIDGE_BAUD);
  Bridge.provide("duck_command", receiveCommand);
  Bridge.provide("duck_cal", receiveCal);
  Bridge.provide("duck_servo", receiveServo);
  k_thread_create(&reportThread, reportStack, K_THREAD_STACK_SIZEOF(reportStack),
                  reportTask, nullptr, nullptr, nullptr, 6, 0, K_NO_WAIT);
  k_thread_name_set(&reportThread, "duck_report100");
  // GCC 14 otherwise emits a tail B.W to a flash syscall from RAM, outside
  // Thumb JUMP24 range; the Uno Q extension loader rejects it with ENOEXEC.
  asm volatile("nop" ::: "memory");
}
void loop() { delay(100); }
