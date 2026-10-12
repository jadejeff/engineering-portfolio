/*
  ESP32-C3 BLE 双轮小车 V14
  ================================================================
  版本目标：
  1. 上位机只使用 RPM，不再把 PWM 作为用户指令或显示量。
  2. Fxxx / Bxxx 中的 xxx 直接表示目标 RPM。
  3. 左右轮分别使用独立 PID，以各自编码器实测 RPM 闭环。
  4. PWM 仅作为底层执行量，保留在 ESP32 内部，不再通过网页显示。
  5. 根据实车测试修正左右测速通道：GPIO4=左轮，GPIO5=右轮。
  6. 使用约 300 ms 滑动计数窗计算转速，降低 20 脉冲/圈、100 ms
     单周期测速造成的 30 RPM 量化跳变。

  BLE 命令（每条以 \n 结束）：
    F300  前进，左右轮目标均为 300 RPM
    B250  后退，左右轮目标均为 250 RPM
    L     原地左转，使用固定目标 RPM，仍闭环调速
    R     原地右转，使用固定目标 RPM，仍闭环调速
    S     停止
    O     开启自动避障
    o     关闭自动避障并停止
    ?     查询状态

  BLE 遥测：
    S:298,302,T:300
      S：左、右轮实测 RPM
      T：当前统一目标 RPM

    D:18.5
      D：超声波距离，单位 cm
*/

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ============================================================
// 一、引脚
// ============================================================

// 电机驱动
const int IN1 = 0;
const int IN2 = 1;
const int IN3 = 10;
const int IN4 = 6;

// 测速通道
// 实车测试表明旧版本左右曲线对应反了，因此这里交换。
const int SPEED_LEFT_PIN  = 4;
const int SPEED_RIGHT_PIN = 5;

// 超声波
const int TRIG_PIN = 12;
const int ECHO_PIN = 7;

// 临时测速模块供电
const bool USE_GPIO_SENSOR_POWER = true;
const int SENSOR_POWER_PIN = 13;

// ============================================================
// 二、转速与控制参数
// ============================================================

// 手动转 10 圈均为 200 出头脉冲，因此左右均按 20 PPR 使用。
const float PULSES_PER_REV_LEFT  = 20.0f;
const float PULSES_PER_REV_RIGHT = 20.0f;

// 网页可下发的目标转速范围。
// 450 RPM 是当前保守上限，后续可根据慢轮的实测极限再提高。
const uint16_t MAX_COMMAND_RPM = 450;
const uint16_t DEFAULT_COMMAND_RPM = 300;

// 目标 RPM 到初始 PWM 的前馈估计。
// 它只负责把输出先送到大致工作点，最终速度仍由实测 RPM 闭环决定。
// 后续做完 PWM-RPM 开环表后，可分别修改左右两个数值。
const float ESTIMATED_FULL_PWM_RPM_LEFT  = 500.0f;
const float ESTIMATED_FULL_PWM_RPM_RIGHT = 500.0f;

// 最低克服静摩擦的有效 PWM。目标为 0 时不会使用该下限。
const int MIN_EFFECTIVE_PWM = 65;

// ==================== 左右独立 PID 初始值 ====================
// 单位：Kp = PWM/RPM，Ki = PWM/(RPM*s)，Kd = PWM*s/RPM。
// 先使用相同且偏保守的参数；左右轮仍是两个独立控制器。
float KP_LEFT = 0.15f;
float KI_LEFT = 0.16f;
float KD_LEFT = 0.01f;

float KP_RIGHT = 0.15f;
float KI_RIGHT = 0.16f;
float KD_RIGHT = 0.001f;

const float RPM_ERROR_DEADBAND = 3.0f;
const float TARGET_CHANGE_RESET_RPM = 20.0f;

// 100 ms 执行一次控制；用 3 个周期组成约 300 ms 测速窗。
// 20 PPR 时分辨率约为 10 RPM，比旧版单周期的 30 RPM 更适合闭环。
const uint32_t CONTROL_PERIOD_MS = 100;
const size_t SPEED_WINDOW_SAMPLES = 3;

// 滑动窗之后再做轻度低通，兼顾平滑与响应速度。
const float SPEED_FILTER_ALPHA = 0.45f;

// 过滤非常密集的毛刺。450 RPM、20 PPR 时真实脉冲间隔约 6.7 ms。
const uint32_t MIN_PULSE_INTERVAL_US = 2000;

// 超声波参数
const float OBSTACLE_DISTANCE_CM = 20.0f;
const uint32_t DISTANCE_PERIOD_MS = 80;

// 避障与转向均使用 RPM 目标，并继续闭环控制。
const uint16_t AUTO_FORWARD_RPM = 150;
const uint16_t AUTO_BACK_RPM = 150;
const uint32_t AUTO_BACK_TIME_MS = 500;

const bool AUTO_TURN_LEFT = true;
const uint16_t AUTO_TURN_RPM = 150;
const uint32_t AUTO_TURN_TIME_MS = 700;

const uint16_t MANUAL_TURN_RPM = 150;

const uint32_t TELEMETRY_PERIOD_MS = 100;

// ============================================================
// 三、BLE
// ============================================================

const char *BLE_NAME = "BT-CAR-JF";

#define SERVICE_UUID "0000fff0-0000-1000-8000-00805f9b34fb"
#define TX_UUID      "0000fff1-0000-1000-8000-00805f9b34fb"
#define RX_UUID      "0000fff2-0000-1000-8000-00805f9b34fb"

BLEServer *bleServer = nullptr;
BLECharacteristic *txCharacteristic = nullptr;

volatile bool bleConnected = false;
volatile bool stopRequested = false;

String rxBuffer;

// ============================================================
// 四、运行状态
// ============================================================

enum RunMode {
  MODE_STOP,
  MODE_MANUAL,
  MODE_AUTO
};

enum ManualAction {
  MANUAL_STOP,
  MANUAL_FORWARD,
  MANUAL_BACKWARD,
  MANUAL_LEFT,
  MANUAL_RIGHT
};

enum AutoState {
  AUTO_FORWARD,
  AUTO_BACKWARD,
  AUTO_TURN
};

RunMode runMode = MODE_STOP;
ManualAction manualAction = MANUAL_STOP;
AutoState autoState = AUTO_FORWARD;

uint16_t targetRpmCommand = DEFAULT_COMMAND_RPM;

uint32_t autoStateStartMs = 0;
uint8_t obstacleConfirmCount = 0;

// ============================================================
// 五、测速与输出状态
// ============================================================

volatile uint32_t leftPulseCount = 0;
volatile uint32_t rightPulseCount = 0;
volatile uint32_t lastLeftPulseUs = 0;
volatile uint32_t lastRightPulseUs = 0;

uint32_t leftPulseHistory[SPEED_WINDOW_SAMPLES] = {0};
uint32_t rightPulseHistory[SPEED_WINDOW_SAMPLES] = {0};
uint32_t elapsedHistoryMs[SPEED_WINDOW_SAMPLES] = {0};
size_t speedWindowIndex = 0;
size_t speedWindowCount = 0;
uint32_t leftPulseWindowTotal = 0;
uint32_t rightPulseWindowTotal = 0;
uint32_t elapsedWindowTotalMs = 0;

float speedLeftRPM = 0.0f;
float speedRightRPM = 0.0f;
float distanceCm = -1.0f;

// PWM 仅作为底层执行量，不再发送给网页。
int actualPwmLeft = 0;
int actualPwmRight = 0;

uint32_t lastControlMs = 0;
uint32_t lastDistanceMs = 0;
uint32_t lastTelemetryMs = 0;
uint32_t lastDistanceTelemetryMs = 0;

// ============================================================
// 六、PID 控制器
// ============================================================

struct PIDController {
  float integral = 0.0f;
  float lastMeasuredRPM = 0.0f;
  float filteredDerivative = 0.0f;
  float lastTargetRPM = 0.0f;
  bool initialized = false;

  void reset() {
    integral = 0.0f;
    lastMeasuredRPM = 0.0f;
    filteredDerivative = 0.0f;
    lastTargetRPM = 0.0f;
    initialized = false;
  }

  int update(
      float targetRPM,
      float measuredRPM,
      float feedforwardPwm,
      float dt,
      float kp,
      float ki,
      float kd) {

    if (targetRPM <= 0.0f || dt <= 0.0f) {
      reset();
      return 0;
    }

    if (initialized &&
        fabsf(targetRPM - lastTargetRPM) >= TARGET_CHANGE_RESET_RPM) {
      // 目标速度明显改变时清除旧积分，避免旧工况拖累新响应。
      integral = 0.0f;
      filteredDerivative = 0.0f;
      initialized = false;
    }

    float error = targetRPM - measuredRPM;
    if (fabsf(error) < RPM_ERROR_DEADBAND) {
      error = 0.0f;
    }

    float rawDerivative = 0.0f;
    if (initialized) {
      // 对测量值求微分，避免目标阶跃造成微分冲击。
      rawDerivative = -(measuredRPM - lastMeasuredRPM) / dt;
    } else {
      initialized = true;
    }

    filteredDerivative =
        0.20f * rawDerivative +
        0.80f * filteredDerivative;

    const float pTerm = kp * error;
    const float dTerm = kd * filteredDerivative;

    float candidateIntegral = integral + error * dt;

    // 根据 Ki 给积分状态设置安全边界。
    if (ki > 0.0001f) {
      const float integralLimit = 255.0f / ki;
      candidateIntegral = constrain(
          candidateIntegral,
          -integralLimit,
          integralLimit);
    } else {
      candidateIntegral = 0.0f;
    }

    const float candidateOutput =
        feedforwardPwm +
        pTerm +
        ki * candidateIntegral +
        dTerm;

    // 输出饱和时，仅允许积分朝解除饱和的方向发展。
    const bool allowIntegral =
        (candidateOutput > 0.0f && candidateOutput < 255.0f) ||
        (candidateOutput >= 255.0f && error < 0.0f) ||
        (candidateOutput <= 0.0f && error > 0.0f);

    if (allowIntegral) {
      integral = candidateIntegral;
    }

    float output =
        feedforwardPwm +
        pTerm +
        ki * integral +
        dTerm;

    output = constrain(output, 0.0f, 255.0f);

    if (output > 0.0f && output < MIN_EFFECTIVE_PWM) {
      output = MIN_EFFECTIVE_PWM;
    }

    lastMeasuredRPM = measuredRPM;
    lastTargetRPM = targetRPM;

    return (int)lroundf(output);
  }
};

PIDController pidLeft;
PIDController pidRight;

// ============================================================
// 七、左右轮目标
// ============================================================

struct WheelTarget {
  float targetRPM;
  int direction;  // 1 正转，-1 反转，0 停止
};

WheelTarget leftTarget = {0.0f, 0};
WheelTarget rightTarget = {0.0f, 0};

int lastLeftDirection = 0;
int lastRightDirection = 0;

float rpmToFeedforwardPwm(float targetRPM, float estimatedFullPwmRPM) {
  if (targetRPM <= 0.0f || estimatedFullPwmRPM <= 0.0f) {
    return 0.0f;
  }

  return constrain(
      targetRPM * 255.0f / estimatedFullPwmRPM,
      0.0f,
      255.0f);
}

// ============================================================
// 八、测速中断
// ============================================================

void IRAM_ATTR leftSpeedISR() {
  const uint32_t nowUs = micros();

  if (nowUs - lastLeftPulseUs >= MIN_PULSE_INTERVAL_US) {
    leftPulseCount++;
    lastLeftPulseUs = nowUs;
  }
}

void IRAM_ATTR rightSpeedISR() {
  const uint32_t nowUs = micros();

  if (nowUs - lastRightPulseUs >= MIN_PULSE_INTERVAL_US) {
    rightPulseCount++;
    lastRightPulseUs = nowUs;
  }
}

// ============================================================
// 九、电机底层
// ============================================================

void setRightMotor(int direction, int pwm) {
  pwm = constrain(pwm, 0, 255);

  if (direction > 0) {
    analogWrite(IN1, 0);
    analogWrite(IN2, pwm);
  } else if (direction < 0) {
    analogWrite(IN1, pwm);
    analogWrite(IN2, 0);
  } else {
    analogWrite(IN1, 0);
    analogWrite(IN2, 0);
  }
}

void setLeftMotor(int direction, int pwm) {
  pwm = constrain(pwm, 0, 255);

  if (direction > 0) {
    analogWrite(IN3, pwm);
    analogWrite(IN4, 0);
  } else if (direction < 0) {
    analogWrite(IN3, 0);
    analogWrite(IN4, pwm);
  } else {
    analogWrite(IN3, 0);
    analogWrite(IN4, 0);
  }
}

void stopMotors() {
  actualPwmLeft = 0;
  actualPwmRight = 0;

  setLeftMotor(0, 0);
  setRightMotor(0, 0);

  pidLeft.reset();
  pidRight.reset();
}

// ============================================================
// 十、BLE 发送
// ============================================================

void notifyText(const String &text) {
  if (!bleConnected || txCharacteristic == nullptr) {
    return;
  }

  txCharacteristic->setValue(text);
  txCharacteristic->notify();
}

void sendAck(const String &command) {
  notifyText("OK," + command + "\n");
}

void sendError(const String &message) {
  notifyText("ERR," + message + "\n");
}

// ============================================================
// 十一、命令处理
// ============================================================

bool parseTargetRPM(const String &command, uint16_t &value) {
  if (command.length() < 2) {
    return false;
  }

  for (size_t i = 1; i < command.length(); i++) {
    if (!isDigit(command[i])) {
      return false;
    }
  }

  const long number = command.substring(1).toInt();
  value = (uint16_t)constrain(number, 0L, (long)MAX_COMMAND_RPM);
  return true;
}

void requestStop() {
  runMode = MODE_STOP;
  manualAction = MANUAL_STOP;
  stopRequested = true;
}

void processCommand(String command) {
  command.trim();

  if (command.length() == 0) {
    return;
  }

  const char first = command[0];

  if (first == 'F' || first == 'B') {
    uint16_t newTargetRPM = 0;

    if (!parseTargetRPM(command, newTargetRPM)) {
      sendError("PARAM");
      return;
    }

    if (newTargetRPM == 0) {
      requestStop();
      sendAck("S");
      return;
    }

    targetRpmCommand = newTargetRPM;
    runMode = MODE_MANUAL;
    manualAction =
        (first == 'F')
        ? MANUAL_FORWARD
        : MANUAL_BACKWARD;

    sendAck(String(first) + String(targetRpmCommand));
    return;
  }

  switch (first) {
    case 'L':
      runMode = MODE_MANUAL;
      manualAction = MANUAL_LEFT;
      sendAck("L");
      break;

    case 'R':
      runMode = MODE_MANUAL;
      manualAction = MANUAL_RIGHT;
      sendAck("R");
      break;

    case 'S':
      requestStop();
      sendAck("S");
      break;

    case 'O':
      runMode = MODE_AUTO;
      autoState = AUTO_FORWARD;
      autoStateStartMs = millis();
      obstacleConfirmCount = 0;
      sendAck("O");
      break;

    case 'o':
      requestStop();
      sendAck("o");
      break;

    case '?': {
      const int modeCode =
          runMode == MODE_AUTO ? 2 :
          runMode == MODE_MANUAL ? 1 : 0;

      notifyText(
          String("V:14,M:") + String(modeCode) +
          ",T:" + String((int)targetRpmCommand) + "\n");
      break;
    }

    default:
      sendError("UNKNOWN");
      break;
  }
}

void feedReceivedText(const String &data) {
  for (size_t i = 0; i < data.length(); i++) {
    const char c = data[i];

    if (c == '\n') {
      const String command = rxBuffer;
      rxBuffer = "";
      processCommand(command);
    } else if (c != '\r') {
      if (rxBuffer.length() < 50) {
        rxBuffer += c;
      } else {
        rxBuffer = "";
        sendError("TOO_LONG");
      }
    }
  }
}

// ============================================================
// 十二、BLE 回调
// ============================================================

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *server) override {
    bleConnected = true;
  }

  void onDisconnect(BLEServer *server) override {
    bleConnected = false;
    requestStop();
    server->startAdvertising();
  }
};

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    const String data = String(characteristic->getValue().c_str());
    feedReceivedText(data);
  }
};

void initBLE() {
  BLEDevice::init(BLE_NAME);

  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new ServerCallbacks());

  BLEService *service = bleServer->createService(SERVICE_UUID);

  txCharacteristic = service->createCharacteristic(
      TX_UUID,
      BLECharacteristic::PROPERTY_NOTIFY |
      BLECharacteristic::PROPERTY_READ);

  txCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *rxCharacteristic = service->createCharacteristic(
      RX_UUID,
      BLECharacteristic::PROPERTY_WRITE |
      BLECharacteristic::PROPERTY_WRITE_NR);

  rxCharacteristic->setCallbacks(new RxCallbacks());

  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);

  BLEDevice::startAdvertising();
}

// ============================================================
// 十三、超声波与自动避障
// ============================================================

float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  const uint32_t echoTime = pulseIn(ECHO_PIN, HIGH, 25000UL);

  if (echoTime == 0) {
    return -1.0f;
  }

  return echoTime * 0.0343f / 2.0f;
}

void updateDistance() {
  const uint32_t nowMs = millis();

  if (nowMs - lastDistanceMs < DISTANCE_PERIOD_MS) {
    return;
  }

  lastDistanceMs = nowMs;
  distanceCm = readDistanceCm();

  if (runMode != MODE_AUTO ||
      autoState != AUTO_FORWARD ||
      distanceCm <= 0.0f) {
    obstacleConfirmCount = 0;
    return;
  }

  if (distanceCm < OBSTACLE_DISTANCE_CM) {
    obstacleConfirmCount++;
  } else {
    obstacleConfirmCount = 0;
  }

  if (obstacleConfirmCount >= 2) {
    obstacleConfirmCount = 0;
    autoState = AUTO_BACKWARD;
    autoStateStartMs = nowMs;
    notifyText("OBSTACLE\n");
  }
}

void updateAutoState() {
  if (runMode != MODE_AUTO) {
    return;
  }

  const uint32_t elapsed = millis() - autoStateStartMs;

  switch (autoState) {
    case AUTO_FORWARD:
      break;

    case AUTO_BACKWARD:
      if (elapsed >= AUTO_BACK_TIME_MS) {
        autoState = AUTO_TURN;
        autoStateStartMs = millis();
      }
      break;

    case AUTO_TURN:
      if (elapsed >= AUTO_TURN_TIME_MS) {
        autoState = AUTO_FORWARD;
        autoStateStartMs = millis();
      }
      break;
  }
}

// ============================================================
// 十四、生成左右轮 RPM 目标
// ============================================================

WheelTarget makeTarget(float targetRPM, int direction) {
  WheelTarget target;
  target.targetRPM = constrain(targetRPM, 0.0f, (float)MAX_COMMAND_RPM);
  target.direction = direction;
  return target;
}

void updateTargets() {
  leftTarget = makeTarget(0.0f, 0);
  rightTarget = makeTarget(0.0f, 0);

  if (runMode == MODE_STOP) {
    return;
  }

  if (runMode == MODE_MANUAL) {
    switch (manualAction) {
      case MANUAL_FORWARD:
        leftTarget = makeTarget(targetRpmCommand, 1);
        rightTarget = makeTarget(targetRpmCommand, 1);
        break;

      case MANUAL_BACKWARD:
        leftTarget = makeTarget(targetRpmCommand, -1);
        rightTarget = makeTarget(targetRpmCommand, -1);
        break;

      case MANUAL_LEFT:
        leftTarget = makeTarget(MANUAL_TURN_RPM, 1);
        rightTarget = makeTarget(MANUAL_TURN_RPM, -1);
        break;

      case MANUAL_RIGHT:
        leftTarget = makeTarget(MANUAL_TURN_RPM, -1);
        rightTarget = makeTarget(MANUAL_TURN_RPM, 1);
        break;

      case MANUAL_STOP:
      default:
        break;
    }

    return;
  }

  switch (autoState) {
    case AUTO_FORWARD:
      leftTarget = makeTarget(AUTO_FORWARD_RPM, 1);
      rightTarget = makeTarget(AUTO_FORWARD_RPM, 1);
      break;

    case AUTO_BACKWARD:
      leftTarget = makeTarget(AUTO_BACK_RPM, -1);
      rightTarget = makeTarget(AUTO_BACK_RPM, -1);
      break;

    case AUTO_TURN:
      if (AUTO_TURN_LEFT) {
        leftTarget = makeTarget(AUTO_TURN_RPM, 1);
        rightTarget = makeTarget(AUTO_TURN_RPM, -1);
      } else {
        leftTarget = makeTarget(AUTO_TURN_RPM, -1);
        rightTarget = makeTarget(AUTO_TURN_RPM, 1);
      }
      break;
  }
}

// ============================================================
// 十五、滑动窗测速与闭环控制
// ============================================================

void updateSpeedEstimate(
    uint32_t leftPulses,
    uint32_t rightPulses,
    uint32_t elapsedMs) {

  // 移除环形缓冲区中将被覆盖的旧数据。
  leftPulseWindowTotal -= leftPulseHistory[speedWindowIndex];
  rightPulseWindowTotal -= rightPulseHistory[speedWindowIndex];
  elapsedWindowTotalMs -= elapsedHistoryMs[speedWindowIndex];

  // 加入本周期数据。
  leftPulseHistory[speedWindowIndex] = leftPulses;
  rightPulseHistory[speedWindowIndex] = rightPulses;
  elapsedHistoryMs[speedWindowIndex] = elapsedMs;

  leftPulseWindowTotal += leftPulses;
  rightPulseWindowTotal += rightPulses;
  elapsedWindowTotalMs += elapsedMs;

  speedWindowIndex = (speedWindowIndex + 1) % SPEED_WINDOW_SAMPLES;
  if (speedWindowCount < SPEED_WINDOW_SAMPLES) {
    speedWindowCount++;
  }

  if (elapsedWindowTotalMs == 0) {
    return;
  }

  const float rawLeftRPM =
      leftPulseWindowTotal * 60000.0f /
      (PULSES_PER_REV_LEFT * elapsedWindowTotalMs);

  const float rawRightRPM =
      rightPulseWindowTotal * 60000.0f /
      (PULSES_PER_REV_RIGHT * elapsedWindowTotalMs);

  // 窗内完全无脉冲时直接归零，避免滤波尾巴拖得过长。
  if (leftPulseWindowTotal == 0) {
    speedLeftRPM = 0.0f;
  } else {
    speedLeftRPM =
        SPEED_FILTER_ALPHA * rawLeftRPM +
        (1.0f - SPEED_FILTER_ALPHA) * speedLeftRPM;
  }

  if (rightPulseWindowTotal == 0) {
    speedRightRPM = 0.0f;
  } else {
    speedRightRPM =
        SPEED_FILTER_ALPHA * rawRightRPM +
        (1.0f - SPEED_FILTER_ALPHA) * speedRightRPM;
  }
}

void updateControl() {
  const uint32_t nowMs = millis();
  const uint32_t elapsedMs = nowMs - lastControlMs;

  if (elapsedMs < CONTROL_PERIOD_MS) {
    return;
  }

  lastControlMs = nowMs;
  const float dt = elapsedMs / 1000.0f;

  uint32_t leftPulses = 0;
  uint32_t rightPulses = 0;

  noInterrupts();
  leftPulses = leftPulseCount;
  rightPulses = rightPulseCount;
  leftPulseCount = 0;
  rightPulseCount = 0;
  interrupts();

  updateSpeedEstimate(leftPulses, rightPulses, elapsedMs);
  updateTargets();

  if (leftTarget.direction != lastLeftDirection) {
    pidLeft.reset();
    lastLeftDirection = leftTarget.direction;
  }

  if (rightTarget.direction != lastRightDirection) {
    pidRight.reset();
    lastRightDirection = rightTarget.direction;
  }

  if (leftTarget.direction == 0 && rightTarget.direction == 0) {
    stopMotors();
    return;
  }

  const float leftFeedforward = rpmToFeedforwardPwm(
      leftTarget.targetRPM,
      ESTIMATED_FULL_PWM_RPM_LEFT);

  const float rightFeedforward = rpmToFeedforwardPwm(
      rightTarget.targetRPM,
      ESTIMATED_FULL_PWM_RPM_RIGHT);

  actualPwmLeft = pidLeft.update(
      leftTarget.targetRPM,
      speedLeftRPM,
      leftFeedforward,
      dt,
      KP_LEFT,
      KI_LEFT,
      KD_LEFT);

  actualPwmRight = pidRight.update(
      rightTarget.targetRPM,
      speedRightRPM,
      rightFeedforward,
      dt,
      KP_RIGHT,
      KI_RIGHT,
      KD_RIGHT);

  setLeftMotor(leftTarget.direction, actualPwmLeft);
  setRightMotor(rightTarget.direction, actualPwmRight);
}

// ============================================================
// 十六、仅发送 RPM 与距离遥测
// ============================================================

float getCurrentTargetRPM() {
  if (leftTarget.direction == 0 && rightTarget.direction == 0) {
    return 0.0f;
  }

  return (leftTarget.targetRPM + rightTarget.targetRPM) / 2.0f;
}

void sendTelemetry() {
  const uint32_t nowMs = millis();

  if (nowMs - lastTelemetryMs < TELEMETRY_PERIOD_MS) {
    return;
  }

  lastTelemetryMs = nowMs;

  const int leftRpmReport = max(0, (int)lroundf(speedLeftRPM));
  const int rightRpmReport = max(0, (int)lroundf(speedRightRPM));
  const int targetRpmReport = max(0, (int)lroundf(getCurrentTargetRPM()));

  notifyText(
      String("S:") + String(leftRpmReport) +
      "," + String(rightRpmReport) +
      ",T:" + String(targetRpmReport) + "\n");

  if (nowMs - lastDistanceTelemetryMs >= 500) {
    lastDistanceTelemetryMs = nowMs;

    if (distanceCm > 0.0f) {
      notifyText(String("D:") + String(distanceCm, 1) + "\n");
    }
  }
}

// ============================================================
// 十七、setup 与 loop
// ============================================================

void setup() {
  Serial.begin(115200);

  if (USE_GPIO_SENSOR_POWER) {
    pinMode(SENSOR_POWER_PIN, OUTPUT);
    digitalWrite(SENSOR_POWER_PIN, HIGH);
    delay(100);
  }

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(SPEED_LEFT_PIN, INPUT_PULLUP);
  pinMode(SPEED_RIGHT_PIN, INPUT_PULLUP);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  stopMotors();

  attachInterrupt(
      digitalPinToInterrupt(SPEED_LEFT_PIN),
      leftSpeedISR,
      RISING);

  attachInterrupt(
      digitalPinToInterrupt(SPEED_RIGHT_PIN),
      rightSpeedISR,
      RISING);

  initBLE();

  lastControlMs = millis();
  lastDistanceMs = millis();
  lastTelemetryMs = millis();
  lastDistanceTelemetryMs = millis();
}

void loop() {
  if (stopRequested) {
    stopRequested = false;
    stopMotors();
  }

  updateDistance();
  updateAutoState();
  updateControl();
  sendTelemetry();

  delay(1);
}
