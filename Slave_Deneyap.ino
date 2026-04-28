/**
 * @file Slave_Deneyap.ino
 * @brief Robot Control Receiver - ESP-NOW Version
 *
 * This firmware runs on the Deneyap Kart physically controlling the robot.
 * It has been modified to receive control commands via the ESP-NOW protocol
 * instead of an HTTP server, significantly reducing communication latency.
 *
 * @version 4.1 (Error Fixes)
 */

// Core ESP-NOW and WiFi libraries
#include <esp_now.h>
#include <WiFi.h>

// Other necessary libraries from the original project
#include <SoftwareSerial.h>
#include <esp_task_wdt.h>

// =================================================================
//      COMMUNICATION & DATA STRUCTURE
// =================================================================

// This struct MUST EXACTLY match the one in your QT application (ControlData.h)
// and the Master_Deneyap.ino sketch.
#pragma pack(push, 1)
struct ControlData {
    // Car Control
    int16_t left_motor_speed;
    int16_t right_motor_speed;

    // Arm Control - Physical angles for each servo
    float arm_base_angle;
    float arm_shoulder_angle;
    float arm_elbow_angle;

    // Gripper and Dumper Control - Simple on/off states
    uint8_t gripper_state;
    uint8_t dumper_state;

    // Flags (kept for struct compatibility)
    uint8_t reset_arm_flag;
    uint8_t auto_release_flag;
};
#pragma pack(pop)

// Global variable to hold the latest data received via ESP-NOW
volatile ControlData incomingData;
// A copy of the last processed data to detect changes (prevents command spam)
ControlData lastProcessedData;

// Flag to indicate that new data has arrived from the callback
volatile bool newDataAvailable = false;


// =================================================================
//      HARDWARE & PIN CONFIGURATION
// =================================================================

// Use SoftwareSerial for STM32 comms
SoftwareSerial stm32Serial(D14, D13); // RX, TX

// Left side motor pins (Driver 1)
const uint8_t leftForward = A4;
const uint8_t leftBackward = A5;
const int R_EN_LEFT = D7;
const int L_EN_LEFT = D9;

// Right side motor pins (Driver 2)
const uint8_t rightForward = D0;
const uint8_t rightBackward = D1;
const int R_EN_RIGHT = D4;
const int L_EN_RIGHT = D6;

// =================================================================
//      SYSTEM STATE & TIMERS
// =================================================================

// Motor state structure (unchanged)
struct MotorState {
  int currentLeft = 0, currentRight = 0;
  int targetLeft = 0, targetRight = 0;
  bool isMoving = false;
} motors;

// System configuration (unchanged)
hw_timer_t *motorTimer = NULL;
volatile bool motorUpdateFlag = false;
const uint8_t ACCEL_STEP = 12, DECEL_STEP = 15;
bool DEBUG = true;
volatile unsigned long lastDataMillis = 0;

// Non-blocking STM32 command system (unchanged)
struct STM32Command {
  String command;
  unsigned long timestamp;
  bool sent;
};

const int STM32_QUEUE_SIZE = 16;
STM32Command stm32Queue[STM32_QUEUE_SIZE];
volatile int queueHead = 0;
volatile int queueTail = 0;
volatile int queueCount = 0;
unsigned long lastSTM32Send = 0;
const unsigned long STM32_SEND_INTERVAL = 20;


// =================================================================
//      ESP-NOW CALLBACK
// =================================================================

// This function is called every time an ESP-NOW packet is received.
void OnDataRecv(const uint8_t * mac, const uint8_t *data, int len) {
  if (len == sizeof(incomingData)) {
    memcpy((void*)&incomingData, data, sizeof(incomingData));
    newDataAvailable = true;
  }
}

// =================================================================
//      SETUP
// =================================================================

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== Robot Control System v4.1 (ESP-NOW) ===");

  esp_task_wdt_init(5, true);
  esp_task_wdt_add(NULL);

  setupPins();
  setupMotorTimer();

  stm32Serial.begin(57600);
  delay(100);
  flushSTM32Response();
  initializeSTM32Queue();

  setupEspNow();

  enqueueSTM32Command("verbose off");
  enqueueSTM32Command("mode coord");

  Serial.println("ESP-NOW Receiver Initialized. Waiting for commands...");
  Serial.printf("MAC Address: %s\n", WiFi.macAddress().c_str());
  Serial.printf("Free heap: %u bytes\n", ESP.getFreeHeap());
}


// =================================================================
//      MAIN LOOP
// =================================================================

void loop() {
  esp_task_wdt_reset();

  if (newDataAvailable) {
    processIncomingData();
  }

  if (motorUpdateFlag) {
    motorUpdateFlag = false;
    updateMotors();
  }

  processSTM32Queue();

  if (motors.isMoving && (millis() - lastDataMillis > 1000)) {
    gradualStop();
    if (DEBUG) Serial.println("[SAFETY] ESP-NOW timeout - stopping motors");
  }

  if (stm32Serial.available() > 64) {
    flushSTM32Response();
  }

  vTaskDelay(1);
}

// =================================================================
//      INITIALIZATION FUNCTIONS
// =================================================================

void setupPins() {
  pinMode(rightForward, OUTPUT);
  pinMode(rightBackward, OUTPUT);
  pinMode(leftForward, OUTPUT);
  pinMode(leftBackward, OUTPUT);
  pinMode(R_EN_LEFT, OUTPUT);
  pinMode(L_EN_LEFT, OUTPUT);
  pinMode(R_EN_RIGHT, OUTPUT);
  pinMode(L_EN_RIGHT, OUTPUT);

  digitalWrite(R_EN_LEFT, HIGH);
  digitalWrite(L_EN_LEFT, HIGH);
  digitalWrite(R_EN_RIGHT, HIGH);
  digitalWrite(L_EN_RIGHT, HIGH);
}

// **FIX 1**: The 'onMotorTimer' function is now defined before it's used in 'setupMotorTimer'.
void IRAM_ATTR onMotorTimer() {
  motorUpdateFlag = true;
}

void setupMotorTimer() {
  motorTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(motorTimer, &onMotorTimer, true);
  timerAlarmWrite(motorTimer, 25000, true); // 25ms = 40Hz
  timerAlarmEnable(motorTimer);
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
}

void initializeSTM32Queue() {
  for (int i = 0; i < STM32_QUEUE_SIZE; i++) {
    stm32Queue[i].command = "";
    stm32Queue[i].timestamp = 0;
    stm32Queue[i].sent = false;
  }
}

// =================================================================
//      DATA & COMMAND PROCESSING
// =================================================================

void processIncomingData() {
    // Reset the flag first to be ready for the next packet
    newDataAvailable = false;

    // **FIX 2**: Create a local, non-volatile copy of the data. This prevents race
    // conditions and fixes the 'volatile' assignment compiler error.
    ControlData dataCopy;
    
    // Atomically copy the volatile data to our local variable
    noInterrupts(); // Disable interrupts to prevent data corruption during copy
    memcpy(&dataCopy, (const void*)&incomingData, sizeof(ControlData));
    interrupts();   // Re-enable interrupts immediately after the copy

    // From now on, use 'dataCopy' for all processing in this function.
    lastDataMillis = millis();

    // 1. Update Car Motor Speeds
    motors.targetLeft = dataCopy.left_motor_speed;
    motors.targetRight = dataCopy.right_motor_speed;

    Serial.print("New data: ");
    Serial.print(motors.targetLeft);
    Serial.print("New data: ");
    Serial.print(motors.targetRight);
    Serial.println("");

    // 2. Update Arm Angles (only if they have changed)
    if (abs(dataCopy.arm_base_angle - lastProcessedData.arm_base_angle) > 0.1) {
        String cmd = "b " + String(dataCopy.arm_base_angle, 1);
        enqueueSTM32Command(cmd);
    }
    if (abs(dataCopy.arm_shoulder_angle - lastProcessedData.arm_shoulder_angle) > 0.1) {
        String cmd = "s " + String(dataCopy.arm_shoulder_angle, 1);
        enqueueSTM32Command(cmd);
    }
    if (abs(dataCopy.arm_elbow_angle - lastProcessedData.arm_elbow_angle) > 0.1) {
        String cmd = "e " + String(dataCopy.arm_elbow_angle, 1);
        enqueueSTM32Command(cmd);
    }

    // 3. Update Gripper State (only if it has changed)
    if (dataCopy.gripper_state != lastProcessedData.gripper_state) {
        enqueueSTM32Command(dataCopy.gripper_state == 1 ? "g 0" : "g 40");
    }

    // 4. Update Dumper State (only if it has changed)
    if (dataCopy.dumper_state != lastProcessedData.dumper_state) {
        enqueueSTM32Command(dataCopy.dumper_state == 1 ? "d 0" : "d 90");
    }
    
    // 5. This assignment is now valid (non-volatile to non-volatile).
    lastProcessedData = dataCopy;
}


// =================================================================
//      MOTOR CONTROL (Unchanged)
// =================================================================

void setMotor(uint8_t forwardPin, uint8_t backwardPin, int speed) {
  if (speed > 0) {
    analogWrite(forwardPin, speed);
    analogWrite(backwardPin, 0);
  } else if (speed < 0) {
    analogWrite(forwardPin, 0);
    analogWrite(backwardPin, -speed);
  } else {
    analogWrite(forwardPin, 0);
    analogWrite(backwardPin, 0);
  }
}

void updateMotors() {
  bool changed = false;
  if (motors.currentLeft != motors.targetLeft) {
    int diff = motors.targetLeft - motors.currentLeft;
    int step = (diff > 0) ? ACCEL_STEP : -DECEL_STEP;
    if (abs(diff) <= abs(step)) {
      motors.currentLeft = motors.targetLeft;
    } else {
      motors.currentLeft += step;
    }
    changed = true;
  }
  if (motors.currentRight != motors.targetRight) {
    int diff = motors.targetRight - motors.currentRight;
    int step = (diff > 0) ? ACCEL_STEP : -DECEL_STEP;
    if (abs(diff) <= abs(step)) {
      motors.currentRight = motors.targetRight;
    } else {
      motors.currentRight += step;
    }
    changed = true;
  }
  if (changed) {
    setMotor(leftForward, leftBackward, motors.currentLeft);
    setMotor(rightForward, rightBackward, motors.currentRight);
    motors.isMoving = (motors.currentLeft != 0 || motors.currentRight != 0);
  }
}

void gradualStop() {
  motors.targetLeft = 0;
  motors.targetRight = 0;
}


// =================================================================
//      STM32 COMMUNICATION (Unchanged)
// =================================================================

bool enqueueSTM32Command(const String& cmd) {
  if (queueCount >= STM32_QUEUE_SIZE) {
    if (DEBUG) Serial.println("[STM32] Queue full, dropping command");
    return false;
  }
  int nextTail = (queueTail + 1) % STM32_QUEUE_SIZE;
  stm32Queue[queueTail].command = cmd;
  stm32Queue[queueTail].timestamp = millis();
  stm32Queue[queueTail].sent = false;
  queueTail = nextTail;
  queueCount++;
  return true;
}

void processSTM32Queue() {
  if (queueCount == 0) return;
  if (millis() - lastSTM32Send < STM32_SEND_INTERVAL) return;

  STM32Command& cmd = stm32Queue[queueHead];
  if (!cmd.sent) {
    stm32Serial.println(cmd.command);
    cmd.sent = true;
    lastSTM32Send = millis();
    if (DEBUG) Serial.printf("[STM32] Sent: %s\n", cmd.command.c_str());
  }
  queueHead = (queueHead + 1) % STM32_QUEUE_SIZE;
  queueCount--;
}

void flushSTM32Response() {
  while (stm32Serial.available()) {
    stm32Serial.read();
  }
}