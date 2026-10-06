// ============================================================================
// PROJECT: LD2410 Radar + Blynk IoT - Optimized & Stable Version
// PURPOSE: ตรวจจับการเคลื่อนไหว ควบคุมไฟ และส่งข้อมูลไปยัง Blynk
// AUTHOR: Optimized for Production Stability
// ============================================================================

// --- Blynk IoT Configuration ---
#define BLYNK_TEMPLATE_ID "TMPL6Q66ynPXl"
#define BLYNK_TEMPLATE_NAME "RadarT1"
#define BLYNK_AUTH_TOKEN "7vzWrpbYTM58aCL88qfS3jvjDTSsAUSW"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <ld2410.h>

// --- WiFi Configuration ---
char ssid[] = "AIZAWA";
char pass[] = "11223344";

// --- Hardware Configuration ---
ld2410 radar;
HardwareSerial radarSerial(1);

#define RX_PIN 5
#define TX_PIN 4
#define LED_PIN 6

// ============================================================================
// STABILITY & TIMING CONFIGURATIONS
// ============================================================================

// Debounce & Hysteresis (ป้องกัน false trigger)
#define DETECTION_DEBOUNCE_TIME 300    // 300ms - รอให้สัญญาณ stable ก่อนตัดสินใจ
#define LED_HOLD_TIME 2000             // 2000ms - ไฟติดต่อไปตามหลังจนแน่ใจว่าจริง ๆ ไม่มีคน
#define PRESENCE_TIMEOUT 5000          // 5000ms - ถ้าไม่เจอ sensor 5 วิให้ถือว่าไม่มีคน

// Distance Threshold (ระยะการตรวจจับ)
#define MOVING_DISTANCE_MAX 500        // Maximum 500cm (5 meters) for moving target
#define STATIC_DISTANCE_MAX 200        // Maximum 200cm (2 meters) for stationary target
#define MIN_DETECTION_DISTANCE 50      // Minimum 50cm to avoid false triggers from near objects

// Blynk Sync Interval
#define BLYNK_SYNC_INTERVAL 1000       // ส่งข้อมูลระยะทางทุก 1 วินาที

// Serial Debug Output
#define SERIAL_PRINT_INTERVAL 500      // แสดง Serial monitor ทุก 500ms

// ============================================================================
// GLOBAL STATE VARIABLES
// ============================================================================

// Timing Variables
unsigned long lastPrintTime = 0;
unsigned long lastDetectionTime = 0;
unsigned long lastBlynkSyncTime = 0;
unsigned long lastStateChangeTime = 0;

// Detection State Machine
enum DetectionState {
  STATE_EMPTY,           // ห้องว่าง
  STATE_DETECTING,       // กำลังตรวจจับ (debounce)
  STATE_OCCUPIED,        // มีคนอยู่ยืนยันแล้ว
  STATE_LEAVING          // คนเรียบร้อยที่ดับไฟ
};

DetectionState currentState = STATE_EMPTY;
DetectionState previousState = STATE_EMPTY;

// Sensor Data
int movingDistance = 0;
int staticDistance = 0;
bool sensorHasTarget = false;

// LED Status
bool ledIsOn = false;

// ============================================================================
// SETUP FUNCTION
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(500);
  
  Serial.println("\n\n=== LD2410 Radar + Blynk IoT (Stable Version) ===\n");
  
  // Configure Radar Serial Communication
  radarSerial.setRxBufferSize(256);
  radarSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  
  // Setup LED Pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  ledIsOn = false;
  
  Serial.println("📡 Initializing LD2410 Radar Sensor...");
  delay(100);
  
  // Initialize Radar
  if (radar.begin(radarSerial)) {
    Serial.println("✅ LD2410 Sensor Connected Successfully!");
    delay(500);
    
    // Start Engineering Mode for stable real-time readings
    radar.requestStartEngineeringMode();
    Serial.println("⚙️  Engineering Mode Activated");
  } else {
    Serial.println("❌ Failed to connect LD2410! Check TX/RX pins and power supply.");
    while (1) {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }
  
  delay(500);
  Serial.println("\n🌐 Connecting to WiFi and Blynk...");
  
  // Connect to Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  Serial.println("✅ Blynk Connected Successfully!\n");
  
  // Initialize Timing
  lastPrintTime = millis();
  lastDetectionTime = millis();
  lastBlynkSyncTime = millis();
  lastStateChangeTime = millis();
  
  Serial.println("🚀 System Ready! Waiting for movement detection...\n");
}

// ============================================================================
// MAIN LOOP FUNCTION
// ============================================================================

void loop() {
  Blynk.run();           // Process Blynk events
  radar.read();          // Read sensor data continuously

  if (radar.isConnected()) {
    updateSensorData();  // Read and filter sensor data
    updateDetectionState(); // Update state machine
    updateLEDOutput();    // Control LED based on state
    syncToBlynk();        // Send data to Blynk
    printDebugInfo();     // Print to Serial monitor
  } else {
    handleSensorDisconnect();
  }
}

// ============================================================================
// SENSOR DATA READING (with noise filtering)
// ============================================================================

void updateSensorData() {
  if (radar.presenceDetected()) {
    movingDistance = radar.movingTargetDistance();
    staticDistance = radar.stationaryTargetDistance();
    
    // Filter: Only consider valid detections within range
    sensorHasTarget = false;
    
    if ((movingDistance >= MIN_DETECTION_DISTANCE && movingDistance <= MOVING_DISTANCE_MAX) ||
        (staticDistance >= MIN_DETECTION_DISTANCE && staticDistance <= STATIC_DISTANCE_MAX)) {
      sensorHasTarget = true;
      lastDetectionTime = millis();  // Update last detection timestamp
    }
  } else {
    movingDistance = 0;
    staticDistance = 0;
    sensorHasTarget = false;
  }
}

// ============================================================================
// STATE MACHINE: Debounce & Hysteresis Logic
// ============================================================================

void updateDetectionState() {
  unsigned long now = millis();
  unsigned long timeSinceLastDetection = now - lastDetectionTime;
  unsigned long timeSinceLastStateChange = now - lastStateChangeTime;
  
  switch (currentState) {
    
    case STATE_EMPTY:
      // ถ้าเซ็นเซอร์เห็นการเคลื่อนไหว ให้เข้าสู่ debounce state
      if (sensorHasTarget) {
        currentState = STATE_DETECTING;
        lastStateChangeTime = now;
      }
      break;
    
    case STATE_DETECTING:
      // รอให้สัญญาณ stable สักพัก (debounce period)
      if (sensorHasTarget) {
        if (timeSinceLastStateChange >= DETECTION_DEBOUNCE_TIME) {
          currentState = STATE_OCCUPIED;
          lastStateChangeTime = now;
        }
      } else {
        // False alarm - ย้อนกลับไปว่าง
        currentState = STATE_EMPTY;
      }
      break;
    
    case STATE_OCCUPIED:
      // ยังรู้สึกถึงการเคลื่อนไหวหรือเพิ่งหายไป?
      if (sensorHasTarget) {
        // ยังมีคนอยู่
        lastDetectionTime = now;
      } else {
        // ไม่รู้สึกถึงการเคลื่อนไหว ให้รอสักพัก
        if (timeSinceLastDetection >= PRESENCE_TIMEOUT) {
          currentState = STATE_LEAVING;
          lastStateChangeTime = now;
        }
      }
      break;
    
    case STATE_LEAVING:
      // ถ้ามีการเคลื่อนไหวอีก ให้กลับไปว่างโดยปิดไฟ
      if (sensorHasTarget) {
        currentState = STATE_OCCUPIED;
        lastDetectionTime = now;
      } else {
        // ยังไม่มี signal หลังจาก LED hold time ให้ดับไฟ
        if (timeSinceLastStateChange >= LED_HOLD_TIME) {
          currentState = STATE_EMPTY;
        }
      }
      break;
  }
  
  // ตรวจจับการเปลี่ยนสถานะ
  if (currentState != previousState) {
    previousState = currentState;
    Serial.print("🔄 STATE CHANGE: ");
    printState(previousState);
    Serial.print(" → ");
    printState(currentState);
    Serial.println();
  }
}

// ============================================================================
// LED CONTROL
// ============================================================================

void updateLEDOutput() {
  bool shouldLedBeOn = (currentState == STATE_OCCUPIED || currentState == STATE_LEAVING);
  
  if (shouldLedBeOn && !ledIsOn) {
    digitalWrite(LED_PIN, HIGH);
    ledIsOn = true;
    Serial.println("💡 LED ON");
  } else if (!shouldLedBeOn && ledIsOn) {
    digitalWrite(LED_PIN, LOW);
    ledIsOn = false;
    Serial.println("⚫ LED OFF");
  }
}

// ============================================================================
// BLYNK SYNCHRONIZATION
// ============================================================================

void syncToBlynk() {
  unsigned long now = millis();
  
  // Send presence status only when state changes
  if (currentState != previousState) {
    bool presenceValue = (currentState == STATE_OCCUPIED || currentState == STATE_LEAVING);
    Blynk.virtualWrite(V3, presenceValue ? 1 : 0);
    Serial.printf("📤 Blynk V3 (Presence): %d\n", presenceValue ? 1 : 0);
  }
  
  // Send distance values every sync interval
  if (now - lastBlynkSyncTime >= BLYNK_SYNC_INTERVAL) {
    lastBlynkSyncTime = now;
    
    if (currentState == STATE_OCCUPIED || currentState == STATE_DETECTING) {
      Blynk.virtualWrite(V4, movingDistance);
      Blynk.virtualWrite(V5, staticDistance);
    } else {
      Blynk.virtualWrite(V4, 0);
      Blynk.virtualWrite(V5, 0);
    }
  }
}

// ============================================================================
// DEBUG OUTPUT TO SERIAL MONITOR
// ============================================================================

void printDebugInfo() {
  unsigned long now = millis();
  
  if (now - lastPrintTime >= SERIAL_PRINT_INTERVAL) {
    lastPrintTime = now;
    
    Serial.print("[");
    printState(currentState);
    Serial.print("] ");
    
    if (currentState == STATE_OCCUPIED || currentState == STATE_DETECTING) {
      Serial.printf("🎯 Detection - Moving: %d cm | Static: %d cm | LED: %s\n",
                    movingDistance, staticDistance, ledIsOn ? "ON" : "OFF");
    } else {
      Serial.printf("✅ Empty Room | LED: %s\n", ledIsOn ? "ON" : "OFF");
    }
  }
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

void printState(DetectionState state) {
  switch (state) {
    case STATE_EMPTY:
      Serial.print("EMPTY");
      break;
    case STATE_DETECTING:
      Serial.print("DETECTING");
      break;
    case STATE_OCCUPIED:
      Serial.print("OCCUPIED");
      break;
    case STATE_LEAVING:
      Serial.print("LEAVING");
      break;
  }
}

void handleSensorDisconnect() {
  Serial.println("⚠️  Sensor Disconnected!");
  digitalWrite(LED_PIN, LOW);
  ledIsOn = false;
  currentState = STATE_EMPTY;
  
  delay(100);
  if (millis() % 1000 < 500) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
}

// ============================================================================
// BLYNK VIRTUAL PIN HANDLERS (Optional - for manual control)
// ============================================================================

BLYNK_WRITE(V0) {
  // Manual LED control (optional feature)
  int value = param.asInt();
  if (value) {
    digitalWrite(LED_PIN, HIGH);
    Serial.println("📱 Blynk: Manual LED ON");
  } else {
    digitalWrite(LED_PIN, LOW);
    Serial.println("📱 Blynk: Manual LED OFF");
  }
}

BLYNK_CONNECTED() {
  Serial.println("✅ Blynk App Connected!");
}

// ============================================================================
// END OF CODE
// ============================================================================
