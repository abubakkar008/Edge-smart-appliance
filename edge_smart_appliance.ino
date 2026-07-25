/*
  =====================================================================
   PROJECT 4 — Edge-Computing Smart Home Appliance (Interrupts & Safety)
   DecodeLabs IoT Industrial Training Kit — Batch 2026
  =====================================================================

   GOAL
   Build a deterministic, fail-safe embedded system that:
     1) Uses a hardware interrupt (PIR sensor) to control a smart light
        the instant motion is detected — zero polling delay.
     2) Uses a hardware interrupt on a gas/smoke sensor's digital
        threshold line to instantly PREEMPT everything else in the
        system the moment a hazard is detected.
     3) Cross-validates that hazard against the sensor's analog reading
        (a second, independent measurement) before locking the system
        into a FAULT state — this avoids false-positive shutdowns from
        electrical noise while still reacting in hardware time.
     4) Drives a physical buzzer + red LED alarm and cuts power to the
        appliance through a Normally-Open (NO) relay, so that ANY power
        loss or MCU crash also fails to the safe (off) state.

   HARDWARE / PIN MAP (ESP32 Dev Board)
   -------------------------------------------------------------------
   PIR_PIN          GPIO 27   PIR motion sensor SIGNAL (digital, RISING)
   GAS_DO_PIN       GPIO 25   Gas/smoke sensor DIGITAL threshold output
                              (this is what we hardware-interrupt on)
   GAS_AO_PIN       GPIO 34   Gas/smoke sensor ANALOG output (ADC1_CH6)
                              (used only for confirmation, never for
                              the interrupt itself — ADC pins on ESP32
                              cannot be interrupt sources)
   RELAY_PIN        GPIO 26   NO relay driving the smart light / load
   BUZZER_PIN       GPIO 32   Piezo buzzer (active alarm)
   RED_LED_PIN      GPIO 33   Red fault-indicator LED
   RESET_BTN_PIN    GPIO 13   Manual "acknowledge fault & reset" button
                              (active LOW, INPUT_PULLUP)

   WIRING NOTES
   - PIR sensor: VCC -> 5V/3V3 (per module), GND -> GND,
     SIGNAL -> GPIO27. Use INPUT_PULLUP so the line never floats.
   - Relay module: remove the JD-VCC jumper so the relay's coil/EMF
     domain is optically isolated from the ESP32's logic domain.
     Relay IN -> GPIO26. Wire the appliance through the NO (Normally
     Open) contacts ONLY — never NC — so a de-energized relay
     (power loss, brown-out, panic/reset) always leaves the load OFF.
   - Gas/smoke sensor (e.g. MQ-2 style module): has both a DO
     (digital, threshold-comparator) pin and an AO (analog) pin.
     DO -> GPIO25 (interrupt), AO -> GPIO34 (ADC confirmation read).

   CORE DESIGN PRINCIPLE — "Freeze. Execute. Return."
   ISRs here do the absolute minimum: set a volatile flag and record a
   timestamp. All real logic (debounce, validation, actuation) happens
   in loop(), which is non-blocking (no delay(), no Serial.print()
   inside ISRs, no dynamic memory in ISRs).
   =====================================================================
*/

// ---------------------- Pin Definitions ----------------------
const uint8_t PIR_PIN        = 27;
const uint8_t GAS_DO_PIN     = 25;
const uint8_t GAS_AO_PIN     = 34;
const uint8_t RELAY_PIN      = 26;
const uint8_t BUZZER_PIN     = 32;
const uint8_t RED_LED_PIN    = 33;
const uint8_t RESET_BTN_PIN  = 13;

// ---------------------- Tunable Thresholds ----------------------
const uint16_t GAS_ANALOG_THRESHOLD   = 1800;  // 0-4095 (12-bit ADC), calibrate to your sensor
const unsigned long PIR_DEBOUNCE_MS   = 50;    // ignore PIR retriggers faster than this
const unsigned long LIGHT_ON_TIME_MS  = 8000;  // how long the light stays on after motion
const unsigned long GAS_VALIDATION_MS = 200;   // window to cross-check DO trigger against AO reading
const unsigned long ALARM_BLINK_MS    = 250;   // red LED / buzzer blink period during fault

// ---------------------- Volatile ISR-shared state ----------------------
// Anything written inside an ISR and read in loop() MUST be volatile,
// otherwise the compiler may cache a stale value in a CPU register.
volatile bool          motionFlag        = false;
volatile unsigned long lastPirInterrupt  = 0;

volatile bool          gasTriggerFlag    = false;
volatile unsigned long gasTriggerTime    = 0;

// ---------------------- Non-ISR system state ----------------------
enum SystemState { NORMAL, VALIDATING_GAS, FAULT_LOCKED };
SystemState currentState = NORMAL;

bool lightOn               = false;
unsigned long lightOnAt    = 0;
unsigned long lastAlarmToggle = 0;
bool alarmBlinkState       = false;

// Non-blocking reset-button debounce state (no delay() in loop()).
const unsigned long RESET_DEBOUNCE_MS = 30;
bool resetBtnCandidate        = false;
unsigned long resetBtnEdgeAt  = 0;

// =====================================================================
//  INTERRUPT SERVICE ROUTINES
//  IRAM_ATTR forces these into fast internal RAM so a flash-cache miss
//  can never introduce non-deterministic latency into the response.
// =====================================================================

void IRAM_ATTR pirISR() {
  unsigned long now = millis();
  // Hardware debounce lockout window — filters mechanical/EM contact bounce.
  if (now - lastPirInterrupt > PIR_DEBOUNCE_MS) {
    motionFlag = true;
    lastPirInterrupt = now;
  }
}

void IRAM_ATTR gasEmergencyISR() {
  // This is the "Mission 2" absolute-preemption interrupt.
  // No blocking calls, no Serial, no delay, no malloc — just flag + timestamp.
  // Latch the timestamp only on the FIRST trigger of a validation cycle.
  // If every retrigger were allowed to overwrite gasTriggerTime, a noisy
  // or chattering DO line would keep pushing the validation deadline
  // forward and the system could get stuck in VALIDATING_GAS forever,
  // never confirming a real hazard and never dismissing a transient one.
  if (!gasTriggerFlag) {
    gasTriggerFlag = true;
    gasTriggerTime = millis();
  }
}

// =====================================================================
//  SETUP
// =====================================================================
void setup() {
  Serial.begin(115200);

  pinMode(PIR_PIN, INPUT_PULLUP);
  pinMode(GAS_DO_PIN, INPUT_PULLUP);
  pinMode(GAS_AO_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(RESET_BTN_PIN, INPUT_PULLUP);

  // Fail-safe default: relay de-energized (light OFF), alarm OFF.
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(RED_LED_PIN, LOW);

  // Mission 1: event-driven convenience lighting.
  attachInterrupt(digitalPinToInterrupt(PIR_PIN), pirISR, RISING);

  // Mission 2: zero-lag safety override. Most MQ-style DO comparator
  // outputs go LOW when the gas threshold is exceeded — adjust the
  // mode (RISING/FALLING) to match your specific module's polarity.
  attachInterrupt(digitalPinToInterrupt(GAS_DO_PIN), gasEmergencyISR, FALLING);

  Serial.println(F("[BOOT] Edge-Computing Smart Home Appliance online."));
  Serial.println(F("[BOOT] Interrupts armed: PIR (RISING), GAS_DO (FALLING)."));
}

// =====================================================================
//  LOOP — all real decision-making happens here, never in an ISR.
// =====================================================================
void loop() {

  // ---- 1) Handle manual fault-reset button (only works from FAULT state)
  //         Non-blocking debounce: require the LOW level to be stable for
  //         RESET_DEBOUNCE_MS before acting on it, without ever calling
  //         delay() — a blocking call here would be exactly the kind of
  //         "background loop hides the ISR" problem this design avoids.
  if (currentState == FAULT_LOCKED) {
    bool pressedNow = (digitalRead(RESET_BTN_PIN) == LOW);
    if (pressedNow && !resetBtnCandidate) {
      resetBtnCandidate = true;
      resetBtnEdgeAt = millis();
    } else if (!pressedNow) {
      resetBtnCandidate = false;
    } else if (resetBtnCandidate && (millis() - resetBtnEdgeAt >= RESET_DEBOUNCE_MS)) {
      clearFault();
      resetBtnCandidate = false;
    }
  }

  // ---- 2) Absolute preemption: a validated/raw gas trigger always
  //         takes priority over whatever normal-mode logic is doing.
  if (gasTriggerFlag && currentState == NORMAL) {
    currentState = VALIDATING_GAS;
    Serial.println(F("[GAS] Digital threshold interrupt fired. Opening validation window..."));
  }

  if (currentState == VALIDATING_GAS) {
    unsigned long elapsed = millis() - gasTriggerTime;
    int analogReading = analogRead(GAS_AO_PIN);

    if (analogReading >= GAS_ANALOG_THRESHOLD) {
      // Confirmed by the independent analog channel -> real hazard.
      enterFault(analogReading);
    } else if (elapsed >= GAS_VALIDATION_MS) {
      // Digital line tripped but analog never confirmed within the
      // window -> treat as a transient/noise event, log and recover.
      Serial.print(F("[GAS] Transient fault dismissed. AO="));
      Serial.println(analogReading);
      gasTriggerFlag = false;
      currentState = NORMAL;
    }
    // else: still inside the validation window, keep polling non-blockingly.
  }

  // ---- 3) FAULT_LOCKED: alarm output, relay stays open, ignore PIR.
  if (currentState == FAULT_LOCKED) {
    runAlarmBlink();
    motionFlag = false; // discard any motion events queued during fault
    return;              // nothing else in the loop runs while locked
  }

  // ---- 4) NORMAL mode: standard PIR-driven lighting logic.
  if (currentState == NORMAL) {
    if (motionFlag) {
      motionFlag = false;
      digitalWrite(RELAY_PIN, HIGH); // energize NO relay -> light ON
      lightOn = true;
      lightOnAt = millis();
      Serial.println(F("[PIR] Motion detected -> light ON"));
    }

    if (lightOn && (millis() - lightOnAt >= LIGHT_ON_TIME_MS)) {
      digitalWrite(RELAY_PIN, LOW); // de-energize -> light OFF (fail-safe default)
      lightOn = false;
      Serial.println(F("[PIR] Timeout elapsed -> light OFF"));
    }
  }
}

// =====================================================================
//  Helper functions (non-blocking)
// =====================================================================

void enterFault(int confirmedReading) {
  currentState = FAULT_LOCKED;
  digitalWrite(RELAY_PIN, LOW); // immediately de-energize relay -> appliance OFF
  lightOn = false;
  Serial.print(F("[FAULT] Gas/smoke hazard CONFIRMED. AO="));
  Serial.println(confirmedReading);
  Serial.println(F("[FAULT] System locked. Normal operation suspended."));
}

void clearFault() {
  currentState = NORMAL;
  gasTriggerFlag = false;
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(RED_LED_PIN, LOW);
  Serial.println(F("[RESET] Fault acknowledged. System restored to NORMAL."));
}

void runAlarmBlink() {
  if (millis() - lastAlarmToggle >= ALARM_BLINK_MS) {
    lastAlarmToggle = millis();
    alarmBlinkState = !alarmBlinkState;
    digitalWrite(RED_LED_PIN, alarmBlinkState);
    digitalWrite(BUZZER_PIN, alarmBlinkState);
  }
}
