# Project 4 — Edge-Computing Smart Home Appliance (Interrupts & Safety)

**DecodeLabs IoT Industrial Training Kit — Batch 2026**
**Track:** Optional Mastery Phase — Edge Computing & Fail-Safe Architecture

## 1. Overview

This project implements an ESP32-based smart appliance controller that
proves two core embedded-systems skills:

1. **Real-time responsiveness** — a PIR motion sensor drives a smart
   light using a hardware interrupt (ISR), instead of a slow polling loop.
2. **Industrial safety fail-safes** — an analog/digital gas-smoke sensor
   can instantly preempt the entire system, lock it into a fault state,
   sound a buzzer, flash a red LED, and cut power to the appliance
   through a fail-safe relay — regardless of what the CPU was doing at
   that moment.

The design deliberately avoids polling for the safety path: a blocking
`delay()` or a busy background loop must never be able to hide a gas
leak from the system, so detection is handled entirely in hardware
(interrupt-driven), and only the *confirmation* logic runs in the main
loop.

## 2. Hardware / Wiring

| Component              | ESP32 Pin | Notes |
|-------------------------|-----------|-------|
| PIR sensor (SIGNAL)     | GPIO 27   | `INPUT_PULLUP`, triggers on `RISING` edge |
| Gas/smoke sensor DO     | GPIO 25   | Digital threshold comparator output — hardware interrupt source |
| Gas/smoke sensor AO     | GPIO 34   | Analog (ADC1_CH6) — used only to *confirm* a DO trigger |
| Relay module IN         | GPIO 26   | Drives the light/appliance through the relay's **NO** contacts |
| Buzzer                  | GPIO 32   | Active alarm during fault |
| Red LED                 | GPIO 33   | Blinks during fault |
| Reset button             | GPIO 13   | `INPUT_PULLUP`, active LOW — acknowledges & clears a fault |

**Safety wiring notes**

- The appliance/load is wired through the relay's **Normally Open (NO)**
  contacts only. If the ESP32 loses power, panics, or resets, the relay
  coil de-energizes and the mechanical spring breaks the circuit —
  the system fails to **OFF**, never to "stuck on."
- Remove the relay module's **JD-VCC jumper** so the high-voltage relay
  coil / back-EMF domain is optically isolated from the ESP32's 3.3V
  logic domain. This keeps inductive spikes from the coil away from the
  MCU.
- The gas sensor's analog pin is used purely for **confirmation**, never
  as the interrupt trigger — ESP32 ADC pins cannot be interrupt sources.

## 3. How the Logic Works

### Mission 1 — Event-driven lighting (convenience)
`attachInterrupt(PIR_PIN, pirISR, RISING)` binds motion detection
directly to hardware. The ISR does the absolute minimum: it applies a
50 ms debounce lockout (to filter EM noise / contact bounce) and sets a
`volatile` flag. All actual work — turning the relay on, starting the
auto-off timer — happens in `loop()`.

### Mission 2 — Absolute preemption for hazards (criticality)
The gas sensor's digital threshold pin is also hardware-interrupted.
The moment it fires, the system state jumps to `VALIDATING_GAS` — this
immediately halts the standard lighting cycle. Rather than trusting a
single analog spike (which risks false-positive shutdowns from noise),
the system opens a short, non-blocking **200 ms validation window** and
cross-checks the event against the sensor's independent analog reading.
Only if both agree does the system escalate to a locked fault.

### Fault state
Once confirmed, the relay is de-energized immediately, the buzzer and
red LED blink continuously, and the system **refuses to resume normal
operation** until a human presses the physical reset button — this is
the "unignorable state indication" requirement from the spec.

### Robustness details worth knowing
- **The gas ISR latches only the first trigger of a validation cycle.**
  `gasEmergencyISR()` ignores retriggers while `gasTriggerFlag` is
  already set, so a noisy/chattering DO line can't keep pushing the
  200 ms validation deadline forward and stalling the system in
  `VALIDATING_GAS` indefinitely.
- **The reset button is debounced without `delay()`.** `loop()` uses a
  small millis()-based state machine (30 ms stability window) instead
  of a blocking call, so the main loop genuinely never blocks —
  consistent with the "no delay() in the hot path" design goal.

### Why ISRs stay this minimal
- **No `delay()`** — it depends on the system tick timer and would
  freeze interrupt handling.
- **No `Serial.print()`** in an ISR — UART buffering can deadlock the
  core.
- **No dynamic memory (`malloc`)** — heap locks aren't interrupt-safe
  and can trigger a kernel panic.
- **`IRAM_ATTR`** on both ISRs forces them into fast internal RAM so a
  flash-cache miss can never introduce latency into the response path.
- **`volatile`** on every ISR-shared variable stops the compiler from
  caching a stale value in a CPU register.

## 4. Validation / Demo Protocol

To prove the system is genuinely interrupt-driven and not just a fast
polling loop, run this stress test (as specified in the training
material):

1. Add a computationally heavy dummy task to `loop()` (e.g. a tight
   floating-point loop) to saturate the CPU.
2. While that's running, trigger the gas sensor (or manually pull the
   `GAS_DO_PIN` line low with a jumper wire to simulate it).
3. Expected result: the hardware interrupt fires and the system enters
   `VALIDATING_GAS` **immediately**, regardless of what the dummy task
   was doing — because the interrupt controller preempts the CPU at the
   silicon level, not the software level.

Basic functional tests:

- Wave a hand in front of the PIR sensor → light turns on, turns off
  automatically after 8 seconds of no further motion.
- Expose the gas sensor to smoke/gas (or simulate via the AO reading
  crossing `GAS_ANALOG_THRESHOLD`) → light cuts immediately, buzzer and
  LED alarm start, and stay locked until the reset button is pressed.
- Trigger the gas DO pin briefly without a real hazard (transient
  noise) → system should log a "transient fault dismissed" message and
  return to normal without locking, since the analog channel never
  confirmed it.

## 5. Files

- `edge_smart_appliance.ino` — full Arduino/ESP32 sketch implementing
  the system described above. Flash with the Arduino IDE or
  `arduino-cli` targeting an ESP32 Dev Module board.

## 6. Calibration Notes

- `GAS_ANALOG_THRESHOLD` (default `1800` on a 12-bit 0–4095 ADC scale)
  should be calibrated against your specific gas/smoke sensor module
  and ambient baseline before relying on it for a real demo.
- `GAS_DO_PIN`'s interrupt mode (`FALLING` in this sketch) depends on
  your module's comparator polarity — some boards pull DO high on
  alarm instead of low. Check your sensor's datasheet and swap to
  `RISING` if needed.
