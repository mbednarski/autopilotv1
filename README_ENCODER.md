# Rotary Encoder Hardware Configuration

## Overview

This document describes the hardware connections and configuration for rotary encoder support on the STM32F091RC Nucleo board. The encoder is used to send HDG:SET commands to the PC via UART.

---

## Hardware Requirements

### STM32 Board
- **Board:** NUCLEO-F091RC
- **MCU:** STM32F091RCTx (Cortex-M0, 48 MHz)
- **Voltage:** 3.3V logic levels

### Rotary Encoder Specifications

The firmware is designed for standard **incremental quadrature encoders** with the following characteristics:

#### Electrical
- **Type:** Incremental rotary encoder with quadrature output (A/B channels)
- **Output:** Open-drain or open-collector (requires pull-up resistors)
- **Voltage:** 3.3V or 5V tolerant
- **Current:** Typically <5mA per channel

#### Mechanical
- **Detents:** Optional (20 detents per revolution is common)
- **Push Button:** Optional, used for HDG:RESET function
- **Shaft:** Typically 6mm diameter, D-shaft or flatted

#### Example Compatible Encoders
| Part Number | Manufacturer | Type | Detents | Button | Price Range |
|-------------|-------------|------|---------|--------|-------------|
| EC11E183440C | Alps | 20mm | 20 | Yes | $1-2 |
| PEC11R-4215F-S0024 | Bourns | 20mm | 24 | Yes | $2-3 |
| EC12E2420801 | Alps | 12mm | 24 | No | $1-2 |
| RKJXT1F42001 | Alps | Joystick | 24 | Yes | $3-5 |
| KY-040 Module | Generic | PCB Module | 20 | Yes | $1 |

⚠️ **Note:** Most cheap rotary encoders (like KY-040) are 5V tolerant but work fine at 3.3V. Verify your encoder's voltage rating!

---

## Pin Connections

### STM32 Nucleo Pin Assignments

| Function | STM32 Pin | Arduino Header | Encoder Terminal | Notes |
|----------|-----------|----------------|------------------|-------|
| **Encoder A** | PA0 | A0 | CLK / A / Pin A | TIM2_CH1 |
| **Encoder B** | PA1 | A1 | DT / B / Pin B | TIM2_CH2 |
| **Encoder Button** | PA4 | A2 | SW / Push / Button | GPIO EXTI4 |
| **Ground** | GND | GND | GND / - | Common ground |
| **Power (3.3V)** | +3.3V | +3.3V | + / VCC | Only if encoder needs power |

### Wiring Diagram

```
STM32 Nucleo (CN8 Header)          Rotary Encoder
┌────────────────┐                 ┌──────────────┐
│                │                 │              │
│  PA0 (A0) ●────┼─────────────────┤ A (CLK)      │
│                │                 │              │
│  PA1 (A1) ●────┼─────────────────┤ B (DT)       │
│                │                 │              │
│  PA4 (A2) ●────┼─────────────────┤ SW (Button)  │
│                │                 │              │
│  GND      ●────┼─────────────────┤ GND (-)      │
│                │                 │              │
│  +3.3V    ●────┼─────────────────┤ VCC (+)      │
│                │             (Optional, some encoders
│                │              don't need power)
└────────────────┘                 └──────────────┘
```

### Physical Location on Nucleo Board

**CN8 Connector (Left side, top rows):**
```
Pin 1 (NC)       ●  ●  Pin 2 (IOREF)
Pin 3 (RESET)    ●  ●  Pin 4 (3.3V)  ← +3.3V (optional)
Pin 5 (5V)       ●  ●  Pin 6 (GND)   ← Ground
Pin 7 (GND)      ●  ●  Pin 8 (GND)
```

**CN8 Connector (Left side, analog pins):**
```
Pin 9  (A0/PA0)  ●  ●  Pin 10 (A1/PA1) ← Encoder A & B
Pin 11 (A2/PA4)  ●  ●  Pin 12 (A3)     ← Encoder Button
Pin 13 (A4)      ●  ●  Pin 14 (A5)
```

---

## Pull-up Resistor Configuration

### Internal Pull-ups (Used in This Design)

The STM32 **internal pull-up resistors** are enabled on all encoder pins:
- **PA0:** Pull-up enabled (~40kΩ typical)
- **PA1:** Pull-up enabled (~40kΩ typical)
- **PA4:** Pull-up enabled (~40kΩ typical)

This means:
- ✅ **No external resistors needed** for most encoders
- ✅ Simplified wiring
- ⚠️ **Encoder outputs must be open-drain/open-collector** (common for rotary encoders)

### When to Add External Pull-ups

Add external 4.7kΩ pull-ups if:
- Encoder cable is >15cm (long wires = more noise, need stronger pull-ups)
- You experience erratic counting or missed steps
- The encoder manufacturer recommends external pull-ups

**External Pull-up Wiring:**
```
         +3.3V
           │
          ┌┴┐
          │ │ 4.7kΩ resistor
          └┬┘
           ├──────── To Encoder A (PA0)
           │
          ┌┴┐
          │ │ 4.7kΩ resistor
          └┬┘
           ├──────── To Encoder B (PA1)
           │
          ┌┴┐
          │ │ 4.7kΩ resistor (10kΩ also OK for button)
          └┬┘
           └──────── To Encoder SW (PA4)
```

---

## STM32CubeMX Configuration

### Pins

**PA0 (Encoder A):**
- Mode: `TIM2_CH1` (Alternate Function)
- GPIO mode: Alternate Function Push-Pull
- Pull-up/Pull-down: **Pull-up**
- User Label: `KNOB1_A`

**PA1 (Encoder B):**
- Mode: `TIM2_CH2` (Alternate Function)
- GPIO mode: Alternate Function Push-Pull
- Pull-up/Pull-down: **Pull-up**
- User Label: `KNOB1_B`

**PA4 (Button):**
- Mode: `GPIO_EXTI4` (External Interrupt)
- GPIO mode: External Interrupt Mode with Falling edge trigger
- Pull-up/Pull-down: **Pull-up**
- User Label: `BTN_KNOB1`

### TIM2 Configuration

**Mode:**
- Combined Channels: `Encoder Mode`
- Encoder Mode: **`TI1 and TI2`** (4x resolution, counts on all edges)

**Configuration → Parameter Settings:**
- Counter Period (AutoReload): `4294967295` (max 32-bit value = 0xFFFFFFFF)
- Prescaler: `0` (no prescaling)
- Counter Mode: Up
- Encoder Mode: `TI1 and TI2`

**Why TI1 and TI2 (4x resolution)?**
- Counts on **all 4 edges** per encoder cycle (A rising, A falling, B rising, B falling)
- Maximum resolution and noise immunity
- Most reliable mode for rotary encoders

### NVIC Settings

**Interrupts to Enable:**
- ✅ `EXTI line 4 to 15 interrupts` (for button on PA4)
- Priority: 0 (default is fine)

### Clock Configuration

- System Clock: 48 MHz (HSI48)
- APB1 Timer Clock: 48 MHz (TIM2 runs at this frequency)

---

## Software Configuration

### Constants (main.c)

```c
// Protocol commands (already defined)
#define PROTO_START      0xAA
#define CMD_HDG_RESET    0x10  // Button press command
#define CMD_HDG_SET      0x11  // Encoder rotation command

// Timing constants (adjust if needed)
#define MIN_SEND_INTERVAL_MS  20   // Encoder rate limit (50 Hz)
#define DEBOUNCE_TIME_MS      50   // Button debounce time
```

### Adjustable Parameters

**Encoder Rate Limiting:**
```c
const uint32_t MIN_SEND_INTERVAL_MS = 20;  // In ProcessEncoder()
```
- **Lower (10ms):** More responsive, more UART traffic
- **Higher (50ms):** Less responsive, less UART traffic
- **Recommended:** 20ms (50 Hz) for smooth operation

**Button Debounce Time:**
```c
const uint32_t DEBOUNCE_TIME_MS = 50;  // In HAL_GPIO_EXTI_Callback()
```
- **Lower (10ms):** Faster double-click, may catch bounces
- **Higher (100ms):** Slower response, better bounce filtering
- **Recommended:** 50ms for most mechanical switches

**Main Loop Polling Rate:**
```c
HAL_Delay(10);  // In main loop
```
- **Lower (5ms):** Higher CPU usage, faster encoder read
- **Higher (20ms):** Lower CPU usage, still responsive
- **Recommended:** 10ms (100 Hz polling)

---

## How It Works

### Hardware Encoder Mode (TIM2)

The STM32 timer peripheral has a **dedicated hardware encoder mode** that automatically:

1. **Monitors both encoder channels** (A and B on PA0/PA1)
2. **Counts pulses in hardware** (zero CPU overhead!)
3. **Determines direction** based on phase relationship:
   - If A leads B: Count UP (clockwise)
   - If B leads A: Count DOWN (counter-clockwise)
4. **Never misses steps** (even if main loop is delayed)

**Quadrature Encoding:**
```
Clockwise Rotation:
A: __|‾‾|__|‾‾|__    (Channel A toggles)
B: ‾‾|__|‾‾|__|‾‾    (Channel B lags 90°)
      ↑  ↑  ↑  ↑
   Count increments on each edge

Counter-Clockwise Rotation:
A: __|‾‾|__|‾‾|__    (Channel A toggles)
B: __|‾‾|__|‾‾|__    (Channel B leads 90°)
      ↑  ↑  ↑  ↑
   Count decrements on each edge
```

### Encoder Processing Flow

1. **Hardware counting** (TIM2 in background):
   - User rotates encoder → GPIO pins toggle
   - TIM2 hardware detects edges and updates counter
   - No CPU intervention needed!

2. **Software polling** (main loop every 10ms):
   ```c
   ProcessEncoder() {
       current_count = read TIM2 counter
       delta = current_count - last_count

       if (delta != 0 && rate_limit_OK) {
           SendHdgSet(delta)
           last_count = current_count
       }
   }
   ```

3. **UART transmission** (DMA in background):
   - `SendHdgSet()` triggers DMA transfer
   - 4-byte packet sent without CPU involvement
   - PC receives and decodes command

### Button Interrupt Flow

1. **Button press** → PA4 goes LOW (pulled HIGH normally, grounded when pressed)
2. **Falling edge trigger** → EXTI4 interrupt fires
3. **Interrupt handler** → `HAL_GPIO_EXTI_Callback()` called
4. **Debounce check** → Ignore if within 50ms of last press
5. **Send command** → `SendHdgReset()` via UART DMA

---

## Troubleshooting

### Encoder Not Counting

**Symptom:** Rotating encoder does nothing, no UART packets sent.

**Possible Causes:**
1. **Encoder not powered:** Some encoders need +3.3V/+5V on VCC pin
   - Check encoder datasheet
   - Measure voltage on VCC pin (should be 3.3V)

2. **Wrong wiring:** A/B channels swapped or not connected
   - Verify PA0 → Encoder A, PA1 → Encoder B
   - Check continuity with multimeter

3. **No pull-ups:** Encoder outputs floating
   - Verify pull-ups enabled in CubeMX (check main.c GPIO init)
   - Try external 4.7kΩ pull-ups

4. **Timer not started:** Missing `HAL_TIM_Encoder_Start()`
   - Check main.c line ~175 for `HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);`

5. **Debug:** Add LED toggle in `ProcessEncoder()` to verify it's being called

### Encoder Counts Wrong Direction

**Symptom:** CW rotation decrements, CCW increments (backwards).

**Fix:** Swap A and B connections:
- PA0 → Encoder B
- PA1 → Encoder A

Alternatively, negate delta in software:
```c
delta = -(current_count - encoder_last_count);  // Reverse direction
```

### Erratic Counting / Random Jumps

**Symptom:** Encoder counts erratically, jumps multiple values, or counts when not touched.

**Possible Causes:**
1. **Noise on long wires:** Add external pull-ups (4.7kΩ) or shielded cable
2. **Bad encoder:** Mechanical wear, dirty contacts
   - Clean with contact cleaner
   - Try a different encoder
3. **EMI interference:** Keep encoder wires away from power lines, motors
4. **Weak pull-ups:** Add external 4.7kΩ resistors

**Hardware filtering:** Add 100nF capacitors between A/B pins and GND (close to encoder)

### Button Does Nothing

**Symptom:** Pressing button has no effect.

**Possible Causes:**
1. **Not connected:** Check PA4 wiring to button
2. **No pull-up:** Enable pull-up in CubeMX on PA4
3. **EXTI not enabled:** Check NVIC configuration for EXTI4_15
4. **Wrong trigger:** Should be "Falling edge" (LOW when pressed)

**Debug:** Add LED toggle in `HAL_GPIO_EXTI_Callback()` to verify interrupt fires

### Button Sends Multiple Commands (Bouncing)

**Symptom:** Single button press sends 5-10 HDG:RESET commands.

**Fix:** Increase debounce time:
```c
const uint32_t DEBOUNCE_TIME_MS = 100;  // Increase from 50ms
```

Or add hardware filtering: 100nF capacitor between PA4 and GND

### UART Flooded with Packets

**Symptom:** Spinning encoder floods UART, PC receiver lags.

**Fix:** Increase rate limit interval:
```c
const uint32_t MIN_SEND_INTERVAL_MS = 50;  // Slower update rate
```

---

## Testing Procedure

### 1. Visual Inspection
- [ ] Verify all 4 wires connected: A, B, Button, GND
- [ ] Check for short circuits (especially VCC to GND)
- [ ] Ensure encoder is powered if needed

### 2. Multimeter Testing (Power Off)
- [ ] Continuity: PA0 ↔ Encoder A
- [ ] Continuity: PA1 ↔ Encoder B
- [ ] Continuity: PA4 ↔ Encoder Button
- [ ] No short: PA0/PA1/PA4 to GND (should be open when encoder not pressed)

### 3. Logic Analyzer / Oscilloscope (Power On)
- [ ] PA0: Should see pulses when rotating encoder
- [ ] PA1: Should see pulses 90° phase-shifted from PA0
- [ ] PA4: Should go LOW when button pressed, HIGH when released
- [ ] Pull-up check: Idle voltage should be ~3.3V on all pins

### 4. Software Testing
1. **Build and flash** firmware
2. **Open PC serial monitor** (115200 baud)
3. **Rotate encoder CW** → Should see HDG:SET commands with positive delta
4. **Rotate encoder CCW** → Should see HDG:SET commands with negative delta
5. **Press button** → Should see HDG:RESET command

### 5. Debug Output (Optional)
Add printf debugging via SWO or UART:
```c
void ProcessEncoder(void) {
    int32_t current_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    printf("Counter: %ld\r\n", current_count);  // Debug output
    // ... rest of function
}
```

---

## Future Expansion: Adding More Encoders

This design supports up to **4 encoders** on the STM32F091:

| Encoder | Timer | Pins A/B | Button Pin | Command |
|---------|-------|----------|------------|---------|
| **1 (HDG)** | TIM2 | PA0, PA1 | PA4 | HDG:SET / HDG:RESET |
| **2 (ALT)** | TIM3 | PA6, PA7 | PA5 | ALT:SET / ALT:RESET |
| **3 (SPD)** | TIM1 | PA8, PA9 | PA10 | SPD:SET / SPD:RESET |
| **4 (Custom)** | GPIO EXTI | PB0, PB1 | PB2 | Custom command |

**Timers with encoder mode:** TIM1, TIM2, TIM3 (only 3!)
**4th encoder:** Must use GPIO interrupts (EXTI) instead of hardware timer

---

## Pin Usage Summary

### Currently Used Pins
| Pin | Function | Direction | Notes |
|-----|----------|-----------|-------|
| PA0 | TIM2_CH1 (Encoder A) | Input | Pull-up enabled |
| PA1 | TIM2_CH2 (Encoder B) | Input | Pull-up enabled |
| PA2 | USART2_TX | Output | Serial TX to PC |
| PA3 | USART2_RX | Input | Serial RX from PC |
| PA4 | GPIO_EXTI4 (Button) | Input | Pull-up enabled, falling edge |
| PA5 | LD2 (Green LED) | Output | Nucleo onboard LED (available) |
| PC13 | B1 (Blue Button) | Input | Nucleo onboard button (available) |

### Available for Future Encoders
- **PA5-PA15:** Available
- **PB0-PB15:** Available
- **PC0-PC15:** Available (except PC13 = blue button)

---

## References

### Datasheets
- [STM32F091RC Reference Manual](https://www.st.com/resource/en/reference_manual/rm0091-stm32f0x1stm32f0x2stm32f0x8-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [STM32F0 HAL User Manual](https://www.st.com/resource/en/user_manual/um1785-description-of-stm32f0-hal-and-lowlayer-drivers-stmicroelectronics.pdf)
- [NUCLEO-F091RC User Manual](https://www.st.com/resource/en/user_manual/um1724-stm32-nucleo64-boards-mb1136-stmicroelectronics.pdf)

### Application Notes
- AN4013: "Encoder interface on STM32 MCUs"
- AN4221: "Using Timer in Encoder Mode"

### Example Encoder Distributors
- Digi-Key: Search "rotary encoder incremental"
- Mouser: Search "mechanical encoder"
- AliExpress/Amazon: "KY-040 rotary encoder module"

---

## Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-01-XX | Claude Code | Initial version with TIM2 encoder support |

---

**Questions or issues?** Check PROTOCOL.md for communication protocol details or review the extensively commented code in `main.c`.
