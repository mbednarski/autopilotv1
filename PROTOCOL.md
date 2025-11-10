# UART Communication Protocol

## Overview
This document describes the bidirectional binary protocol used for STM32 ↔ PC communication over UART. The system uses two separate frame formats depending on direction:
- **STM32 → PC**: 4-byte frames (START = `0xAA`)
- **PC → STM32**: 3-byte frames (START = `0x88`)

## Protocol Specification

### Frame Structure

#### STM32 → PC (4-byte frames)
Each message from STM32 to PC consists of exactly 4 bytes:

```
Byte 0: START    = 0xAA (frame synchronization marker)
Byte 1: COMMAND  = 0x00-0xFF (command identifier)
Byte 2: OPERAND  = 0x00-0xFF (command parameter)
Byte 3: CHECKSUM = START ^ COMMAND ^ OPERAND (XOR checksum)
```

#### PC → STM32 (3-byte frames)
Each message from PC to STM32 consists of exactly 3 bytes:

```
Byte 0: START    = 0x88 (frame synchronization marker)
Byte 1: COMMAND  = 0x00-0xFF (command identifier)
Byte 2: CHECKSUM = START ^ COMMAND (XOR checksum)
```

### Checksum Calculation

**For STM32 → PC (4-byte frames):**
```c
checksum = 0xAA ^ command ^ operand
```

**For PC → STM32 (3-byte frames):**
```c
checksum = 0x88 ^ command
```

This provides fast error detection with minimal computational overhead.

### Example Frames

**STM32 → PC (4-byte):**
```
Command: HDG:DELTA with delta = +5
Bytes: [0xAA, 0x11, 0x05, 0xAE]
       [START][CMD ][OPR ][CHK ]

Checksum: 0xAA ^ 0x11 ^ 0x05 = 0xAE
```

**PC → STM32 (3-byte):**
```
Command: LED_ON
Bytes: [0x88, 0x10, 0x98]
       [START][CMD ][CHK ]

Checksum: 0x88 ^ 0x10 = 0x98
```

## Command Set

### STM32 → PC Commands (4-byte frames, START = 0xAA)

#### HDG:DELTA (0x11)
Set heading delta (increment/decrement) via rotary encoder.

- **Command byte**: `0x11`
- **Operand**: Signed 8-bit integer (-128 to +127)
  - Positive values: increment heading
  - Negative values: decrement heading
  - Number of mechanical clicks (not raw counts)
- **Status**: ✅ Implemented (rotary encoder, dma2/Core/Src/main.c:348)
- **Examples**:
  - Delta +10: `[0xAA, 0x11, 0x0A, 0xA1]`
  - Delta -5: `[0xAA, 0x11, 0xFB, 0x40]` (0xFB = -5 in two's complement)
  - Delta +127: `[0xAA, 0x11, 0x7F, 0x24]`

#### ALT:DELTA (0x21)
Set altitude delta (increment/decrement) via rotary encoder.

- **Command byte**: `0x21`
- **Operand**: Signed 8-bit integer (-128 to +127)
  - Positive values: increment altitude
  - Negative values: decrement altitude
  - Number of mechanical clicks (not raw counts)
- **Status**: ✅ Implemented (rotary encoder, dma2/Core/Src/main.c:348)
- **Examples**:
  - Delta +10: `[0xAA, 0x21, 0x0A, 0x81]`
  - Delta -5: `[0xAA, 0x21, 0xFB, 0x70]` (0xFB = -5 in two's complement)
  - Delta +50: `[0xAA, 0x21, 0x32, 0x99]`

#### VS:DELTA (0x31)
Set vertical speed delta (increment/decrement) via rotary encoder.

- **Command byte**: `0x31`
- **Operand**: Signed 8-bit integer (-128 to +127)
  - Positive values: increment vertical speed
  - Negative values: decrement vertical speed
  - Number of mechanical clicks (not raw counts)
- **Status**: ✅ Implemented (rotary encoder, dma2/Core/Src/main.c:348)
- **Examples**:
  - Delta +10: `[0xAA, 0x31, 0x0A, 0x91]`
  - Delta -5: `[0xAA, 0x31, 0xFB, 0x60]` (0xFB = -5 in two's complement)
  - Delta +100: `[0xAA, 0x31, 0x64, 0xFF]`

#### BARO:DELTA (0x41)
Set barometric pressure delta (increment/decrement) via rotary encoder.

- **Command byte**: `0x41`
- **Operand**: Signed 8-bit integer (-128 to +127)
  - Positive values: increment barometric pressure
  - Negative values: decrement barometric pressure
  - Number of mechanical clicks (not raw counts)
- **Status**: ⚠️ Planned (not yet implemented in STM32)
- **Examples**:
  - Delta +10: `[0xAA, 0x41, 0x0A, 0xE1]`
  - Delta -5: `[0xAA, 0x41, 0xFB, 0x10]` (0xFB = -5 in two's complement)

#### BTN:AP_TOGGLE (0x50)
Autopilot mode toggle button pressed.

- **Command byte**: `0x50`
- **Operand**: Always `0x00` (unused)
- **Status**: ⚠️ Planned (not yet implemented in STM32)
- **Example**: `[0xAA, 0x50, 0x00, 0xFA]`

#### BTN:HDG_TOGGLE (0x51)
Heading mode toggle button pressed.

- **Command byte**: `0x51`
- **Operand**: Always `0x00` (unused)
- **Status**: ⚠️ Planned (not yet implemented in STM32)
- **Example**: `[0xAA, 0x51, 0x00, 0xFB]`

#### BTN:VS_TOGGLE (0x52)
Vertical speed mode toggle button pressed.

- **Command byte**: `0x52`
- **Operand**: Always `0x00` (unused)
- **Status**: ⚠️ Planned (not yet implemented in STM32)
- **Example**: `[0xAA, 0x52, 0x00, 0xF8]`

#### BTN:ALT_TOGGLE (0x53)
Altitude mode toggle button pressed.

- **Command byte**: `0x53`
- **Operand**: Always `0x00` (unused)
- **Status**: ⚠️ Planned (not yet implemented in STM32)
- **Example**: `[0xAA, 0x53, 0x00, 0xF9]`

### PC → STM32 Commands (3-byte frames, START = 0x88)

#### LED:ON (0x10)
Turn on the onboard LED (LD2).

- **Command byte**: `0x10`
- **Status**: ✅ Implemented (dma2/Core/Src/main.c)
- **Example**: `[0x88, 0x10, 0x98]`
- **Checksum**: `0x88 ^ 0x10 = 0x98`

#### LED:OFF (0x11)
Turn off the onboard LED (LD2).

- **Command byte**: `0x11`
- **Status**: ✅ Implemented (dma2/Core/Src/main.c)
- **Example**: `[0x88, 0x11, 0x99]`
- **Checksum**: `0x88 ^ 0x11 = 0x99`

#### AP:ENGAGE (0x60)
Engage the autopilot system.

- **Command byte**: `0x60`
- **Status**: ✅ Implemented (dma2/Core/Src/main.c)
- **Example**: `[0x88, 0x60, 0xE8]`
- **Checksum**: `0x88 ^ 0x60 = 0xE8`
- **Effect**: Sets AP_STATUS_ENGAGED bit, updates OLED display

#### AP:DISENGAGE (0x61)
Disengage the autopilot system (clears all active modes).

- **Command byte**: `0x61`
- **Status**: ✅ Implemented (dma2/Core/Src/main.c)
- **Example**: `[0x88, 0x61, 0xE9]`
- **Checksum**: `0x88 ^ 0x61 = 0xE9`
- **Effect**: Clears all AP status bits, blanks OLED display

#### HDG:MODE_ON (0x62)
Activate heading mode.

- **Command byte**: `0x62`
- **Status**: ✅ Implemented (dma2/Core/Src/main.c)
- **Example**: `[0x88, 0x62, 0xEA]`
- **Checksum**: `0x88 ^ 0x62 = 0xEA`
- **Effect**: Sets AP_STATUS_HDG_ACTIVE bit, updates display

#### HDG:MODE_OFF (0x63)
Deactivate heading mode.

- **Command byte**: `0x63`
- **Status**: ✅ Implemented (dma2/Core/Src/main.c)
- **Example**: `[0x88, 0x63, 0xEB]`
- **Checksum**: `0x88 ^ 0x63 = 0xEB`
- **Effect**: Clears AP_STATUS_HDG_ACTIVE bit, updates display

#### ALT:MODE_ON (0x64)
Activate altitude mode.

- **Command byte**: `0x64`
- **Status**: ✅ Implemented (dma2/Core/Src/main.c)
- **Example**: `[0x88, 0x64, 0xEC]`
- **Checksum**: `0x88 ^ 0x64 = 0xEC`
- **Effect**: Sets AP_STATUS_ALT_ACTIVE bit, updates display

#### ALT:MODE_OFF (0x65)
Deactivate altitude mode.

- **Command byte**: `0x65`
- **Status**: ✅ Implemented (dma2/Core/Src/main.c)
- **Example**: `[0x88, 0x65, 0xED]`
- **Checksum**: `0x88 ^ 0x65 = 0xED`
- **Effect**: Clears AP_STATUS_ALT_ACTIVE bit, updates display

#### VS:MODE_ON (0x66)
Activate vertical speed mode.

- **Command byte**: `0x66`
- **Status**: ✅ Implemented (dma2/Core/Src/main.c)
- **Example**: `[0x88, 0x66, 0xEE]`
- **Checksum**: `0x88 ^ 0x66 = 0xEE`
- **Effect**: Sets AP_STATUS_VS_ACTIVE bit, updates display

#### VS:MODE_OFF (0x67)
Deactivate vertical speed mode.

- **Command byte**: `0x67`
- **Status**: ✅ Implemented (dma2/Core/Src/main.c)
- **Example**: `[0x88, 0x67, 0xEF]`
- **Checksum**: `0x88 ^ 0x67 = 0xEF`
- **Effect**: Clears AP_STATUS_VS_ACTIVE bit, updates display

## Implementation Notes

### STM32 Implementation (dma2/Core/Src/main.c)

**Transmit (STM32 → PC):**
- Uses `HAL_UART_Transmit_DMA()` for non-blocking transmission
- Each button has its own dedicated 4-byte DMA buffer to prevent race conditions
- Checksum calculated before sending: `START ^ COMMAND ^ OPERAND`
- Button polling in main loop (lines 213-220)

**Receive (PC → STM32):**
- Uses `HAL_UART_Receive_DMA()` with 3-byte buffer
- Callback function: `HAL_UART_RxCpltCallback()` (lines 434-467)
- Validates START byte (`0x88`) and checksum before processing
- Controls GPIO pins (LED ON/OFF) and updates OLED display
- Automatically restarts DMA reception after each frame

### PC/C# Implementation (driver/Receiver2.cs)

**Receive (STM32 → PC):**
- Uses async reads: `SerialPort.BaseStream.ReadAsync()` (lines 134-135)
- Circular buffer approach with dynamic `List<byte>` (lines 126-164)
- Frame synchronization: scans for START byte `0xAA` (lines 180-193)
- Checksum validation before processing (lines 212-223)
- Handles signed operands for SET commands using `sbyte` cast (lines 332, 344, 356)
- Statistics tracking: valid frames, checksum errors, unknown commands

**Transmit (PC → STM32):**
- Uses `SerialPort.Write()` for synchronous transmission
- 3-byte frame format with checksum: `START ^ COMMAND`
- Console input handler for LED commands (lines 266-307)

### Error Handling

**PC Side:**
1. **Lost START byte**: Scans buffer for next `0xAA`, discards garbage bytes
2. **Checksum mismatch**: Removes bad START byte, attempts resync
3. **Unknown command**: Logs warning with command/operand, continues operation
4. **Statistics**: Tracks valid frames, checksum errors, unknown commands

**STM32 Side:**
1. **Invalid START byte**: Silently discards frame
2. **Checksum mismatch**: Silently discards frame
3. **Unknown command**: Silently ignores
4. **DMA auto-restart**: Reception automatically resumes after each frame

## Known Issues

### Implementation Status
Most commands are currently marked as "⚠️ Planned" and need implementation in STM32 firmware. Only the encoder delta commands (HDG:DELTA, ALT:DELTA, VS:DELTA) are currently implemented.

## Future Extensions

### Planned Features
1. **Rotary Encoder Support**: ✅ Implemented
   - Uses TIM3 Timer Encoder Mode (PA6/PA7) with hardware quadrature decoding
   - Encoder rotation sends HDG:DELTA (0x11), ALT:DELTA (0x21), or VS:DELTA (0x31) based on current mode
   - TI12 mode (4x resolution) with detent filtering - only full mechanical clicks transmitted
   - 50ms throttling and 200ms button debouncing
2. **Debouncing**: ✅ Implemented for encoder button (200ms debounce)
3. **Button Toggle Commands**: BTN:AP_TOGGLE (0x50), BTN:HDG_TOGGLE (0x51), BTN:VS_TOGGLE (0x52), BTN:ALT_TOGGLE (0x53)
4. **BARO:DELTA Command**: Barometric pressure adjustment via encoder (0x41)
5. **Status Feedback**: PC → STM32 acknowledgment frames
6. **Heartbeat/Keepalive**: Detect connection loss

### Reserved Command IDs
- `0x00-0x0F`: System commands (ping, heartbeat, status, etc.)
- `0x10-0x1F`: Heading control (DELTA)
- `0x20-0x2F`: Altitude control (DELTA)
- `0x30-0x3F`: Vertical speed control (DELTA)
- `0x40-0x4F`: Barometric pressure control (DELTA)
- `0x50-0x5F`: Button toggle states (AP, HDG, VS, ALT)
- `0x60-0xFF`: Available for future use

### Multi-byte Operands
If a command requires >1 byte of data, consider:
1. **Multi-packet approach**: Send consecutive frames with sequence numbers
2. **Extended frame format**: Define new START bytes for larger frames
3. **Packed data**: Use bit fields within single operand byte

## Communication Parameters
- **Baud rate**: 115200
- **Data bits**: 8
- **Parity**: None
- **Stop bits**: 1
- **Flow control**: None (hardware flow control disabled)

## Hardware Configuration

### STM32 Pin Configuration
- **UART**: USART2 (PA2=TX, PA3=RX)
- **Buttons**: PC0-PC3 (BTN_0 to BTN_3) with internal pull-up
- **Rotary Encoder**: TIM3 Encoder Mode (PA6=TIM3_CH1, PA7=TIM3_CH2), BTN_KNOB for mode toggle
- **LED**: PA5 (LD2, onboard LED)
- **Display**: I2C1 (PB6=SCL, PB7=SDA) for SSD1306 OLED

### DMA Configuration
- **TX DMA**: DMA1 Channel 2 (USART2_TX)
- **RX DMA**: DMA1 Channel 3 (USART2_RX)
- **Interrupt**: DMA1_Ch2_3_DMA2_Ch1_2_IRQn (Priority 0)
