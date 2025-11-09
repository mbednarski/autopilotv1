# UART Communication Protocol

## Overview
This document describes the minimalistic 4-byte binary protocol used for STM32 ↔ PC communication over UART.

## Protocol Specification

### Frame Structure
Each message consists of exactly 4 bytes:

```
Byte 0: START    = 0xAA (frame synchronization marker)
Byte 1: COMMAND  = 0x00-0xFF (command identifier)
Byte 2: OPERAND  = 0x00-0xFF (command parameter)
Byte 3: CHECKSUM = START ^ COMMAND ^ OPERAND (XOR checksum)
```

### Checksum Calculation
The checksum is calculated using XOR of the first three bytes:
```c
checksum = 0xAA ^ command ^ operand
```

This provides fast error detection with minimal computational overhead.

### Example Frame
```
Command: HDG:SET with delta = +5
Bytes: [0xAA, 0x11, 0x05, 0xAE]
       [START][CMD ][OPR ][CHK ]

Checksum: 0xAA ^ 0x11 ^ 0x05 = 0xAE
```

## Command Set

### HDG:RESET (0x10)
Reset the heading to default position.

- **Command byte**: `0x10`
- **Operand**: Always `0x00` (unused)
- **Direction**: STM32 → PC
- **Example**: `[0xAA, 0x10, 0x00, 0xBA]`

### HDG:SET (0x11)
Set heading delta (increment/decrement).

- **Command byte**: `0x11`
- **Operand**: Signed 8-bit integer (-128 to +127)
  - Positive values: increment heading
  - Negative values: decrement heading
- **Direction**: STM32 → PC
- **Examples**:
  - Set +10: `[0xAA, 0x11, 0x0A, 0xA1]`
  - Set -5: `[0xAA, 0x11, 0xFB, 0x40]` (0xFB = -5 in two's complement)
  - Set +127: `[0xAA, 0x11, 0x7F, 0x24]`

### ALT:RESET (0x20)
Reset the altitude to default position.

- **Command byte**: `0x20`
- **Operand**: Always `0x00` (unused)
- **Direction**: STM32 → PC
- **Example**: `[0xAA, 0x20, 0x00, 0x8A]`
- **Checksum**: `0xAA ^ 0x20 ^ 0x00 = 0x8A`

### ALT:SET (0x21)
Set altitude delta (increment/decrement).

- **Command byte**: `0x21`
- **Operand**: Signed 8-bit integer (-128 to +127)
  - Positive values: increment altitude
  - Negative values: decrement altitude
- **Direction**: STM32 → PC
- **Examples**:
  - Set +10: `[0xAA, 0x21, 0x0A, 0x81]`
  - Set -5: `[0xAA, 0x21, 0xFB, 0x70]` (0xFB = -5 in two's complement)
  - Set +50: `[0xAA, 0x21, 0x32, 0x99]`

### VS:RESET (0x30)
Reset the vertical speed to default position (typically 0 ft/min).

- **Command byte**: `0x30`
- **Operand**: Always `0x00` (unused)
- **Direction**: STM32 → PC
- **Example**: `[0xAA, 0x30, 0x00, 0x9A]`
- **Checksum**: `0xAA ^ 0x30 ^ 0x00 = 0x9A`

### VS:SET (0x31)
Set vertical speed delta (increment/decrement).

- **Command byte**: `0x31`
- **Operand**: Signed 8-bit integer (-128 to +127)
  - Positive values: increment vertical speed
  - Negative values: decrement vertical speed
- **Direction**: STM32 → PC
- **Examples**:
  - Set +10: `[0xAA, 0x31, 0x0A, 0x91]`
  - Set -5: `[0xAA, 0x31, 0xFB, 0x60]` (0xFB = -5 in two's complement)
  - Set +100: `[0xAA, 0x31, 0x64, 0xFF]`

## Implementation Notes

### STM32 (Transmitter)
- Use DMA for non-blocking transmission: `HAL_UART_Transmit_DMA()`
- Always calculate checksum before sending
- Ensure DMA transfer completes before modifying buffer

### PC/C# (Receiver)
- Use async reads: `SerialPort.BaseStream.ReadAsync()`
- Validate START byte (0xAA) for frame synchronization
- Verify checksum before processing command
- Discard invalid frames and resynchronize on next START byte

### Error Handling
1. **Lost START byte**: Scan buffer for next 0xAA
2. **Checksum mismatch**: Discard frame, wait for next START byte
3. **Unknown command**: Log warning, continue operation
4. **Timeout**: Detect using timestamp of last valid packet

## Future Extensions

### Reserved Command IDs
- `0x00-0x0F`: System commands (ping, heartbeat, etc.)
- `0x10-0x1F`: Heading control
- `0x20-0x2F`: Altitude control
- `0x30-0x3F`: Speed control
- `0x40-0x4F`: Button/switch states
- `0x50-0xFF`: Available for future use

### Multi-byte Operands
If a command requires >1 byte of data, use command variants:
- `CMD_XXX_SET1` with 1-byte operand
- `CMD_XXX_SET2` with 2-byte operand (send two consecutive packets)

## Communication Parameters
- **Baud rate**: 115200
- **Data bits**: 8
- **Parity**: None
- **Stop bits**: 1
- **Flow control**: None (for simple connections) or RTS/CTS (for reliability)
