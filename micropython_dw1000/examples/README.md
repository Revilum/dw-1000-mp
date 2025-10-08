# DW1000 MicroPython Examples

Simple, focused examples demonstrating UWB communication with the DW1000 module.

## Examples

### `transmitter.py`
**Simple UWB Transmitter**
- Continuously sends numbered messages
- Uses 850 kbps data rate for reliability
- 1-second interval between messages

**Usage:**
```bash
mpremote run transmitter.py
```

### `callback_receiver.py`
**Event-Driven UWB Receiver**
- Uses callbacks for efficient frame reception
- Automatic receiver reset for robust operation
- Displays received messages with statistics

**Usage:**
```bash
mpremote run callback_receiver.py
```

## Hardware Setup

Both examples use the standard pinout:
- **SPI0**: 1MHz, CPOL=0, CPHA=0
- **CS Pin**: 17 (Chip Select)
- **Reset Pin**: 21 (recommended)
- **IRQ Pin**: 20 (for callbacks)

## Quick Start

1. **Run receiver** on one Pico:
   ```bash
   mpremote run callback_receiver.py
   ```

2. **Run transmitter** on another Pico:
   ```bash
   mpremote run transmitter.py
   ```

3. **Watch communication** between devices!

## Configuration

Both examples use Channel 2 (3993.6 MHz) with 850 kbps data rate for optimal reliability and range.