# DW1000 MicroPython Examples

This directory contains clean, production-ready examples for the DW1000 UWB module.

## Examples Overview

### Core Examples
- **`transmitter.py`** - Simple UWB transmitter sending periodic messages
- **`receiver_polling.py`** - Basic receiver using polling mode
- **`receiver_interrupt.py`** - Professional receiver with hardware interrupts and callbacks

## Quick Start

### 1. Transmitter
```bash
mpremote run transmitter.py
```
Sends "Hello UWB World!" messages every second on channel 2.

### 2. Polling Receiver
```bash
mpremote run receiver_polling.py
```
Simple reception using status polling. Good for learning and debugging.

### 3. Interrupt Receiver (Recommended)
```bash
mpremote run receiver_interrupt.py
```
Professional interrupt-driven reception with optimal performance.

## Hardware Configuration

All examples use the same pin configuration:
```python
SPI_FREQ = 1000000      # 1 MHz SPI
CS_PIN = 17             # Chip Select
RESET_PIN = 21          # Reset 
IRQ_PIN = 20            # Interrupt
```

**SPI0 Pins:**
- SCK: Pin 18
- MOSI: Pin 19  
- MISO: Pin 16

## Communication Settings

All examples use compatible settings:
- **Channel:** 2
- **Data Rate:** BR_850K (850 kbps for best reliability)
- **Auto RX:** Enabled (continuous reception)

## Performance Comparison

| Example | Mode | CPU Usage | Latency | Recommended For |
|---------|------|-----------|---------|-----------------|
| `receiver_polling.py` | Polling | High | ~10ms | Learning, debugging |
| `receiver_interrupt.py` | IRQ + Callbacks | Low | <1ms | Production, ranging |

## Usage Tips

1. **Start transmitter first** on one device
2. **Run receiver** on another device  
3. **Check pin connections** if no frames received
4. **Use interrupt receiver** for best performance
5. **Monitor frame counters** for reception quality

## Interrupt System

The interrupt receiver demonstrates our hybrid IRQ implementation:
- **Hardware IRQ** triggers immediately on frame reception
- **C-level processing** in `dwt.process_events()`
- **Python callbacks** for application logic
- **MicroPython compatibility** with Pin.irq() system

This achieves hardware-level performance while maintaining framework compatibility.