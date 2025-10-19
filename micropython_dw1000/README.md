# MicroPython DW1000 UWB Module

A **production-ready** MicroPython C module providing comprehensive access to the DecaWave DW1000 Ultra-Wideband (UWB) transceiver using the proven **lab11 DW1000 driver**.

## Overview

This module provides a complete MicroPython interface to the DW1000 UWB transceiver, enabling:
- **Ultra-wideband communication** with event-driven callbacks
- **Precise distance measurement** (ranging) capabilities  
- **Indoor positioning systems** development
- **Time-of-flight measurements** with nanosecond precision
- **Robust error recovery** with automatic reset mechanisms

## Key Features

- **🔥 Event-Driven Architecture**: Full callback system for non-blocking operation
- **🛡️ Robust Error Recovery**: Automatic timeout and error state management
- **⚡ Simple Initialization**: Easy device setup with clear error handling
- **🔍 Enhanced Diagnostics**: Real-time register access and frame analysis
- **🏗️ Lab11 Driver Integration**: Uses proven, stable DW1000 driver
- **📡 19 Python Methods**: Complete API coverage for UWB development

## Status: ✅ PRODUCTION READY

✅ **Full callback system** working with real frame reception  
✅ **Event-driven architecture** tested and stable  
✅ **Clear initialization process** with robust error handling  
✅ **Enhanced diagnostics** for development and debugging  
✅ **Cross-device communication** verified at 850K data rate

## Architecture

**Lab11 Driver Integration:**
- **`dw1000-driver/`**: Official DecaWave C driver (lab11 fork) - **read-only dependency**
- **`micropython_dw1000/`**: Thin MicroPython wrapper - **never modify driver, only wrapper**
- **`polypoint/`**: Reference implementation showing proper driver usage patterns

**Key Principle**: This module is a **thin wrapper** around the proven lab11 DW1000 driver. All UWB functionality comes from the stable lab11 driver. PolyPoint demonstrates correct usage patterns.

## Project Structure

```
micropython_dw1000/
├── dw1000/                     # Main module directory
│   ├── moddw1000.c             # MicroPython C module (callbacks + configuration)
│   ├── dw1000_hal.c            # Hardware Abstraction Layer
│   ├── dw1000_hal.h            # HAL header with callback support
│   ├── micropython.mk          # Make build configuration
│   └── micropython.cmake       # CMake build configuration
├── micropython.cmake           # Top-level CMake config
├── examples/                   # Usage examples and patterns
└── README.md                   # This file
```

## Hardware Requirements

- **DW1000 Module**: Any DW1000 breakout board or module
- **Microcontroller**: Raspberry Pi Pico/Pico W (primary target)
- **SPI Interface**: 3-wire SPI connection at 1MHz
- **GPIO Pins**: CS (required), Reset and IRQ (optional but recommended)
- **Power**: Stable 3.3V power supply for DW1000
- **Debug**: Raspberry Pi Debug Probe recommended for development

**Standard Pin Configuration (Pico):**
- SPI0: SCK=Pin 18, MOSI=Pin 19, MISO=Pin 16
- CS=Pin 17, Reset=Pin 21, IRQ=Pin 20

## Building

### Requirements
- **MicroPython source tree** with USER_C_MODULES support
- **ARM cross-compiler**: `arm-none-eabi-gcc` for Pico targets
- **CMake**: Version 3.16+ for build configuration

### 1. Clone with Submodules

```bash
git clone --recursive https://github.com/yourusername/dw1000-mp.git
cd dw1000-mp
```

### 2. Build for Raspberry Pi Pico

```bash
cd micropython/ports/rp2
make USER_C_MODULES=../../../micropython_dw1000/micropython.cmake BOARD=RPI_PICO -j16
```

### 3. Build for Raspberry Pi Pico W

```bash
cd micropython/ports/rp2  
make USER_C_MODULES=../../../micropython_dw1000/micropython.cmake BOARD=RPI_PICO_W -j16
```

### 4. Flash Firmware

```bash
# Copy firmware to Pico in BOOTSEL mode
cp build-RPI_PICO/firmware.uf2 /media/user/RPI-RP2/
```

## Quick Start

### Basic Setup

```python
import machine
import dw1000

# Configure SPI and pins
spi = machine.SPI(0, baudrate=1000000, polarity=0, phase=0,
                 sck=machine.Pin(18), mosi=machine.Pin(19), miso=machine.Pin(16))
cs_pin = machine.Pin(17, machine.Pin.OUT)
reset_pin = machine.Pin(21, machine.Pin.OUT)  # Recommended
irq_pin = machine.Pin(20, machine.Pin.IN)     # Recommended

# Create DW1000 instance and initialize
dwt = dw1000.DW1000(spi, cs_pin, reset_pin, irq_pin)
dwt.init()  # Initialize the device

# Verify connection
device_id = dwt.read_device_id()
print(f"Device ID: 0x{device_id:08X}")  # Should show 0xDECA0130

# Configure for operation (new keyword argument syntax)
dwt.configure(channel=2, data_rate=dw1000.BR_850K)  # Recommended for reliability
```

### Event-Driven Receiver (Recommended)

```python
# Frame counter
frame_count = 0

def rx_callback(frame_info):
    global frame_count
    frame_count += 1
    
    if frame_info['event'] == 'rx_good':
        data = frame_info['data']
        length = frame_info['length']
        timestamp = frame_info['timestamp']
        
        print(f"Frame #{frame_count}: {data}")
        print(f"Length: {length}, Timestamp: {timestamp}")

# Register callback and start receiving
dwt.set_rx_callback(rx_callback)
dwt.rx_enable()

# Event processing loop
while True:
    dwt.process_events()  # Triggers callbacks
    time.sleep_ms(10)
```

### Simple Transmitter

```python
# Transmit messages
message_count = 0

while True:
    message = f"Hello UWB #{message_count}".encode()
    dwt.tx_frame(message)
    
    message_count += 1
    time.sleep(1)  # 1 second interval
```

## Complete API Reference

### DW1000 Class

#### Constructor
```python
DW1000(spi, cs_pin, reset_pin=None, irq_pin=None)
```
Creates a DW1000 instance with **automatic initialization**.

**Parameters:**
- `spi`: SPI object configured for DW1000 (1MHz, CPOL=0, CPHA=0)
- `cs_pin`: Chip select pin object (machine.Pin.OUT)
- `reset_pin`: Optional reset pin object (machine.Pin.OUT) - **recommended**
- `irq_pin`: Optional interrupt pin object (machine.Pin.IN) - **recommended**

**Returns:** DW1000 instance, ready for use

**Raises:**
- `OSError(MP_EIO)`: Hardware communication failure
- `RuntimeError`: Invalid device ID or initialization failure

---

### Core Methods

#### `read_device_id()`
Read the DW1000 device identification register.

**Returns:** `int` - Device ID (should be `0xDECA0130`)

**Example:**
```python
device_id = dwt.read_device_id()
if device_id == 0xDECA0130:
    print("✅ DW1000 detected")
```

---

#### `configure(**kwargs)`
Configure the DW1000 with keyword arguments for all configuration parameters.

**Parameters (all optional with smart defaults):**
- `channel`: `int` - UWB channel (1-7), default: 5
- `prf`: `int` - Pulse Repetition Frequency (PRF_16M, PRF_64M), default: PRF_64M  
- `tx_preamble_length`: `int` - TX preamble length, default: PLEN_128
- `rx_pac`: `int` - RX Preamble Acquisition Chunk size, default: PAC8
- `tx_code`: `int` - TX preamble code (1-20), default: 9
- `rx_code`: `int` - RX preamble code (1-20), default: 9
- `non_standard_sfd`: `bool` - Use non-standard SFD, default: True
- `data_rate`: `int` - Data rate (BR_110K, BR_850K, BR_6M8), default: BR_6M8
- `phr_mode`: `int` - PHR mode (PHRMODE_STD, PHRMODE_EXT), default: PHRMODE_STD
- `sfd_timeout`: `int` - SFD timeout in symbols, default: 129

**Examples:**

**Basic configuration (recommended for most applications):**
```python
# Simple reliable configuration
dwt.configure(channel=2, data_rate=dw1000.BR_850K)
```

**High-performance ranging configuration:**
```python
# Optimized for precision ranging
dwt.configure(
    channel=2,
    prf=dw1000.PRF_64M,
    tx_preamble_length=dw1000.PLEN_128,
    data_rate=dw1000.BR_6M8,
    non_standard_sfd=True
)
```

**Long-range configuration:**
```python
# Maximize range with slower data rate
dwt.configure(
    channel=1,
    prf=dw1000.PRF_16M,
    tx_preamble_length=dw1000.PLEN_1024,
    rx_pac=dw1000.PAC32,
    data_rate=dw1000.BR_110K
)
```

**Default configuration (called with no parameters):**
```python
# Uses optimal ranging settings (PolyPoint-style)
dwt.configure()  # Channel 5, 64MHz PRF, 128 preamble, 6.8M data rate
```

---

#### `tx_frame(data)`
Transmit a data frame.

**Parameters:**
- `data`: `bytes` - Data to transmit (max ~1000 bytes)

**Raises:**
- `RuntimeError`: Device not initialized
- `OSError`: Transmission failure

**Example:**
```python
message = b"Hello UWB World!"
dwt.tx_frame(message)
```

---

#### `rx_enable()`
Enable the receiver for incoming frames.

**Note:** Use with polling or callbacks for frame detection.

**Example:**
```python
dwt.rx_enable()
# Now ready to receive frames
```

---

#### `read_rx_data()`
Read received frame data.

**Returns:** `bytes` - Received frame data

**Note:** Call after detecting frame reception via status or callback.

**Example:**
```python
# After frame detection
data = dwt.read_rx_data()
print(f"Received: {data}")
```

---

### Status and Diagnostics

#### `get_status()`
Read the device status register.

**Returns:** `int` - 32-bit status register value

**Key Status Bits:**
- `0x0080`: TX Frame Sent
- `0x2000`: RX Frame Good  
- `0x0400`: RX Frame Error
- `0x8000`: RX Frame Ready

**Example:**
```python
status = dwt.get_status()
frame_good = (status & 0x2000) != 0
frame_error = (status & 0x0400) != 0
```

---

#### `get_sys_state()`
Read the system state register for diagnostics.

**Returns:** `int` - 32-bit system state value

**Example:**
```python
state = dwt.get_sys_state()
print(f"System state: 0x{state:08X}")
```

---

#### `get_rx_finfo()`
Read RX frame info register (critical for frame length detection).

**Returns:** `int` - RX frame info register

**Frame Length Extraction:**
```python
rx_finfo = dwt.get_rx_finfo()
frame_length = rx_finfo & 0x000003FF  # Bits 0-9
```

---

#### `read_register(register_id)`
Read any 32-bit register for diagnostics.

**Parameters:**
- `register_id`: `int` - Register ID (0x00-0x3F)

**Returns:** `int` - 32-bit register value

**Example:**
```python
device_id = dwt.read_register(0x00)  # Device ID register
```

---

### Reset and Control

#### `force_trx_off()`
Force the transceiver off (stops TX/RX operations).

**Use:** Error recovery, state resets

**Example:**
```python
dwt.force_trx_off()  # Stop all radio operations
```

---

#### `rx_reset()`
Reset the RX buffers and state.

**Use:** Clear RX errors, prepare for new reception

**Example:**
```python
dwt.rx_reset()  # Clear RX buffers
```

---

#### `sync_rx_bufptrs()`
Synchronize RX buffer pointers.

**Use:** Part of proper RX reset sequence

**Example:**
```python
# Proper RX reset sequence
dwt.force_trx_off()
dwt.rx_reset()
dwt.sync_rx_bufptrs()
dwt.rx_enable()
```

---

#### `deinit()`
Deinitialize the device and release resources.

**Example:**
```python
dwt.deinit()  # Cleanup
```

---

### 🔥 Event-Driven Callbacks

#### `set_rx_callback(callback)`
Register a callback for RX frame events.

**Parameters:**
- `callback`: `function` or `None` - Callback function

**Callback Signature:**
```python
def rx_callback(frame_info):
    # frame_info is a dict with:
    # 'event': str - 'rx_good', 'rx_error', etc.
    # 'status': int - Status register value
    # 'length': int - Frame length in bytes
    # 'data': bytes - Received frame data
    # 'timestamp': int - RX timestamp
    # 'rx_time_ms': int - Reception time in ms
    pass
```

**Example:**
```python
def on_frame_received(frame_info):
    if frame_info['event'] == 'rx_good':
        print(f"Received: {frame_info['data']}")

dwt.set_rx_callback(on_frame_received)
```

---

#### `set_tx_callback(callback)`
Register a callback for TX completion events.

**Parameters:**
- `callback`: `function` or `None` - Callback function

**Example:**
```python
def on_tx_complete(tx_info):
    print("Frame transmitted successfully")

dwt.set_tx_callback(on_tx_complete)
```

---

#### `set_error_callback(callback)`
Register a callback for error events.

**Parameters:**
- `callback`: `function` or `None` - Callback function

**Example:**
```python
def on_error(error_info):
    print(f"Error occurred: {error_info}")

dwt.set_error_callback(on_error)
```

---

#### `set_timeout_callback(callback)`
Register a callback for timeout events.

**Parameters:**
- `callback`: `function` - Callback function (no parameters)

**Example:**
```python
def on_timeout():
    print("Operation timed out")

dwt.set_timeout_callback(on_timeout)
```

---

#### `process_events()`
Process pending events and trigger callbacks.

**Must be called regularly** in main loop for event-driven operation.

**Example:**
```python
while True:
    dwt.process_events()  # Triggers callbacks
    time.sleep_ms(10)
```

---

#### `set_auto_rx(enable)`
Enable/disable automatic RX mode.

**Parameters:**
- `enable`: `bool` - True to enable auto RX

**Example:**
```python
dwt.set_auto_rx(True)   # Automatic RX restart after frame
dwt.set_auto_rx(False)  # Manual RX control
```

---

### Constants

#### Data Rates
```python
dw1000.BR_110K    # 110 kbps (long range)
dw1000.BR_850K    # 850 kbps (recommended - best reliability)
dw1000.BR_6M8     # 6.8 Mbps (high speed)
```

#### Pulse Repetition Frequency (PRF)
```python
dw1000.PRF_16M    # 16 MHz PRF (longer range)
dw1000.PRF_64M    # 64 MHz PRF (better precision)
```

#### TX Preamble Lengths
```python
dw1000.PLEN_64      # 64 symbols
dw1000.PLEN_128     # 128 symbols (default)
dw1000.PLEN_256     # 256 symbols
dw1000.PLEN_512     # 512 symbols
dw1000.PLEN_1024    # 1024 symbols
dw1000.PLEN_1536    # 1536 symbols
dw1000.PLEN_2048    # 2048 symbols
dw1000.PLEN_4096    # 4096 symbols
```

#### RX Preamble Acquisition Chunk (PAC) Sizes
```python
dw1000.PAC8     # 8 symbols (for preambles ≤128)
dw1000.PAC16    # 16 symbols (for preamble 256)
dw1000.PAC32    # 32 symbols (for preamble 512)
dw1000.PAC64    # 64 symbols (for preambles ≥1024)
```

#### PHR Modes
```python
dw1000.PHRMODE_STD    # Standard IEEE 802.15.4 PHR
dw1000.PHRMODE_EXT    # Extended DW1000-specific PHR
```

#### Device Information
```python
dw1000.DEVICE_ID  # Expected device ID: 0xDECA0130
```

---

### Class Behavior and Error Recovery

#### Initialization Process
The DW1000 class requires explicit initialization after object creation:
- Constructor creates the object and sets up pin assignments
- `init()` method loads microcode and configurations
- Sets up default parameters and verifies device communication
- Device ready for use after successful `init()` call

#### Error Handling Philosophy
- **Hardware errors** → `OSError(MP_EIO)` - SPI communication failures
- **Configuration errors** → `RuntimeError` - Invalid parameters or device state
- **Automatic recovery** - Built-in timeout and reset mechanisms
- **Defensive programming** - All methods check initialization state

#### Robust Operation Patterns

**1. Receiver Reset Pattern (handles blocking issues):**
```python
def reset_receiver():
    """Complete receiver reset - fixes all RX blocking issues"""
    dwt.force_trx_off()
    time.sleep_ms(2)      # Allow state transition
    dwt.rx_reset()
    dwt.sync_rx_bufptrs()
    time.sleep_ms(1)      
    dwt.rx_enable()

# Use after each frame or on timeout
reset_receiver()
```

**2. Status-Based Error Detection:**
```python
def check_errors(status):
    """Detect and handle various error conditions"""
    if status & 0x0400:   # RX frame error
        reset_receiver()
    elif status & 0x1000: # PHR error  
        reset_receiver()
    elif status & 0x10000: # Reed Solomon error
        reset_receiver()
```

**3. Timeout-Based Recovery:**
```python
ACTIVITY_TIMEOUT = 5000    # 5 seconds
last_activity = time.ticks_ms()

def monitor_activity():
    global last_activity
    current_time = time.ticks_ms()
    
    if time.ticks_diff(current_time, last_activity) > ACTIVITY_TIMEOUT:
        reset_receiver()  # Automatic recovery
        last_activity = current_time
```

#### Thread Safety
- **Single-threaded operation** - Not thread-safe
- **IRQ compatibility** - Callbacks can be called from IRQ context
- **No blocking calls** - All operations return immediately

---

## Error Codes and Status Values

### Exception Types
- `OSError(MP_EIO)`: Hardware/SPI communication error
- `RuntimeError`: Device configuration or state error

### Status Register Bits (get_status())
```python
# Key status bits for frame detection
TX_SENT         = 0x0080    # Bit 7: TX frame sent
RX_GOOD         = 0x2000    # Bit 13: Good frame received  
RX_ERROR        = 0x0400    # Bit 10: Frame error
RX_READY        = 0x8000    # Bit 15: Frame ready
PHR_ERROR       = 0x1000    # Bit 12: PHR error
REED_ERROR      = 0x10000   # Bit 16: Reed Solomon error
FRAME_WAIT_TO   = 0x20000   # Bit 17: Frame wait timeout
```

### Device ID Values
```python
EXPECTED_DEVICE_ID = 0xDECA0130  # DW1000 device ID
INVALID_DEVICE_ID  = 0x00000000  # Communication failure
```

---

## 🔧 Complete Configuration Parameters Guide

The `configure()` method accepts keyword arguments for comprehensive DW1000 configuration. All parameters are optional with intelligent defaults optimized for reliable operation.

### Channel Configuration
```python
# Supported UWB channels (1-7) with center frequencies
channel=1   # 3494.4 MHz - European band
channel=2   # 3993.6 MHz - Global band (recommended for reliability)  
channel=3   # 4492.8 MHz - Global band
channel=4   # 3993.6 MHz - Global band (same frequency as channel 2)
channel=5   # 6489.6 MHz - Global band (default - good performance)
channel=7   # 6489.6 MHz - Global band (same frequency as channel 5)

# Usage examples
dwt.configure(channel=2)  # Reliable global frequency
dwt.configure(channel=5)  # Default high-performance frequency
```

### Pulse Repetition Frequency (PRF)
```python
# PRF affects range, power consumption, and precision
prf=dw1000.PRF_16M    # 16 MHz - Longer range, lower power
prf=dw1000.PRF_64M    # 64 MHz - Better precision (default)

# Usage examples
dwt.configure(prf=dw1000.PRF_16M)  # For maximum range
dwt.configure(prf=dw1000.PRF_64M)  # For ranging precision
```

### TX Preamble Length
```python
# Longer preambles improve range and sensitivity
tx_preamble_length=dw1000.PLEN_64      # 64 symbols - Fast acquisition
tx_preamble_length=dw1000.PLEN_128     # 128 symbols (default)
tx_preamble_length=dw1000.PLEN_256     # 256 symbols
tx_preamble_length=dw1000.PLEN_512     # 512 symbols  
tx_preamble_length=dw1000.PLEN_1024    # 1024 symbols - Best range
tx_preamble_length=dw1000.PLEN_1536    # 1536 symbols
tx_preamble_length=dw1000.PLEN_2048    # 2048 symbols
tx_preamble_length=dw1000.PLEN_4096    # 4096 symbols - Maximum range

# Usage examples
dwt.configure(tx_preamble_length=dw1000.PLEN_1024)  # Long range
dwt.configure(tx_preamble_length=dw1000.PLEN_128)   # Fast communication
```

### RX Preamble Acquisition Chunk (PAC)
```python
# PAC should match preamble length for optimal performance
rx_pac=dw1000.PAC8     # 8 symbols (default) - for preambles ≤128
rx_pac=dw1000.PAC16    # 16 symbols - for preamble 256
rx_pac=dw1000.PAC32    # 32 symbols - for preamble 512  
rx_pac=dw1000.PAC64    # 64 symbols - for preambles ≥1024

# PAC recommendations based on preamble length
# PLEN_64, PLEN_128 → PAC8
# PLEN_256 → PAC16
# PLEN_512 → PAC32
# PLEN_1024+ → PAC64

# Usage examples
dwt.configure(tx_preamble_length=dw1000.PLEN_256, rx_pac=dw1000.PAC16)
dwt.configure(tx_preamble_length=dw1000.PLEN_1024, rx_pac=dw1000.PAC64)
```

### Preamble Codes
```python
# TX and RX codes must match between communicating devices
# Code selection depends on channel and PRF
tx_code=1    # Preamble code 1-20 (channel/PRF dependent)
rx_code=1    # Should match TX code for communication

# Recommended codes by channel (PRF 64MHz):
# Channel 1: codes 1, 2
# Channel 2: codes 3, 4  
# Channel 3: codes 5, 6
# Channel 4: codes 7, 8
# Channel 5: codes 9, 10 (default uses code 9)
# Channel 7: codes 11, 12

# Usage examples
dwt.configure(channel=2, tx_code=3, rx_code=3)  # Channel 2 with code 3
dwt.configure(channel=5, tx_code=9, rx_code=9)  # Channel 5 with code 9 (default)
```

### Data Rate
```python
# Data rate affects range, power, and transmission time
data_rate=dw1000.BR_110K    # 110 kbps - Maximum range, minimum power
data_rate=dw1000.BR_850K    # 850 kbps - Best reliability (recommended)
data_rate=dw1000.BR_6M8     # 6.8 Mbps - Minimum range, fastest (default)

# Performance characteristics:
# BR_110K: ~300m range, lowest power, slowest
# BR_850K: ~150m range, good power, reliable (recommended for most apps)
# BR_6M8:  ~80m range, highest power, fastest transmission

# Usage examples
dwt.configure(data_rate=dw1000.BR_850K)  # Recommended for reliability
dwt.configure(data_rate=dw1000.BR_110K)  # Maximum range applications
dwt.configure(data_rate=dw1000.BR_6M8)   # High-speed applications
```

### Special Frame Detection (SFD)
```python
# Standard vs Non-Standard SFD
non_standard_sfd=False    # Use IEEE 802.15.4 standard SFD
non_standard_sfd=True     # Use DW1000 proprietary SFD (default)

# Non-standard SFD provides:
# - Better performance in multipath environments
# - Improved frame detection reliability
# - Recommended for most applications

# Usage examples
dwt.configure(non_standard_sfd=True)   # Better performance (default)
dwt.configure(non_standard_sfd=False)  # Standards compliance
```

### PHR (PHY Header) Mode  
```python
# PHR mode affects frame format and compatibility
phr_mode=dw1000.PHRMODE_STD    # Standard IEEE 802.15.4 PHR (default)
phr_mode=dw1000.PHRMODE_EXT    # Extended DW1000-specific PHR

# Standard PHR:
# - IEEE 802.15.4 compatible
# - Maximum frame length ~127 bytes
# - Best interoperability

# Extended PHR:
# - DW1000-specific extensions
# - Longer frame lengths possible
# - Better for custom protocols

# Usage examples  
dwt.configure(phr_mode=dw1000.PHRMODE_STD)  # Standards compliant (default)
dwt.configure(phr_mode=dw1000.PHRMODE_EXT)  # Extended features
```

### SFD Timeout
```python
# SFD timeout in symbol periods
sfd_timeout=129        # Default timeout (preamble_length + 1 + 8 - 8)
sfd_timeout=1041       # Conservative timeout for difficult environments

# The timeout should be:
# - At least preamble_length + 1 + 8 - 8
# - Higher values more tolerant of interference
# - Lower values faster error detection

# Usage examples
dwt.configure(sfd_timeout=129)    # Default (matches 128 preamble)
dwt.configure(sfd_timeout=1041)   # Conservative for noisy environments
```

### 🎯 Pre-configured Profiles

Instead of manually configuring each parameter, use these tested profiles:

```python
# PROFILE 1: Maximum Reliability (recommended for most applications)
dwt.configure(
    channel=2,                              # Global frequency
    prf=dw1000.PRF_64M,                    # Good precision
    tx_preamble_length=dw1000.PLEN_128,    # Balanced performance
    rx_pac=dw1000.PAC8,                    # Matches preamble
    data_rate=dw1000.BR_850K,              # Best reliability
    non_standard_sfd=True                   # Better performance
)

# PROFILE 2: Maximum Range
dwt.configure(
    channel=1,                              # European band
    prf=dw1000.PRF_16M,                    # Lower power, longer range
    tx_preamble_length=dw1000.PLEN_1024,   # Long preamble
    rx_pac=dw1000.PAC64,                   # Matches long preamble
    data_rate=dw1000.BR_110K,              # Lowest data rate
    non_standard_sfd=True
)

# PROFILE 3: High-Speed Communication
dwt.configure(
    channel=5,                              # High frequency
    prf=dw1000.PRF_64M,                    # Fast acquisition
    tx_preamble_length=dw1000.PLEN_64,     # Short preamble
    rx_pac=dw1000.PAC8,                    # Fast acquisition
    data_rate=dw1000.BR_6M8,               # Highest speed
    non_standard_sfd=True
)

# PROFILE 4: Precision Ranging (PolyPoint-style, default)
dwt.configure()  # Uses optimized defaults:
# channel=5, prf=PRF_64M, tx_preamble_length=PLEN_128, 
# rx_pac=PAC8, data_rate=BR_6M8, non_standard_sfd=True
```

### Configuration Validation

The module validates all configuration parameters:

```python
# Invalid configurations will raise RuntimeError
try:
    dwt.configure(channel=8)  # Invalid channel
except RuntimeError as e:
    print(f"Configuration error: {e}")

# Valid ranges:
# channel: 1, 2, 3, 4, 5, 7 (6 is not supported)
# prf: PRF_16M or PRF_64M
# tx_preamble_length: PLEN_64 to PLEN_4096
# rx_pac: PAC8, PAC16, PAC32, PAC64
# tx_code, rx_code: 1-20
# data_rate: BR_110K, BR_850K, BR_6M8
# phr_mode: PHRMODE_STD, PHRMODE_EXT
```

### SPI Configuration
```python
# Required SPI settings for DW1000
spi_config = {
    'baudrate': 1000000,    # 1 MHz max
    'polarity': 0,          # CPOL = 0
    'phase': 0,             # CPHA = 0
    'bits': 8,              # 8-bit transfers
    'firstbit': machine.SPI.MSB  # MSB first
}
```

### Pin Configuration Examples
```python
# Raspberry Pi Pico (standard pinout)
CS_PIN = 17     # Chip Select
RESET_PIN = 21  # Reset (optional but recommended)  
IRQ_PIN = 20    # Interrupt (optional for callbacks)

# Alternative pinout
CS_PIN = 5      # Alternative CS
RESET_PIN = 6   # Alternative Reset
IRQ_PIN = 7     # Alternative IRQ
```

---

## Frame Structure

### Maximum Frame Sizes
```python
MAX_FRAME_SIZE = 1023    # Bytes (theoretical)
PRACTICAL_MAX = 125      # Bytes (recommended for reliability)
MIN_FRAME_SIZE = 1       # Bytes
```

### Frame Info Structure (from get_rx_finfo())
```python
def extract_frame_info(rx_finfo):
    """Extract frame information from RX_FINFO register"""
    frame_length = rx_finfo & 0x000003FF        # Bits 0-9: Frame length
    std_frame = (rx_finfo & 0x00002000) != 0    # Bit 13: Standard frame
    preamble_length = (rx_finfo >> 20) & 0x0F   # Bits 20-23: Preamble
    return {
        'length': frame_length,
        'standard': std_frame,
        'preamble_length': preamble_length
    }
```

---

## Performance Characteristics

### Communication Range (typical)
- **BR_110K**: 200+ meters (line of sight)
- **BR_850K**: 100-150 meters (recommended)  
- **BR_6M8**: 50-80 meters (high speed)

### Power Consumption (typical)
- **TX**: 150-200 mA peak
- **RX**: 100-150 mA continuous  
- **Idle**: 10-20 mA
- **Sleep**: <1 µA (when implemented)

### Timing Characteristics
- **Frame TX time**: 1-10 ms (depends on size and data rate)
- **RX enable time**: <100 µs
- **Reset time**: 1-2 ms
- **SPI transaction**: 10-50 µs per register access

---

## Development and Debugging

### Common Issues and Solutions

#### 1. Device Not Found (OSError)
```python
# Check connections
# Verify SPI wiring
# Check power supply (3.3V)
# Ensure proper grounding
```

#### 2. Frame Reception Issues
```python
# Use robust receiver reset pattern
def fix_reception():
    dwt.force_trx_off()
    time.sleep_ms(2)
    dwt.rx_reset()
    dwt.sync_rx_bufptrs()
    dwt.rx_enable()
```

#### 3. Callback Not Triggered
```python
# Ensure process_events() is called regularly
while True:
    dwt.process_events()
    time.sleep_ms(10)    # Don't block
```

### Debugging Tools
```python
def debug_status():
    """Print comprehensive device status"""
    status = dwt.get_status()
    rx_finfo = dwt.get_rx_finfo()
    sys_state = dwt.get_sys_state()
    
    print(f"Status: 0x{status:08X}")
    print(f"RX_FINFO: 0x{rx_finfo:08X}")
    print(f"SYS_STATE: 0x{sys_state:08X}")
    print(f"Frame Good: {(status & 0x2000) != 0}")
    print(f"Frame Error: {(status & 0x0400) != 0}")
    print(f"Frame Length: {rx_finfo & 0x3FF}")
```

### Test Scripts
The repository includes comprehensive test scripts:
- `test_enhanced_init.py` - Basic functionality test
- `callback_receiver.py` - Event-driven receiver example
- `enhanced_receiver_working.py` - Polling-based receiver
- `test_callbacks.py` - Callback system validation

---

## Lab11 Driver Integration

This module is a **thin wrapper** around the proven Lab11 DW1000 driver:

- **Core functionality**: All UWB operations provided by `dw1000-driver/`
- **Wrapper role**: MicroPython C module only provides language binding
- **Proven reliability**: Lab11 driver used in production UWB systems
- **Reference implementation**: See `polypoint/` for usage patterns
- **No modifications**: Never modify files in `dw1000-driver/` directory

### Driver Architecture
```
MicroPython Application
        ↓
micropython_dw1000/ (wrapper module)
        ↓  
dw1000-driver/ (lab11 driver - READ ONLY)
        ↓
DW1000 Hardware
```

### Adding New Features
1. Find equivalent `dwt_*` function in `deca_device_api.h`
2. Implement wrapper in `moddw1000.c`
3. Test using PolyPoint patterns
4. **Never modify** lab11 driver files

## Platform Notes

### ESP32
- Tested with ESP32-S3
- Use hardware SPI (HSPI or VSPI)
- Ensure proper level shifting if needed

### Raspberry Pi Pico
- Works with both SPI controllers
- Use 3.3V logic levels

### STM32
- Compatible with STM32 SPI peripherals
- May require clock configuration adjustment

## Troubleshooting

### Device ID Read Fails
- Check SPI wiring and configuration
- Verify power supply is stable 3.3V
- Ensure CS pin is properly configured

### Communication Errors  
- Check SPI clock frequency (start with 1MHz)
- Verify SPI mode (CPOL=0, CPHA=0)
- Check for proper grounding

### Build Errors
- Ensure submodules are initialized: `git submodule update --init --recursive`
- Verify include paths in build configuration
- Check MicroPython version compatibility

## Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch
3. Add tests for new functionality
4. Submit a pull request

## License

This project includes:
- **MicroPython module code**: MIT License
- **DW1000 driver code**: DecaWave license (see `dw1000-driver/disclaimer.txt`)

## References

- [DW1000 User Manual](https://www.decawave.com/dw1000/usermanual/)
- [MicroPython C Module Documentation](https://docs.micropython.org/en/latest/develop/cmodules.html)
- [DecaWave DW1000 Driver](https://github.com/lab11/dw1000-driver)