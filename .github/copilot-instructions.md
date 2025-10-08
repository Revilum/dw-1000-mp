# DW1000 MicroPython Project - AI Coding Instructions

## Project Overview

MicroPython interface for DecaWave DW1000 Ultra-Wideband transceiver targeting **Raspberry Pi Pico/Pico W** with **DW1000 breakout boards** via SPI.

**Architecture:**
- **`dw1000-driver/`**: Official DecaWave C driver (lab11 fork) - **ALWAYS use this driver**
- **`micropython_dw1000/`**: MicroPython C wrapper module - **NEVER modify driver, only wrapper**
- **`polypoint/`**: Reference for understanding driver usage patterns and best practices

**Key Principle**: The C module is a **thin wrapper** around the lab11 DW1000 driver. All UWB functionality comes from the proven dw1000-driver by 11labs (dw1000-driver/). PolyPoint shows how to use the driver correctly.

**Hardware Setup:**
- SPI0: 1MHz, pins vary by board
- Standard: CS=Pin 17, Reset=Pin 21, IRQ=Pin 20

## Current Module Status: ✅ FULLY OPERATIONAL

### ✅ Implemented Features (19 methods):

**Core Functionality:**
```python
dwt = dw1000.DW1000(spi, cs_pin, reset_pin, irq_pin)  # Auto-init enabled
dwt.read_device_id()        # Returns 0xDECA0130
dwt.configure(config_dict)  # Channel, data_rate (BR_110K, BR_850K, BR_6M8)
dwt.tx_frame(data)         # Transmit data
dwt.rx_enable()            # Enable receiver  
dwt.read_rx_data()         # Read received data
dwt.get_status()           # Read status register
dwt.force_trx_off()        # Force transceiver off
dwt.rx_reset()             # Reset RX buffers
dwt.sync_rx_bufptrs()      # Sync RX buffer pointers
dwt.deinit()               # Cleanup
```

**Enhanced Diagnostics:**
```python
dwt.get_sys_state()        # System state register
dwt.get_rx_finfo()         # RX frame info with length detection  
dwt.read_register(reg_id)  # Read any 32-bit register
```

**🔥 Callback System (EVENT-DRIVEN):**
```python
def rx_callback(frame_info):
    # frame_info = {'event': 'rx_good', 'status': int, 'length': int, 
    #               'data': bytes, 'timestamp': int, 'rx_time_ms': int}
    print(f"Received: {frame_info['data']}")

dwt.set_rx_callback(rx_callback)      # RX frame events
dwt.set_tx_callback(tx_callback)      # TX completion events  
dwt.set_error_callback(error_callback) # Error events
dwt.set_timeout_callback(timeout_cb)   # Timeout events
dwt.process_events()                   # Trigger callbacks
dwt.set_auto_rx(True/False)           # Auto RX mode
```

## Development Workflow

**Testing:** `mpremote run <script.py>`

**Building Custom Firmware:**
```bash
cd micropython/ports/rp2
make USER_C_MODULES=../../../micropython_dw1000/micropython.cmake BOARD=RPI_PICO -j16
```

**Working Examples:**
- `callback_receiver.py` - **RECOMMENDED** - Event-driven receiver with callbacks
- `enhanced_receiver_working.py` - Polling-based receiver with enhanced diagnostics
- `test_callbacks.py` - Callback system testing
- `test_enhanced_init.py` - Basic functionality tests

## Driver Integration Principles

**Lab11 Driver Usage:**
- All DW1000 operations go through lab11 driver functions (`dwt_*`)
- HAL layer (`dw1000_hal.c`) provides MicroPython ↔ driver interface
- Never modify files in `dw1000-driver/` - it's read-only external dependency

**PolyPoint Reference:**
- Study `polypoint/` for correct driver usage patterns
- Shows proper initialization sequences, timing, and error recovery
- Demonstrates real-world UWB application architecture

**Adding New Features:**
1. Find equivalent `dwt_*` function in lab11 driver (`deca_device_api.h`)
2. Implement wrapper in `moddw1000.c` calling driver function
3. Add to `dw1000_locals_dict` table
4. Test on hardware with PolyPoint patterns

## Key Patterns

**Robust Receiver (handles blocking):**
```python
def setup_robust_receiver():
    dwt.force_trx_off()
    time.sleep_ms(2)
    dwt.rx_reset()
    dwt.sync_rx_bufptrs()
    dwt.rx_enable()

def frame_received_callback(frame_info):
    if frame_info['event'] == 'rx_good':
        data = frame_info['data']
        length = frame_info['length']
        # Process frame...
    setup_robust_receiver()  # Reset for next frame
```

**Frame Length Detection:**
```python
rx_finfo = dwt.get_rx_finfo()
frame_length = rx_finfo & 0x000003FF  # Extract length (bits 0-9)
```

**Status Monitoring:**
```python
status = dwt.get_status()
frame_good = (status & 0x2000) != 0    # Bit 13
frame_error = (status & 0x0400) != 0   # Bit 10  
tx_complete = (status & 0x0080) != 0   # Bit 7
```

## Missing Lab11 Driver Features (Implementation Candidates)

**High Priority:**
- `dwt_setrxtimeout(uint16)` - RX timeout configuration
- `dwt_readrxtimestamp()` - RX timestamp reading  
- `dwt_readtxtimestamp()` - TX timestamp reading
- `dwt_setdelayedtrxtime(uint32)` - Delayed TX/RX timing
- `dwt_setrxantennadelay(uint16)` - Antenna delay calibration
- `dwt_settxantennadelay(uint16)` - Antenna delay calibration

**Medium Priority:**
- `dwt_setsniffmode()` - Low power sniff mode
- `dwt_configuresleep()` - Sleep mode configuration
- `dwt_getpartid()` - Part ID reading
- `dwt_getlotid()` - Lot ID reading
- `dwt_setgpiodirection()/dwt_setgpiovalue()` - GPIO control

## Module Architecture
- `moddw1000.c`: Main MicroPython C module (auto-init + callbacks)
- `dw1000_hal.c/h`: Hardware abstraction layer (SPI, GPIO, timing)
- `micropython.cmake`: CMake integration for USER_C_MODULES
- Links to lab11 driver in `dw1000-driver/deca_device.c`

**Error Handling:**
- Hardware errors → `OSError(MP_EIO)`
- Configuration errors → `RuntimeError`
- All methods require `initialized` flag check

## Current Status: 🏆 PRODUCTION READY
✅ **19 methods implemented** including full callback system
✅ **Event-driven architecture** working perfectly  
✅ **Real frame reception** tested and confirmed
✅ **Auto-initialization** and robust error handling
✅ **Enhanced diagnostics** for debugging and monitoring

The module provides **professional-grade UWB development** with both polling and event-driven patterns.