# MicroPython DW1000 UWB Module

A MicroPython C module providing access to the DecaWave DW1000 Ultra-Wideband (UWB) transceiver.

## Overview

This module provides a MicroPython interface to the DW1000 UWB transceiver, enabling:
- Ultra-wideband communication
- Precise distance measurement (ranging)
- Indoor positioning systems
- Time-of-flight measurements

## Features

- **Complete DW1000 API**: Access to all major DW1000 functions
- **Hardware Abstraction**: Works with any MicroPython platform that supports SPI
- **Efficient Implementation**: Uses the official DecaWave driver with MicroPython HAL
- **Easy Integration**: Simple Python interface for complex UWB operations

## Project Structure

```
micropython_dw1000/
├── dw1000/                     # Main module directory
│   ├── moddw1000.c             # MicroPython C module implementation
│   ├── dw1000_hal.c            # Hardware Abstraction Layer
│   ├── dw1000_hal.h            # HAL header
│   ├── micropython.mk          # Make build configuration
│   └── micropython.cmake       # CMake build configuration
├── micropython.cmake           # Top-level CMake config
├── examples/                   # Usage examples
└── README.md                   # This file
```

## Hardware Requirements

- **DW1000 Module**: Any DW1000 breakout board or module
- **SPI Interface**: 3-wire SPI connection
- **GPIO Pins**: Chip select (CS) pin required, reset and IRQ pins optional
- **Power**: 3.3V power supply for DW1000

## Building

### 1. Copy module to MicroPython project

```bash
# In your MicroPython project directory
git submodule add https://github.com/yourusername/micropython-dw1000.git modules/dw1000
```

### 2. Build with Make (e.g., ESP32, STM32)

```bash
cd micropython/ports/esp32
make USER_C_MODULES=../../../../modules/dw1000/micropython.cmake
```

### 3. Build with CMake (e.g., RP2)

```bash
cd micropython/ports/rp2  
make USER_C_MODULES=../../../../modules/dw1000/micropython.cmake
```

## Usage Examples

### Basic Setup

```python
import machine
import dw1000

# Configure SPI and pins
spi = machine.SPI(1, baudrate=1000000, polarity=0, phase=0)
cs_pin = machine.Pin(5, machine.Pin.OUT)
reset_pin = machine.Pin(4, machine.Pin.OUT)  # Optional
irq_pin = machine.Pin(6, machine.Pin.IN)     # Optional

# Create DW1000 instance
dwm = dw1000.DW1000(spi, cs_pin, reset_pin, irq_pin)

# Initialize the device
dwm.init()

# Read device ID to verify communication
device_id = dwm.read_device_id()
print(f"Device ID: 0x{device_id:08X}")

# Configure for basic operation
config = {
    'channel': 2,
    'data_rate': dw1000.BR_6M8
}
dwm.configure(config)
```

### Transmitting Data

```python
# Transmit a simple message
message = b"Hello, UWB World!"
dwm.tx_frame(message)
print("Message transmitted")
```

### Receiving Data

```python
# Enable receiver
dwm.rx_enable()

# Poll for received data
import time
timeout = time.ticks_ms() + 5000  # 5 second timeout

while time.ticks_ms() < timeout:
    status = dwm.get_status()
    if status & 0x4000:  # RX good frame
        data = dwm.read_rx_data()
        print(f"Received: {data}")
        break
    time.sleep_ms(10)
```

### Distance Measurement (Ranging)

```python
# Two-way ranging example (simplified)
def measure_distance():
    # This is a simplified example - real TWR requires
    # careful timing and multiple message exchanges
    
    # Send ranging request
    request = b"RANGE_REQ"
    dwm.tx_frame(request)
    
    # Enable receiver for response
    dwm.rx_enable()
    
    # Wait for response and calculate distance
    # (Implementation depends on ranging protocol)
    pass
```

## API Reference

### DW1000 Class

#### Constructor
```python
DW1000(spi, cs_pin, reset_pin=None, irq_pin=None)
```
- `spi`: SPI object configured for DW1000 communication
- `cs_pin`: Chip select pin object
- `reset_pin`: Optional reset pin object
- `irq_pin`: Optional interrupt pin object

#### Methods

##### `init()`
Initialize the DW1000 device. Must be called before other operations.

##### `deinit()`
Deinitialize the device and release resources.

##### `read_device_id()`
Read the device ID register. Returns `0xDECA0130` for DW1000.

##### `configure(config=None)`
Configure the device with specified parameters.
- `config`: Dictionary with configuration parameters

##### `tx_frame(data)`
Transmit a data frame.
- `data`: Bytes object containing data to transmit

##### `rx_enable()`
Enable the receiver for incoming frames.

##### `read_rx_data()`
Read received frame data. Returns bytes object.

##### `get_status()`
Read the device status register.

### Constants

- `dw1000.DEVICE_ID`: Expected device ID (0xDECA0130)
- `dw1000.SUCCESS`: Success return code (0)
- `dw1000.ERROR`: Error return code (-1)

#### Data Rates
- `dw1000.BR_110K`: 110 kbps
- `dw1000.BR_850K`: 850 kbps  
- `dw1000.BR_6M8`: 6.8 Mbps

#### Channels
- `dw1000.CHANNEL_1` through `dw1000.CHANNEL_7`

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