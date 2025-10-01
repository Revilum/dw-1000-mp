#!/usr/bin/env python3
"""
Simple DW1000 Transmitter Example

This example demonstrates basic frame transmission using the DW1000's 
built-in CRC validation. No application-level checksum is needed.
"""

import time
import dw1000
from machine import SPI, Pin

# Initialize SPI and DW1000
spi = SPI(0, baudrate=1000000)
cs_pin = machine.Pin(17, machine.Pin.OUT)
reset_pin = machine.Pin(21, machine.Pin.OUT)
irq_pin = machine.Pin(20, machine.Pin.IN)
dw = dw1000.DW1000(spi, cs_pin, reset_pin, irq_pin)

# Initialize and configure
dw.init()
device_id = dw.read_device_id()
print(f"DW1000 Device ID: 0x{device_id:08X}")

dw.configure()
print("DW1000 configured for transmission")

# Transmit frames
counter = 0
while True:
    try:
        # Create simple message (no application checksum needed - DW1000 handles CRC)
        message = f"Test {counter:03d}".encode('utf-8')
        
        print(f"TX: Sending '{message.decode()}' ({len(message)} bytes)")
        
        # Transmit frame - DW1000 automatically adds 2-byte CRC
        dw.tx_frame(message)
        
        counter += 1
        time.sleep(1)
        
    except KeyboardInterrupt:
        print("\nTransmission stopped")
        break
    except Exception as e:
        print(f"Error: {e}")
        time.sleep(1)

print("Simple transmitter finished")