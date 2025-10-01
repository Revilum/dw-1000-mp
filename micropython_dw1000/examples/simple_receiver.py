#!/usr/bin/env python3
"""
Simple DW1000 Receiver Example

This example demonstrates basic frame reception using the DW1000's 
built-in CRC validation. The hardware automatically validates frame integrity.
"""

import time
import dw1000
from machine import SPI, Pin

# Status register constants (from DW1000 driver)
SYS_STATUS_RXFCG = 0x00004000  # Receiver FCS Good
SYS_STATUS_RXFCE = 0x00008000  # Receiver FCS Error
SYS_STATUS_RXRFSD = 0x00000100  # Receiver Reed Solomon Frame Sync Detect

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
print("DW1000 configured for reception")

# Start receiving
print("Starting receiver...")
dw.rx_enable()

while True:
    try:
        # Check status register for received frames
        status = dw.get_status()
        
        if status & SYS_STATUS_RXFCG:  # Frame received with good CRC
            # Read the payload data (CRC already validated by hardware)
            data = dw.read_rx_data()
            
            if len(data) > 0:
                try:
                    # Decode as UTF-8 text
                    message = data.decode('utf-8')
                    print(f"RX: Received '{message}' ({len(data)} bytes) - CRC OK")
                except UnicodeDecodeError:
                    # Display as hex if not valid UTF-8
                    hex_data = ' '.join(f'{b:02X}' for b in data)
                    print(f"RX: Received binary data: {hex_data} ({len(data)} bytes) - CRC OK")
            
            # Re-enable receiver for next frame
            dw.rx_enable()
            
        elif status & SYS_STATUS_RXFCE:  # Frame received with bad CRC
            print("RX: Frame received but CRC failed - discarded")
            dw.rx_enable()
            
        # Brief delay to prevent overwhelming the system
        time.sleep(0.01)
        
    except KeyboardInterrupt:
        print("\nReceiver stopped")
        break
    except Exception as e:
        print(f"Error: {e}")
        time.sleep(0.1)

print("Simple receiver finished")