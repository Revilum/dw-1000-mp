#!/usr/bin/env python3
"""
Test Recovery Functions

This script tests if the recovery functions are properly available
and working in the MicroPython DW1000 module.
"""

import time
import dw1000
from machine import SPI, Pin
import machine

print("=== Testing DW1000 Recovery Functions ===")

# Initialize SPI and DW1000 (same as simple_receiver.py)
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
print("DW1000 configured successfully")

# Test if recovery functions exist and are callable
print("\nTesting recovery function availability:")

try:
    print("Testing force_trx_off()...")
    dw.force_trx_off()
    print("✓ force_trx_off() - SUCCESS")
except AttributeError as e:
    print(f"✗ force_trx_off() - NOT AVAILABLE: {e}")
except Exception as e:
    print(f"✗ force_trx_off() - ERROR: {e}")

try:
    print("Testing rx_reset()...")
    dw.rx_reset()
    print("✓ rx_reset() - SUCCESS")
except AttributeError as e:
    print(f"✗ rx_reset() - NOT AVAILABLE: {e}")
except Exception as e:
    print(f"✗ rx_reset() - ERROR: {e}")

try:
    print("Testing sync_rx_bufptrs()...")
    dw.sync_rx_bufptrs()
    print("✓ sync_rx_bufptrs() - SUCCESS")
except AttributeError as e:
    print(f"✗ sync_rx_bufptrs() - NOT AVAILABLE: {e}")
except Exception as e:
    print(f"✗ sync_rx_bufptrs() - ERROR: {e}")

# Test recovery sequence
print("\nTesting recovery sequence:")
try:
    print("1. force_trx_off()...")
    dw.force_trx_off()
    
    print("2. rx_reset()...")
    dw.rx_reset()
    
    print("3. sync_rx_bufptrs()...")
    dw.sync_rx_bufptrs()
    
    print("4. rx_enable()...")
    dw.rx_enable()
    
    print("✓ Full recovery sequence - SUCCESS")
    
except Exception as e:
    print(f"✗ Recovery sequence failed: {e}")

print("\nRecovery function test complete")