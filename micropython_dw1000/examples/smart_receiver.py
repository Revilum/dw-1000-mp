#!/usr/bin/env python3
"""
Smart DW1000 Receiver with Intelligent Error Recovery

This receiver implements smart error handling strategies:
- Single CRC errors: Just re-enable (could be RF interference)
- Multiple consecutive CRC errors: Perform receiver reset
- Serious errors (timeout/overrun): Full recovery sequence
- Extended inactivity: Preventive recovery

This addresses both the "receiver stops after some time" issue and
provides robust handling of corrupt packets and error patterns.
"""

import time
import dw1000
from machine import SPI, Pin
import machine

# Status register constants (from DW1000 driver)
SYS_STATUS_RXFCG = 0x00004000  # Receiver FCS Good
SYS_STATUS_RXFCE = 0x00008000  # Receiver FCS Error
SYS_STATUS_RXRFTO = 0x00020000  # Receiver timeout
SYS_STATUS_RXOVRR = 0x00100000  # Receiver overrun

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
print("DW1000 configured for reception")

def light_recovery(dw, reason):
    """Light recovery: force off + rx reset"""
    print(f"Performing light recovery: {reason}")
    dw.force_trx_off()
    time.sleep_ms(5)
    dw.rx_reset()
    dw.rx_enable()

def full_recovery(dw, reason):
    """Full recovery: force off + rx reset + buffer sync"""
    print(f"Performing full recovery: {reason}")
    dw.force_trx_off()
    time.sleep_ms(10)
    dw.rx_reset()
    dw.sync_rx_bufptrs()
    dw.rx_enable()

# Start receiving
print("Starting smart receiver with intelligent error recovery...")
print("- Single CRC errors: re-enable only")
print("- 3+ consecutive CRC errors: light recovery") 
print("- Timeout/overrun errors: full recovery")
print("- 60s inactivity: preventive recovery")
dw.rx_enable()

message_count = 0
total_errors = 0
consecutive_crc_errors = 0
light_recoveries = 0
full_recoveries = 0
last_activity_time = time.ticks_ms()

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
                    message_count += 1
                    print(f"[{message_count:04d}] '{message}' ({len(data)}B) [E:{total_errors} L:{light_recoveries} F:{full_recoveries}]")
                except UnicodeDecodeError:
                    # Display as hex if not valid UTF-8
                    hex_data = ' '.join(f'{b:02X}' for b in data)
                    message_count += 1
                    print(f"[{message_count:04d}] Binary: {hex_data} ({len(data)}B) [E:{total_errors} L:{light_recoveries} F:{full_recoveries}]")
            
            # Success: reset error counters and update activity time
            dw.rx_enable()
            consecutive_crc_errors = 0
            last_activity_time = time.ticks_ms()
            
        elif status & SYS_STATUS_RXFCE:  # Frame received with bad CRC
            total_errors += 1
            consecutive_crc_errors += 1
            
            # MANDATORY: DW1000 hardware requires reset after ANY CRC error
            # This is not optional - it's a hardware requirement per DW1000 User Manual
            print(f"CRC error #{consecutive_crc_errors} (total: {total_errors}) - mandatory reset")
            
            dw.force_trx_off()  # Clear status and shutdown transceiver  
            dw.rx_reset()       # Reset receiver (prevents timestamp corruption)
            dw.rx_enable()      # Re-enable receiver
            
            light_recoveries += 1
            consecutive_crc_errors = 0  # Reset after mandatory recovery
            last_activity_time = time.ticks_ms()
            
        elif status & (SYS_STATUS_RXRFTO | SYS_STATUS_RXOVRR):  # Serious receiver errors
            total_errors += 1
            
            if status & SYS_STATUS_RXRFTO:
                error_type = "timeout"
            elif status & SYS_STATUS_RXOVRR:
                error_type = "overrun"
            else:
                error_type = "unknown"
            
            # Serious errors need full recovery
            full_recovery(dw, f"receiver {error_type}")
            full_recoveries += 1
            consecutive_crc_errors = 0
            last_activity_time = time.ticks_ms()
            
        else:
            # No frame received - check for extended inactivity
            inactivity_time = time.ticks_diff(time.ticks_ms(), last_activity_time)
            
            if inactivity_time > 60000:  # 60 seconds of inactivity
                full_recovery(dw, "60s inactivity (preventing 'stopping')")
                full_recoveries += 1
                last_activity_time = time.ticks_ms()
        
        # Brief delay to prevent overwhelming the system
        time.sleep(0.01)
        
    except KeyboardInterrupt:
        print(f"\nSmart receiver stopped by user")
        print(f"Statistics:")
        print(f"  Messages received: {message_count}")
        print(f"  Total errors: {total_errors}")
        print(f"  Light recoveries: {light_recoveries}")
        print(f"  Full recoveries: {full_recoveries}")
        if message_count + total_errors > 0:
            success_rate = message_count / (message_count + total_errors) * 100
            print(f"  Success rate: {success_rate:.1f}%")
        break
    except Exception as e:
        print(f"Error: {e}")
        time.sleep(0.1)

print("Smart receiver finished")