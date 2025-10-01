#!/usr/bin/env python3
"""
Simple Enhanced DW1000 Receiver

This addre        elif status & SYS_STATUS_RXFCE:  # Frame received with bad CRC
            error_count += 1
            consecutive_crc_errors += 1
            
            # MANDATORY: DW1000 hardware requires reset after ANY CRC error
            # Reference: DW1000 User Manual - "RX Message timestamp" section
            # Driver comment: "an RX reset must be applied after any error to ensure
            # the next good frame's timestamp is computed correctly"
            print(f"RX: CRC error - performing mandatory reset (error #{error_count}, consecutive: {consecutive_crc_errors})")
            
            dw.force_trx_off()  # Clear status and shutdown transceiver
            dw.rx_reset()       # Reset receiver (prevents timestamp corruption)
            dw.rx_enable()      # Re-enable receiver
            
            last_recovery_time = time.ticks_ms()
            # Note: consecutive_crc_errors still tracked for statistics stops after some time" issue with minimal changes
to the working simple_receiver.py, only adding basic error recovery when needed.
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

# Start receiving
print("Starting simple enhanced receiver...")
print("This addresses 'receiver stops after some time' with minimal changes")
dw.rx_enable()

message_count = 0
error_count = 0
consecutive_crc_errors = 0
last_recovery_time = time.ticks_ms()

while True:
    try:
        # Check status register for received frames (same as simple_receiver.py)
        status = dw.get_status()
        
        if status & SYS_STATUS_RXFCG:  # Frame received with good CRC
            # Read the payload data (CRC already validated by hardware)
            data = dw.read_rx_data()
            
            if len(data) > 0:
                try:
                    # Decode as UTF-8 text
                    message = data.decode('utf-8')
                    message_count += 1
                    print(f"[{message_count:04d}] Received '{message}' ({len(data)} bytes) - CRC OK")
                except UnicodeDecodeError:
                    # Display as hex if not valid UTF-8
                    hex_data = ' '.join(f'{b:02X}' for b in data)
                    message_count += 1
                    print(f"[{message_count:04d}] Received binary: {hex_data} ({len(data)} bytes) - CRC OK")
            
            # Re-enable receiver for next frame (same as simple_receiver.py)
            dw.rx_enable()
            last_recovery_time = time.ticks_ms()  # Reset recovery timer on successful receive
            consecutive_crc_errors = 0  # Reset CRC error counter on successful receive
            
        elif status & SYS_STATUS_RXFCE:  # Frame received with bad CRC
            error_count += 1
            consecutive_crc_errors += 1
            
            # Smart CRC error handling
            if consecutive_crc_errors >= 3:
                # Multiple consecutive CRC errors suggest receiver state issues
                print(f"RX: {consecutive_crc_errors} consecutive CRC errors - performing recovery (error #{error_count})")
                dw.force_trx_off()
                time.sleep_ms(10)
                dw.rx_reset()
                dw.rx_enable()
                consecutive_crc_errors = 0  # Reset counter after recovery
                last_recovery_time = time.ticks_ms()
            else:
                # Single CRC error - just re-enable (could be RF interference)
                print(f"RX: Frame CRC failed ({consecutive_crc_errors} consecutive) - re-enabling (error #{error_count})")
                dw.rx_enable()
            
        elif status & (SYS_STATUS_RXRFTO | SYS_STATUS_RXOVRR):  # Serious errors that might cause "stopping"
            error_count += 1
            print(f"RX: Receiver error detected (timeout/overrun) - performing light recovery (error #{error_count})")
            
            # Light recovery - just use the new functions we added
            dw.force_trx_off()  # This should clear status and shutdown transceiver
            time.sleep_ms(10)   # Brief pause
            dw.rx_enable()      # Restart receiver
            last_recovery_time = time.ticks_ms()
            
        else:
            # Check if we haven't received anything for a very long time (actual "stopping" condition)
            time_since_activity = time.ticks_diff(time.ticks_ms(), last_recovery_time)
            if time_since_activity > 60000:  # 60 seconds with no activity
                print("No activity for 60 seconds - performing recovery to prevent 'stopping'")
                dw.force_trx_off()
                time.sleep_ms(10)
                dw.rx_enable()
                last_recovery_time = time.ticks_ms()
        
        # Brief delay to prevent overwhelming the system (same as simple_receiver.py)
        time.sleep(0.01)
        
    except KeyboardInterrupt:
        print(f"\nReceiver stopped by user")
        print(f"Messages received: {message_count}")
        print(f"Errors handled: {error_count}")
        break
    except Exception as e:
        print(f"Error: {e}")
        time.sleep(0.1)

print("Simple enhanced receiver finished")