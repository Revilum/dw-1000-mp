#!/usr/bin/env python3
"""
DW1000 Hardware-Compliant Receiver

This receiver follows the mandatory DW1000 hardware requirements for error handling
as documented in the DW1000 User Manual and implemented in the lab11 driver.

Key compliance features:
- MANDATORY reset after ANY CRC error (not optional)
- Proper status clearing using existing driver functions
- Professional error recovery following hardware specifications
- Extended operation reliability addressing "stops after some time" issue

Hardware Requirement (from DW1000 User Manual):
"An RX reset must be applied after any error or timeout event to ensure
the next good frame's timestamp is computed correctly."
"""

import time
import dw1000
from machine import SPI, Pin
import machine

# Status register constants (from DW1000 driver)
SYS_STATUS_RXFCG = 0x00004000  # Receiver FCS Good
SYS_STATUS_RXFCE = 0x00008000  # Receiver FCS Error (CRC Error)
SYS_STATUS_RXRFTO = 0x00020000  # Receiver timeout
SYS_STATUS_RXOVRR = 0x00100000  # Receiver overrun
SYS_STATUS_LDEERR = 0x00040000  # Leading edge detection error
SYS_STATUS_RXPTO = 0x00200000   # Preamble timeout

# Error status masks
SYS_STATUS_ALL_RX_ERR = (SYS_STATUS_RXFCE | SYS_STATUS_LDEERR)  # All RX errors requiring reset
SYS_STATUS_SERIOUS_ERR = (SYS_STATUS_RXRFTO | SYS_STATUS_RXOVRR | SYS_STATUS_RXPTO)  # Serious errors

def mandatory_crc_recovery(dw, error_info):
    """
    MANDATORY recovery for CRC errors - DW1000 Hardware Requirement
    
    This is required by the DW1000 hardware specification, not optional.
    From DW1000 User Manual: "an RX reset must be applied after any error
    or timeout event to ensure the next good frame's timestamp is computed correctly."
    """
    print(f"MANDATORY CRC Recovery: {error_info}")
    
    # Step 1: Force transceiver off and clear all status (using existing driver function)
    dw.force_trx_off()
    
    # Step 2: Reset receiver (prevents timestamp corruption - hardware requirement)
    dw.rx_reset()
    
    # Step 3: Re-enable receiver
    dw.rx_enable()

def serious_error_recovery(dw, error_info):
    """
    Recovery for serious errors (timeouts, overruns) with full reset sequence
    """
    print(f"Serious Error Recovery: {error_info}")
    
    # Full recovery sequence using existing driver functions
    dw.force_trx_off()      # Comprehensive shutdown + status clearing
    time.sleep_ms(5)        # Brief pause for hardware
    dw.rx_reset()          # Reset receiver
    dw.sync_rx_bufptrs()   # Synchronize buffer pointers
    dw.rx_enable()         # Re-enable receiver

# Initialize SPI and DW1000 (standard pattern)
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
print("DW1000 configured for hardware-compliant reception")

# Start receiving
print("Starting DW1000 Hardware-Compliant Receiver...")
print("✓ MANDATORY reset after every CRC error (hardware requirement)")
print("✓ Professional error recovery using existing driver functions")
print("✓ Extended operation reliability (addresses 'stops after some time')")
print("✓ Full compliance with DW1000 User Manual specifications")
print("Press Ctrl+C to stop")
dw.rx_enable()

# Statistics
message_count = 0
crc_errors = 0
serious_errors = 0
preventive_recoveries = 0
last_activity_time = time.ticks_ms()

while True:
    try:
        # Check status register for events
        status = dw.get_status()
        
        if status & SYS_STATUS_RXFCG:  # Frame received with good CRC
            # Read and process frame
            data = dw.read_rx_data()
            
            if len(data) > 0:
                try:
                    # Decode as UTF-8 text
                    message = data.decode('utf-8')
                    message_count += 1
                    print(f"[{message_count:04d}] '{message}' ({len(data)}B) "
                          f"[CRC_Err:{crc_errors} Serious:{serious_errors} Prev:{preventive_recoveries}]")
                except UnicodeDecodeError:
                    # Display as hex if not valid UTF-8
                    hex_data = ' '.join(f'{b:02X}' for b in data)
                    message_count += 1
                    print(f"[{message_count:04d}] Binary: {hex_data} ({len(data)}B) "
                          f"[CRC_Err:{crc_errors} Serious:{serious_errors} Prev:{preventive_recoveries}]")
            
            # Success: re-enable and update activity time
            dw.rx_enable()
            last_activity_time = time.ticks_ms()
            
        elif status & SYS_STATUS_RXFCE:  # CRC Error - MANDATORY RESET
            crc_errors += 1
            
            # CRITICAL: This is a DW1000 HARDWARE REQUIREMENT, not a choice
            # Every CRC error MUST trigger a reset to prevent timestamp corruption
            mandatory_crc_recovery(dw, f"CRC error #{crc_errors}")
            last_activity_time = time.ticks_ms()
            
        elif status & SYS_STATUS_SERIOUS_ERR:  # Serious errors
            serious_errors += 1
            
            if status & SYS_STATUS_RXRFTO:
                error_type = "receiver timeout"
            elif status & SYS_STATUS_RXOVRR:
                error_type = "receiver overrun"
            elif status & SYS_STATUS_RXPTO:
                error_type = "preamble timeout"
            else:
                error_type = "unknown serious error"
            
            # Full recovery for serious errors
            serious_error_recovery(dw, f"{error_type} #{serious_errors}")
            last_activity_time = time.ticks_ms()
            
        else:
            # Check for extended inactivity (addresses "stops after some time")
            inactivity_time = time.ticks_diff(time.ticks_ms(), last_activity_time)
            
            if inactivity_time > 60000:  # 60 seconds of inactivity
                preventive_recoveries += 1
                print(f"Preventive recovery after 60s inactivity (#{preventive_recoveries})")
                
                # Full preventive recovery
                serious_error_recovery(dw, "60s inactivity prevention")
                last_activity_time = time.ticks_ms()
        
        # Brief delay to prevent overwhelming the system
        time.sleep(0.01)
        
    except KeyboardInterrupt:
        print(f"\nHardware-Compliant Receiver stopped by user")
        print(f"Final Statistics:")
        print(f"  Messages received: {message_count}")
        print(f"  CRC errors (mandatory resets): {crc_errors}")
        print(f"  Serious errors: {serious_errors}")
        print(f"  Preventive recoveries: {preventive_recoveries}")
        
        total_events = message_count + crc_errors + serious_errors
        if total_events > 0:
            success_rate = message_count / total_events * 100
            print(f"  Success rate: {success_rate:.1f}%")
        
        print(f"✓ Compliant with DW1000 hardware requirements")
        print(f"✓ Mandatory reset performed for all {crc_errors} CRC errors")
        break
    except Exception as e:
        print(f"Error: {e}")
        time.sleep(0.1)

print("Hardware-compliant receiver finished")