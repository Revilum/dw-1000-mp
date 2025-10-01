#!/usr/bin/env python3
"""
Comprehensive DW1000 Diagnostic Receiver

This receiver implements extensive diagnostics to identify why the receiver 
stops receiving messages. It monitors:

1. System state register (SYS_STATE) - shows RX state machine status
2. Status register (SYS_STATUS) - shows error conditions
3. RX frame info (RX_FINFO) - shows buffer status
4. Receiver state transitions
5. Periodic health checks to detect "stuck" states

Based on the DW1000 User Manual and Software API Guide analysis.
"""

import time
import dw1000
from machine import SPI, Pin
import machine

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
print("DW1000 configured for reception with comprehensive diagnostics")

def decode_rx_state(sys_state):
    """Decode RX state from SYS_STATE register"""
    rx_state = sys_state & dw1000.SYS_STATE_RX_STATE
    states = {
        dw1000.RX_STATE_IDLE: "IDLE",
        dw1000.RX_STATE_RX_READY: "RX_READY", 
        dw1000.RX_STATE_RX_WAIT_SFD: "WAIT_SFD",
        dw1000.RX_STATE_RX_DATA: "RX_DATA",
        dw1000.RX_STATE_RX_VALIDATE: "VALIDATE"
    }
    return states.get(rx_state, f"UNKNOWN(0x{rx_state:02X})")

def decode_status_errors(status):
    """Decode error flags from SYS_STATUS register using lab11 driver constants"""
    errors = []
    if status & dw1000.SYS_STATUS_RXFCE:
        errors.append("CRC_ERR")
    if status & dw1000.SYS_STATUS_RXPHE:
        errors.append("PHY_ERR")
    if status & dw1000.SYS_STATUS_RXRFSL:
        errors.append("RS_ERR")
    if status & dw1000.SYS_STATUS_RXRFTO:
        errors.append("TIMEOUT")
    if status & dw1000.SYS_STATUS_RXOVRR:
        errors.append("OVERRUN")
    if status & dw1000.SYS_STATUS_RXSFDTO:
        errors.append("SFD_TO")
    if status & dw1000.SYS_STATUS_LDEERR:
        errors.append("LDE_ERR")
    if status & dw1000.SYS_STATUS_AFFREJ:
        errors.append("FILT_REJ")
    return errors if errors else ["NONE"]

def comprehensive_diagnosis(dw):
    """Perform comprehensive DW1000 state diagnosis"""
    print("\n=== DW1000 COMPREHENSIVE DIAGNOSIS ===")
    
    # Read key registers
    sys_status = dw.get_status()
    sys_state = dw.get_sys_state() 
    rx_finfo = dw.get_rx_finfo()
    
    print(f"SYS_STATUS: 0x{sys_status:08X}")
    print(f"SYS_STATE:  0x{sys_state:08X}")
    print(f"RX_FINFO:   0x{rx_finfo:08X}")
    
    # Read critical control registers
    try:
        sys_ctrl = dw.read_register(0x0D)  # SYS_CTRL_ID
        print(f"SYS_CTRL:   0x{sys_ctrl:08X}")
    except:
        print("SYS_CTRL:   READ FAILED")
    
    # Decode RX state
    rx_state_name = decode_rx_state(sys_state)
    print(f"RX State: {rx_state_name}")
    
    # Check for frame reception
    if sys_status & dw1000.SYS_STATUS_RXFCG:
        print("Status: FRAME AVAILABLE (good CRC)")
    elif sys_status & dw1000.SYS_STATUS_RXFCE:
        print("Status: FRAME AVAILABLE (bad CRC)")
    else:
        print("Status: NO FRAME")
    
    # Decode errors
    errors = decode_status_errors(sys_status)
    print(f"Errors: {', '.join(errors)}")
    
    # Check frame info
    frame_len = rx_finfo & 0x3FF  # RX_FINFO_RXFLEN_MASK
    if frame_len > 0:
        print(f"Frame Length: {frame_len} bytes")
    
    # Analyze potential issues
    issues = []
    
    # Check if receiver is stuck in wrong state
    if rx_state_name == "IDLE":
        issues.append("RECEIVER NOT ENABLED - needs rx_enable()")
    elif rx_state_name == "UNKNOWN":
        issues.append("RECEIVER IN INVALID STATE - needs reset")
    elif rx_state_name == "WAIT_SFD":
        # This is normal listening state
        pass
    elif rx_state_name in ["RX_DATA", "VALIDATE"]:
        if not (sys_status & (dw1000.SYS_STATUS_RXFCG | dw1000.SYS_STATUS_RXFCE)):
            issues.append("RECEIVER STUCK IN RX - data not read")
    
    # Check for buffer issues
    if sys_status & dw1000.SYS_STATUS_RXOVRR:
        issues.append("BUFFER OVERRUN - host too slow reading data")
    
    # Check for timeout issues
    if sys_status & dw1000.SYS_STATUS_RXRFTO:
        issues.append("RECEIVE TIMEOUT - no SFD detected")
    
    # Check SYS_CTRL register for receiver enable bit
    try:
        sys_ctrl = dw.read_register(0x0D)  # SYS_CTRL_ID
        if (sys_ctrl & 0x0100) == 0:  # SYS_CTRL_RXENAB bit
            issues.append("SYS_CTRL RXENAB BIT CLEAR - rx_enable() failed")
    except:
        issues.append("CANNOT READ SYS_CTRL REGISTER")
    
    if issues:
        print(f"ISSUES DETECTED: {', '.join(issues)}")
    else:
        print("STATE: Normal operation")
    
    print("=" * 40)
    return issues

def smart_recovery(dw, issues, reason=""):
    """Perform targeted recovery based on diagnosed issues"""
    print(f"\nSMART RECOVERY: {reason}")
    
    if "RECEIVER NOT ENABLED" in issues:
        print("- Full initialization sequence")
        # Perform complete reset sequence
        dw.force_trx_off()
        time.sleep_ms(10)
        dw.rx_reset() 
        dw.sync_rx_bufptrs()
        
        # Re-configure the device (might have lost configuration)
        try:
            dw.configure()
            print("- Re-configured DW1000")
        except Exception as e:
            print(f"- Configure failed: {e}")
        
        # Try to enable receiver
        try:
            result = dw.rx_enable()
            if result:
                print("- RX enable successful")
                # Double-check by reading SYS_CTRL
                sys_ctrl = dw.read_register(0x0D)
                if sys_ctrl & 0x0100:
                    print("- SYS_CTRL RXENAB bit confirmed SET")
                else:
                    print("- WARNING: RX enable returned True but RXENAB bit still clear!")
            else:
                print("- RX enable returned False")
        except Exception as e:
            print(f"- RX enable EXCEPTION: {e}")
            
    elif "RECEIVER IN INVALID STATE" in issues or "RECEIVER STUCK IN RX" in issues:
        print("- Full receiver reset")
        dw.force_trx_off()
        time.sleep_ms(10)
        dw.rx_reset()
        dw.sync_rx_bufptrs()
        dw.rx_enable()
    elif "BUFFER OVERRUN" in issues:
        print("- Buffer reset and clear")
        dw.force_trx_off()
        dw.sync_rx_bufptrs()
        dw.rx_enable()
    else:
        print("- Standard recovery")
        dw.force_trx_off()
        time.sleep_ms(5)
        dw.rx_reset()
        dw.rx_enable()

# Start receiving with comprehensive monitoring
print("Starting diagnostic receiver with state monitoring...")
print("This will detect and diagnose receiver state issues")
dw.rx_enable()

message_count = 0
total_errors = 0
recoveries = 0
last_activity_time = time.ticks_ms()
last_diagnostic_time = time.ticks_ms()
previous_rx_state = None

# Perform initial diagnosis
comprehensive_diagnosis(dw)

while True:
    try:
        current_time = time.ticks_ms()
        
        # Periodic detailed diagnostics (every 10 seconds)
        if time.ticks_diff(current_time, last_diagnostic_time) > 10000:
            print(f"\n[{time.ticks_ms()//1000}s] PERIODIC DIAGNOSTIC CHECK")
            issues = comprehensive_diagnosis(dw)
            
            if issues:
                smart_recovery(dw, issues, "periodic diagnostic detected issues")
                recoveries += 1
            
            last_diagnostic_time = current_time
        
        # Check status register for received frames
        status = dw.get_status()
        sys_state = dw.get_sys_state()
        rx_state = decode_rx_state(sys_state)
        
        # Monitor state changes
        if rx_state != previous_rx_state:
            print(f"RX State changed: {previous_rx_state} -> {rx_state}")
            previous_rx_state = rx_state
        
        if status & dw1000.SYS_STATUS_RXFCG:  # Frame received with good CRC
            # Read the payload data
            data = dw.read_rx_data()
            
            if len(data) > 0:
                try:
                    message = data.decode('utf-8')
                    message_count += 1
                    print(f"[{message_count:04d}] '{message}' ({len(data)}B) RX:{rx_state} [E:{total_errors} R:{recoveries}]")
                except UnicodeDecodeError:
                    hex_data = ' '.join(f'{b:02X}' for b in data)
                    message_count += 1
                    print(f"[{message_count:04d}] Binary: {hex_data} ({len(data)}B) RX:{rx_state} [E:{total_errors} R:{recoveries}]")
            
            # Success: re-enable receiver and update activity time
            dw.rx_enable()
            last_activity_time = current_time
            
        elif status & (dw1000.SYS_STATUS_ALL_RX_ERR):  # Use lab11 driver's comprehensive error mask
            
            total_errors += 1
            errors = decode_status_errors(status)
            print(f"ERROR: {', '.join(errors)} in state {rx_state} (total: {total_errors})")
            
            # Mandatory reset for any error per DW1000 hardware requirement
            dw.force_trx_off()
            dw.rx_reset()
            dw.rx_enable()
            
            recoveries += 1
            last_activity_time = current_time
            
        else:
            # No frame received - check for extended inactivity and state issues
            inactivity_time = time.ticks_diff(current_time, last_activity_time)
            
            # Check if receiver is in wrong state
            if rx_state == "IDLE":
                print(f"PROBLEM: Receiver in IDLE state - should be listening!")
                issues = comprehensive_diagnosis(dw)
                smart_recovery(dw, issues, "receiver idle when should be listening")
                recoveries += 1
                last_activity_time = current_time
                
            elif inactivity_time > 30000:  # 30 seconds of inactivity
                print(f"TIMEOUT: {inactivity_time//1000}s inactivity in state {rx_state}")
                issues = comprehensive_diagnosis(dw)
                smart_recovery(dw, issues, f"{inactivity_time//1000}s inactivity")
                recoveries += 1
                last_activity_time = current_time
        
        # Brief delay to prevent overwhelming the system
        time.sleep(0.01)
        
    except KeyboardInterrupt:
        print(f"\nDiagnostic receiver stopped by user")
        print(f"Final diagnosis:")
        comprehensive_diagnosis(dw)
        print(f"\nStatistics:")
        print(f"  Messages received: {message_count}")
        print(f"  Total errors: {total_errors}")
        print(f"  Recoveries performed: {recoveries}")
        if message_count + total_errors > 0:
            success_rate = message_count / (message_count + total_errors) * 100
            print(f"  Success rate: {success_rate:.1f}%")
        break
    except Exception as e:
        print(f"Error: {e}")
        time.sleep(0.1)

print("Diagnostic receiver finished")