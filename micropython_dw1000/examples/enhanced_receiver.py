#!/usr/bin/env python3
"""
Enhanced DW1000 Receiver with Professional Error Recovery

This example demonstrates solving the "receiver stops after some time" issue
by leveraging the existing lab11 driver's comprehensive receiver management
functions instead of implementing custom solutions.

Key enhancements:
- Uses dwt_forcetrxoff() for comprehensive transceiver shutdown + status clearing
- Uses dwt_rxreset() for receiver reset on errors  
- Uses dwt_syncrxbufptrs() for buffer synchronization
- Automatic recovery from receiver timeouts and errors
- Professional error handling using existing driver capabilities
"""

import machine
import time
import dw1000

# Configuration constants
SPI_BAUDRATE = 1000000
CS_PIN = 17
IRQ_PIN = 20
RST_PIN = 21

# Status constants (from driver)
SYS_STATUS_RXFCG = 0x00004000  # Frame received with good CRC
SYS_STATUS_RXFCE = 0x00008000  # Frame received with bad CRC
SYS_STATUS_RXRFTO = 0x00020000  # Receiver timeout
SYS_STATUS_LDEERR = 0x00040000  # Leading edge detection error
SYS_STATUS_RXOVRR = 0x00100000  # Receiver overrun
SYS_STATUS_RXPTO = 0x00200000  # Preamble timeout

# Error status mask (any of these indicates receiver issues)
RX_ERROR_MASK = (SYS_STATUS_RXFCE | SYS_STATUS_RXRFTO | 
                 SYS_STATUS_LDEERR | SYS_STATUS_RXOVRR | SYS_STATUS_RXPTO)

def setup_dw1000():
    """Initialize DW1000 with proper configuration"""
    print("Setting up DW1000...")
    
    # Initialize SPI and DW1000 (using same pattern as simple_receiver.py)
    spi = machine.SPI(0, baudrate=SPI_BAUDRATE)
    cs_pin = machine.Pin(CS_PIN, machine.Pin.OUT)
    reset_pin = machine.Pin(RST_PIN, machine.Pin.OUT)
    irq_pin = machine.Pin(IRQ_PIN, machine.Pin.IN)
    
    dw = dw1000.DW1000(spi, cs_pin, reset_pin, irq_pin)
    dw.init()
    
    # Read device ID to verify communication
    device_id = dw.read_device_id()
    print(f"Device ID: 0x{device_id:08X}")
    
    if device_id != 0xDECA0130:
        raise ValueError(f"Invalid device ID: 0x{device_id:08X}")
    
    # Configure for basic operation
    dw.configure()
    print("DW1000 configured successfully")
    
    return dw

def recover_receiver(dw):
    """
    Professional receiver recovery using existing lab11 driver functions
    
    This leverages the driver's comprehensive receiver management instead
    of implementing custom recovery solutions.
    """
    print("Performing receiver recovery using existing driver functions...")
    
    # Step 1: Use existing driver's comprehensive transceiver shutdown
    # This automatically clears all status flags and properly shuts down
    dw.force_trx_off()
    
    # Step 2: Reset receiver using existing driver function
    dw.rx_reset()
    
    # Step 3: Synchronize receive buffer pointers using existing driver function
    dw.sync_rx_bufptrs()
    
    print("Recovery complete - receiver refreshed")

def enhanced_receive_loop(dw):
    """
    Enhanced receiver loop that addresses the "stops after some time" issue
    
    Uses existing lab11 driver functions for reliable long-term operation
    """
    message_count = 0
    error_count = 0
    recovery_count = 0
    last_recovery_time = time.ticks_ms()
    
    print("Starting enhanced receiver loop...")
    print("This addresses the 'receiver stops after some time' issue")
    print("Press Ctrl+C to stop")
    
    try:
        while True:
            # Enable receiver
            dw.rx_enable()
            
            # Wait for frame with timeout
            start_time = time.ticks_ms()
            timeout_ms = 500  # 500ms timeout per attempt
            frame_received = False
            
            while not frame_received:
                status = dw.get_status()
                
                # Check for successful frame reception
                if status & SYS_STATUS_RXFCG:
                    try:
                        # Read received data
                        rx_data = dw.read_rx_data()
                        
                        # Parse message (excluding 2-byte CRC)
                        if len(rx_data) >= 2:
                            try:
                                message = rx_data[:-2].decode('utf-8')
                            except UnicodeDecodeError:
                                # Display as hex if not valid UTF-8
                                hex_data = ' '.join(f'{b:02X}' for b in rx_data[:-2])
                                message = f"Binary: {hex_data}"
                            
                            message_count += 1
                            
                            print(f"[{message_count:04d}] Received: '{message}' "
                                  f"(errors: {error_count}, recoveries: {recovery_count})")
                        
                        frame_received = True
                        
                    except Exception as e:
                        print(f"Error processing frame: {e}")
                        error_count += 1
                        frame_received = True  # Exit loop but handle error
                
                # Check for receiver errors that need recovery
                elif status & RX_ERROR_MASK:
                    error_count += 1
                    
                    if status & SYS_STATUS_RXFCE:
                        print(f"Frame CRC error (total errors: {error_count})")
                    elif status & SYS_STATUS_RXRFTO:
                        print(f"Receiver timeout (total errors: {error_count})")
                    elif status & SYS_STATUS_LDEERR:
                        print(f"Leading edge detection error (total errors: {error_count})")
                    elif status & SYS_STATUS_RXOVRR:
                        print(f"Receiver overrun - performing recovery (total errors: {error_count})")
                        recover_receiver(dw)
                        recovery_count += 1
                        last_recovery_time = time.ticks_ms()
                    elif status & SYS_STATUS_RXPTO:
                        print(f"Preamble timeout (total errors: {error_count})")
                    
                    frame_received = True  # Exit loop and restart
                
                # Check for timeout or periodic maintenance
                elapsed = time.ticks_diff(time.ticks_ms(), start_time)
                if elapsed > timeout_ms:
                    # Periodic maintenance to prevent receiver from "stopping"
                    time_since_recovery = time.ticks_diff(time.ticks_ms(), last_recovery_time)
                    
                    # Perform preventive recovery every 30 seconds or after many errors
                    if time_since_recovery > 30000 or error_count % 20 == 19:
                        print("Performing preventive receiver maintenance...")
                        recover_receiver(dw)
                        recovery_count += 1
                        last_recovery_time = time.ticks_ms()
                    
                    frame_received = True  # Exit loop and restart
                
                # Small delay to prevent busy waiting
                time.sleep_ms(5)
            
            # Brief pause between receive attempts
            time.sleep_ms(10)
            
    except KeyboardInterrupt:
        print(f"\nReceiver stopped by user")
        print(f"Final statistics:")
        print(f"  Messages received: {message_count}")
        print(f"  Errors encountered: {error_count}")
        print(f"  Recoveries performed: {recovery_count}")
        
        if message_count + error_count > 0:
            success_rate = message_count / (message_count + error_count) * 100
            print(f"  Success rate: {success_rate:.1f}%")
        
        print("Enhanced receiver addresses the 'stops after some time' issue!")

def main():
    """Main application entry point"""
    print("=== Enhanced DW1000 Receiver ===")
    print("Solves 'receiver stops after some time' using existing driver functions")
    print()
    
    try:
        # Setup DW1000
        dw = setup_dw1000()
        
        # Start enhanced receiving with professional error recovery
        enhanced_receive_loop(dw)
        
    except Exception as e:
        print(f"Error: {e}")
        return False
    
    return True

if __name__ == "__main__":
    main()