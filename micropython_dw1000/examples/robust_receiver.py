"""
Robust DW1000 Receiver with Auto-Recovery

This receiver includes automatic error recovery and buffer management
to prevent the receiver from stopping after a few messages.
"""

import machine
import dw1000
import time

def clear_rx_errors(dwm):
    """Clear all RX error flags and reset receiver state"""
    try:
        # Read current status
        status = dwm.get_status()
        
        # Check for various error conditions
        error_flags = {
            0x020000: "RXPHE (PHR Error)",
            0x040000: "RXRFSL (Reed Solomon Error)", 
            0x080000: "RXRFTO (Frame Wait Timeout)",
            0x100000: "LDEERR (Leading Edge Detection Error)",
            0x200000: "RXOVRR (Receiver Overrun)",
            0x400000: "RXPTO (Preamble Detection Timeout)",
            0x800000: "RXSFDTO (SFD Timeout)",
            0x1000000: "HPDWARN (Half Period Delay Warning)",
            0x2000000: "TXBERR (Transmit Buffer Error)"
        }
        
        error_found = False
        for flag, name in error_flags.items():
            if status & flag:
                print(f"  🔧 Clearing {name}")
                error_found = True
        
        if error_found:
            print(f"  📊 Full status before clear: 0x{status:08X}")
            
            # Force receiver reset by disabling and re-enabling
            # This clears internal buffers and state machines
            time.sleep_ms(10)
            dwm.rx_enable()
            
            new_status = dwm.get_status()
            print(f"  📊 Status after reset: 0x{new_status:08X}")
            
        return error_found
        
    except Exception as e:
        print(f"  ❌ Error during RX error clearing: {e}")
        return False

def force_rx_reset(dwm):
    """Force a complete receiver reset"""
    try:
        print("  🔄 Forcing complete RX reset...")
        
        # Multiple reset attempts to ensure clean state
        for i in range(3):
            dwm.rx_enable()
            time.sleep_ms(10)
        
        status = dwm.get_status()
        print(f"  📊 Status after force reset: 0x{status:08X}")
        
        # Verify receiver is in correct state
        if status & 0x4000:  # RXFCG bit shouldn't be set after reset
            print(f"  ⚠️  Warning: RXFCG still set after reset")
        
        return True
        
    except Exception as e:
        print(f"  ❌ Force reset failed: {e}")
        return False

def main():
    print("Robust DW1000 Receiver with Auto-Recovery")
    print("========================================")
    
    spi = machine.SPI(0, baudrate=1000000)
    cs_pin = machine.Pin(17, machine.Pin.OUT)
    reset_pin = machine.Pin(21, machine.Pin.OUT)
    irq_pin = machine.Pin(20, machine.Pin.IN)
    
    try:
        dwm = dw1000.DW1000(spi, cs_pin, reset_pin, irq_pin)
        time.sleep_ms(100)
        
        if not dwm.init():
            print("❌ Init failed")
            return
        
        print("✅ DW1000 initialized")
        
        config = {'channel': 2, 'data_rate': dw1000.BR_850K}
        dwm.configure(config)
        print("✅ Configured for Channel 5, BR_850K")
        
        print("\nWaiting for frames with auto-recovery...")
        print("This receiver will automatically recover from errors")
        
        dwm.rx_enable()
        
        frame_count = 0
        expected_frame = 0
        error_count = 0
        last_frame_time = time.ticks_ms()
        recovery_count = 0
        
        # Recovery parameters
        MAX_FRAME_GAP_MS = 10000  # 10 seconds without frames triggers recovery
        MAX_ERRORS_BEFORE_RECOVERY = 3
        
        while True:
            try:
                current_time = time.ticks_ms()
                status = dwm.get_status()
                
                # Skip obviously corrupted status values
                if status == 0xFFFFFFFF or status > 0x7FFFFFFF:
                    time.sleep_ms(10)
                    continue
                
                # Check for frame timeout (no frames for too long)
                if time.ticks_diff(current_time, last_frame_time) > MAX_FRAME_GAP_MS:
                    print(f"\n⏰ Frame timeout - no frames for {MAX_FRAME_GAP_MS/1000:.1f} seconds")
                    recovery_count += 1
                    print(f"🔧 Performing recovery #{recovery_count}")
                    
                    if force_rx_reset(dwm):
                        error_count = 0
                        last_frame_time = current_time
                        print("✅ Recovery completed")
                    else:
                        print("❌ Recovery failed")
                
                # Check for received frame
                if status & 0x4000:  # RXFCG bit
                    frame_count += 1
                    last_frame_time = current_time
                    error_count = 0  # Reset error count on successful frame
                    
                    print(f"\n{'='*50}")
                    print(f"RECEIVED FRAME {frame_count} (Recovery #{recovery_count})")
                    print(f"{'='*50}")
                    
                    # Read frame data
                    data = dwm.read_rx_data()
                    
                    print(f"Raw bytes: {data.hex()}")
                    print(f"Length: {len(data)}")
                    
                    # Try to decode - separate message from checksum
                    try:
                        if len(data) >= 1:
                            # Split data: last byte is checksum, rest is message
                            message_bytes = data[:-1]  # All but last byte
                            checksum_byte = data[-1]   # Last byte
                            
                            message = message_bytes.decode('utf-8')
                            print(f"Decoded: '{message}'")
                            print(f"Checksum: 0x{checksum_byte:02x} ('{chr(checksum_byte)}' if printable)")
                            
                            # Verify checksum
                            calculated_checksum = sum(message_bytes) & 0xFF
                            if calculated_checksum == checksum_byte:
                                print(f"✅ Checksum valid")
                            else:
                                print(f"❌ Checksum invalid - got 0x{checksum_byte:02x}, expected 0x{calculated_checksum:02x}")
                        else:
                            message = data.decode('utf-8')
                            print(f"Decoded: '{message}'")
                        
                        # Check frame sequence
                        if message.startswith("Frame_") and message.endswith("!"):
                            frame_num_str = message[6:8]
                            try:
                                received_frame_num = int(frame_num_str)
                                
                                if received_frame_num == expected_frame:
                                    print(f"✅ CORRECT - Frame {expected_frame} received in order")
                                    expected_frame += 1
                                elif received_frame_num > expected_frame:
                                    missed_count = received_frame_num - expected_frame
                                    print(f"⚠️  MISSED {missed_count} FRAMES:")
                                    for missed in range(expected_frame, received_frame_num):
                                        print(f"   - Frame_{missed:02d}!")
                                    expected_frame = received_frame_num + 1
                                else:
                                    print(f"❌ OUT OF ORDER - Got frame {received_frame_num}, expected {expected_frame}")
                                
                            except ValueError:
                                print(f"❌ Could not parse frame number from '{frame_num_str}'")
                        else:
                            print(f"📦 Non-sequential frame: '{message}'")
                    
                    except:
                        print(f"❌ Could not decode as UTF-8")
                        print(f"Raw hex: {data.hex()}")
                    
                    # Clear any remaining flags and re-enable receiver
                    clear_rx_errors(dwm)
                    dwm.rx_enable()
                
                # Check for RX errors
                elif status & 0xE0000:  # RX error bits
                    error_count += 1
                    print(f"\n⚠️  RX Error #{error_count}: 0x{status:08X}")
                    
                    # Decode specific error types
                    if status & 0x020000:
                        print("  - PHR Error (header corruption)")
                    if status & 0x040000:
                        print("  - Reed Solomon Error (FEC failure)")
                    if status & 0x080000:
                        print("  - Frame Wait Timeout")
                    if status & 0x400000:
                        print("  - Preamble Detection Timeout")
                    if status & 0x800000:
                        print("  - SFD Timeout")
                    
                    # Clear errors and try recovery
                    if clear_rx_errors(dwm):
                        print("  ✅ Errors cleared")
                    
                    # If too many errors, force full recovery
                    if error_count >= MAX_ERRORS_BEFORE_RECOVERY:
                        recovery_count += 1
                        print(f"🔧 Too many errors - performing recovery #{recovery_count}")
                        
                        if force_rx_reset(dwm):
                            error_count = 0
                            print("✅ Error recovery completed")
                        else:
                            print("❌ Error recovery failed")
                
                # Small delay to prevent busy waiting
                time.sleep_ms(10)
                
            except Exception as e:
                print(f"❌ Main loop error: {e}")
                error_count += 1
                
                # Try to recover from exception
                try:
                    if force_rx_reset(dwm):
                        print("✅ Recovered from exception")
                    else:
                        print("❌ Could not recover from exception")
                except:
                    print("❌ Recovery attempt failed")
                
                time.sleep_ms(100)
    
    except KeyboardInterrupt:
        print(f"\n⏹️ Receiver stopped by user")
        print(f"📊 Session Statistics:")
        print(f"   Total frames received: {frame_count}")
        print(f"   Expected next frame: {expected_frame}")
        print(f"   Recovery attempts: {recovery_count}")
        if frame_count > 0:
            success_rate = ((frame_count / expected_frame) * 100) if expected_frame > 0 else 100
            print(f"   Success rate: {success_rate:.1f}%")
    
    except Exception as e:
        print(f"❌ Critical error: {e}")
    
    finally:
        try:
            dwm.deinit()
            print("🔄 DW1000 deinitialized")
        except:
            pass

if __name__ == "__main__":
    main()