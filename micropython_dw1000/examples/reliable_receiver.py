"""
DW1000 Reliable Receiver Example

Based on minimal test - uses stable configuration and proper error handling
"""

import machine
import dw1000
import time

def main():
    print("DW1000 Reliable Receiver")
    print("========================")
    
    # Use same stable settings as minimal test
    spi = machine.SPI(0, baudrate=1000000)  # 1MHz for stability
    cs_pin = machine.Pin(17, machine.Pin.OUT)
    reset_pin = machine.Pin(21, machine.Pin.OUT)
    irq_pin = machine.Pin(20, machine.Pin.IN)
    
    try:
        # Initialize with extra stabilization time
        print("Initializing DW1000...")
        dwm = dw1000.DW1000(spi, cs_pin, reset_pin, irq_pin)
        
        time.sleep_ms(100)  # Extra time for stability
        
        if not dwm.init():
            print("❌ Failed to initialize DW1000")
            return
        
        print("✅ DW1000 initialized successfully")
        
        # Configure device
        print("Configuring DW1000...")
        dwm.configure({
            'channel': 5,
            'data_rate': dw1000.BR_850K
        })
        print("✅ Configuration complete")
        
        print("\nStarting receiver...")
        print("Waiting for frames... (Press Ctrl+C to stop)")
        
        # Enable receiver
        dwm.rx_enable()
        
        frame_count = 0
        error_count = 0
        last_status_check = time.ticks_ms()
        
        while True:
            try:
                # Periodic status validation (every 5 seconds)
                now = time.ticks_ms()
                if time.ticks_diff(now, last_status_check) > 5000:
                    status = dwm.get_status()
                    if status == 0xFFFFFFFF or status > 0x7FFFFFFF:
                        print(f"⚠️  Detected corrupted status: 0x{status:08X} - resetting")
                        dwm.rx_enable()
                        error_count += 1
                        if error_count > 5:
                            print("❌ Too many errors - stopping")
                            break
                    last_status_check = now
                
                # Check for received frames
                status = dwm.get_status()
                
                # Skip obviously corrupted status values
                if status == 0xFFFFFFFF or status > 0x7FFFFFFF:
                    time.sleep_ms(10)
                    continue
                
                # Check if we received a good frame (RXFCG bit)
                if status & 0x4000:
                    try:
                        frame_count += 1
                        print(f"\n--- Frame {frame_count} ---")
                        
                        # Read the received data
                        data = dwm.read_rx_data()
                        
                        print(f"Length: {len(data)} bytes")
                        
                        # Only process reasonably sized frames
                        if len(data) > 100:
                            print("⚠️  Frame too large - possible corruption")
                        elif len(data) == 0:
                            print("⚠️  Empty frame received")
                        else:
                            # Check if frame has checksum (last byte)
                            if len(data) > 1:
                                data_part = data[:-1]
                                received_checksum = data[-1]
                                calculated_checksum = sum(data_part) & 0xFF
                                
                                checksum_ok = (received_checksum == calculated_checksum)
                                
                                print(f"Checksum: {'✅ OK' if checksum_ok else '❌ FAIL'} (rx:{received_checksum:02x} calc:{calculated_checksum:02x})")
                                
                                if checksum_ok:
                                    # Checksum is good, try to decode
                                    try:
                                        message = data_part.decode('utf-8')
                                        print(f"✅ Received: '{message}' (verified)")
                                        continue  # Skip the other decode attempts
                                    except:
                                        print(f"📦 Binary data (verified): {data_part.hex()}")
                                        continue
                                else:
                                    print(f"⚠️  Data corruption detected!")
                                    print(f"   Full frame hex: {data.hex()}")
                                    
                                    # Try to decode anyway for pattern analysis
                                    try:
                                        message = data_part.decode('utf-8', errors='replace')
                                        print(f"   Corrupted text: '{message}'")
                                    except:
                                        print(f"   Raw data: {data_part.hex()}")
                                    
                                    # Show byte-by-byte corruption analysis for first few frames
                                    if frame_count <= 3:
                                        print(f"   Byte analysis:")
                                        for i, byte_val in enumerate(data):
                                            char = chr(byte_val) if 32 <= byte_val <= 126 else f'\\x{byte_val:02x}'
                                            print(f"     [{i}]: 0x{byte_val:02x} = '{char}'")
                                    
                                    continue  # Skip the fallback decode
                            
                            # Try to decode as text (fallback for frames without checksum)
                            try:
                                message = data.decode('utf-8')
                                print(f"✅ Received: '{message}'")
                            except:
                                # Show hex for non-text data and try partial decode
                                print(f"📦 Binary data: {data.hex()}")
                                
                                # Try to decode first part to see if it's partially readable
                                try:
                                    # Try first 6 bytes for partial text
                                    partial = data[:6].decode('utf-8')
                                    corrupt_bytes = data[6:].hex() if len(data) > 6 else ""
                                    print(f"   Partial text: '{partial}' + corrupted bytes: {corrupt_bytes}")
                                except:
                                    print(f"   No readable text found")
                                    
                                # Check if this looks like our test pattern
                                if data.hex().startswith('546573742030'):  # "Test 0"
                                    expected_frame = data[:6].decode('utf-8', errors='ignore')
                                    print(f"   🎯 Looks like corrupted test frame: '{expected_frame}...'")
                                    print(f"   🔧 Corruption in last {len(data)-6} bytes")
                        
                        # Re-enable receiver for next frame
                        dwm.rx_enable()
                        
                    except Exception as e:
                        print(f"❌ Error processing frame: {e}")
                        dwm.rx_enable()
                
                # Check for receive errors
                elif status & 0xE0000:  # RX error bits
                    print(f"⚠️  RX Error: 0x{status:08X}")
                    dwm.rx_enable()
                
                # Small delay to prevent busy waiting
                time.sleep_ms(10)
                
            except Exception as e:
                print(f"❌ Main loop error: {e}")
                time.sleep_ms(100)
                try:
                    dwm.rx_enable()
                except:
                    pass
        
    except KeyboardInterrupt:
        print("\n⏹️  Receiver stopped by user")
        print(f"📊 Total frames received: {frame_count}")
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