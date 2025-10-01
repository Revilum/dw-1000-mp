"""
DW1000 Reliable Transmitter Example

Based on minimal test - uses stable configuration
"""

import machine
import dw1000
import time

def main():
    print("DW1000 Reliable Transmitter")
    print("===========================")
    
    # Use same stable settings as minimal test
    spi = machine.SPI(0, baudrate=1000000)  # 1MHz for stability
    cs_pin = machine.Pin(17, machine.Pin.OUT)
    reset_pin = machine.Pin(21, machine.Pin.OUT)
    
    try:
        # Initialize with extra stabilization time
        print("Initializing DW1000...")
        dwm = dw1000.DW1000(spi, cs_pin, reset_pin)
        
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
        
        print("\nStarting transmission...")
        print("Sending 10 test messages...")
        
        # Transmit messages with error checking
        for message_count in range(10):
            try:
                # Create simple message with checksum
                message_text = f"Test {message_count:03d}"
                message = message_text.encode('utf-8')
                
                # Add simple checksum for integrity checking
                checksum = sum(message) & 0xFF
                message_with_checksum = message + bytes([checksum])
                
                print(f"\nFrame {message_count + 1}:")
                print(f"  Text: '{message_text}'")
                print(f"  Bytes: {len(message)} + 1 checksum = {len(message_with_checksum)}")
                print(f"  Data: {message.hex()}")
                print(f"  Checksum: {checksum:02x}")
                print(f"  Full frame: {message_with_checksum.hex()}")
                
                # Send the frame with checksum
                result = dwm.tx_frame(message_with_checksum)
                
                if result is None:
                    print("  ✅ Transmitted successfully")
                else:
                    print(f"  ❌ TX error: {result}")
                
                # Wait between transmissions
                time.sleep(2)
                
            except Exception as e:
                print(f"  ❌ Exception during TX: {e}")
                time.sleep(1)  # Brief pause before continuing
        
        print("\n✅ Transmission sequence complete")
        
    except KeyboardInterrupt:
        print("\n⏹️  Transmission stopped by user")
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