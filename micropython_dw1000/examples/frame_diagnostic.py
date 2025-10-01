"""
Frame Structure Diagnostic

Analyzes the exact frame structure to determine correct overhead bytes
and fix the remaining corruption issues.
"""

import machine
import dw1000
import time

def analyze_frame_structure():
    """Analyze what we're actually receiving vs what's being sent"""
    print("=== Frame Structure Analysis ===")
    
    # What the transmitter is sending
    print("Expected transmitter frames:")
    for i in range(5):
        message = f"Test {i:03d}"
        checksum = sum(message.encode()) & 0xFF
        expected_frame = message.encode() + bytes([checksum])
        print(f"  Frame {i}: '{message}' + checksum({checksum:02x}) = {expected_frame.hex()}")
    
    print("\nFrames received (from your output):")
    received_frames = [
        "546573742030303050",  # "Test 000P"
        "546573742030303252",  # "Test 002R" 
        "546573742030303454",  # "Test 004T"
        "546573742030303656",  # "Test 006V"
        "546573742030303858",  # "Test 008X"
    ]
    
    for i, hex_frame in enumerate(received_frames):
        frame_bytes = bytes.fromhex(hex_frame)
        decoded = frame_bytes.decode('utf-8', errors='replace')
        print(f"  Frame {i}: {hex_frame} = '{decoded}'")
        
        # Analyze corruption
        expected_text = f"Test {i*2:03d}"  # We're getting every other frame
        expected_checksum = sum(expected_text.encode()) & 0xFF
        expected_hex = expected_text.encode().hex() + f"{expected_checksum:02x}"
        
        print(f"    Expected: {expected_hex} = '{expected_text}' + {expected_checksum:02x}")
        print(f"    Received: {hex_frame}")
        
        if hex_frame == expected_hex:
            print(f"    ✅ PERFECT MATCH")
        else:
            # Find where they differ
            expected_bytes = bytes.fromhex(expected_hex)
            for j in range(min(len(frame_bytes), len(expected_bytes))):
                if frame_bytes[j] != expected_bytes[j]:
                    print(f"    ❌ First difference at byte {j}: got 0x{frame_bytes[j]:02x}, expected 0x{expected_bytes[j]:02x}")
                    break

def test_frame_info_register():
    """Test what the RX_FINFO register actually contains"""
    print("\n=== RX_FINFO Register Analysis ===")
    
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
        
        config = {'channel': 5, 'data_rate': dw1000.BR_850K}
        dwm.configure(config)
        
        print("Waiting for a few frames to analyze RX_FINFO register...")
        dwm.rx_enable()
        
        frames_analyzed = 0
        
        while frames_analyzed < 3:
            status = dwm.get_status()
            
            if status == 0xFFFFFFFF or status > 0x7FFFFFFF:
                time.sleep_ms(10)
                continue
            
            if status & 0x4000:  # RXFCG bit
                frames_analyzed += 1
                print(f"\n--- Frame {frames_analyzed} Register Analysis ---")
                
                # We need to read the raw register to see what it contains
                # The frame length calculation should be:
                # total_frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
                
                # Let's see what our current read_rx_data returns
                data = dwm.read_rx_data()
                print(f"Current read_rx_data result:")
                print(f"  Length: {len(data)} bytes")
                print(f"  Hex: {data.hex()}")
                print(f"  Text: '{data.decode('utf-8', errors='replace')}'")
                
                # For a "Test 000" frame with checksum, we expect:
                # "Test 000" = 8 bytes + 1 checksum = 9 bytes payload
                # Total frame = 9 bytes payload + overhead
                
                if len(data) == 9:
                    print(f"  ✅ Length is correct (9 bytes)")
                    
                    # Check if last byte looks like a checksum
                    payload = data[:-1]
                    received_checksum = data[-1]
                    calculated_checksum = sum(payload) & 0xFF
                    
                    print(f"  Payload: '{payload.decode('utf-8', errors='replace')}'")
                    print(f"  Received checksum: 0x{received_checksum:02x} ({chr(received_checksum) if 32 <= received_checksum <= 126 else '?'})")
                    print(f"  Calculated checksum: 0x{calculated_checksum:02x}")
                    
                    if received_checksum == calculated_checksum:
                        print(f"  ✅ Checksum is CORRECT")
                    else:
                        print(f"  ❌ Checksum is WRONG")
                        print(f"     This suggests frame corruption or wrong overhead calculation")
                
                else:
                    print(f"  ❌ Length is wrong (expected 9 bytes)")
                
                dwm.rx_enable()
            
            elif status & 0xE0000:
                print(f"⚠️  RX Error: 0x{status:08X}")
                dwm.rx_enable()
            
            time.sleep_ms(50)
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
    finally:
        try:
            dwm.deinit()
        except:
            pass

def main():
    print("Frame Structure Diagnostic Tool")
    print("===============================")
    print("Analyzing frame corruption and overhead calculation\n")
    
    # First, analyze the pattern from the received data
    analyze_frame_structure()
    
    # Then test with actual hardware
    test_frame_info_register()
    
    print("\n" + "="*50)
    print("DIAGNOSTIC SUMMARY")
    print("="*50)
    print("Based on the analysis:")
    print("1. Check if checksums match expected values")
    print("2. Verify frame overhead calculation (2 vs 4 bytes)")
    print("3. Look for patterns in frame corruption")
    print("4. Consider if transmitter is sending correct data")

if __name__ == "__main__":
    main()