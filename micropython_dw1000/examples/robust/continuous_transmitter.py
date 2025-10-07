"""
Continuous DW1000 Transmitter
Runs indefinitely sending test frames at regular intervals
Perfect for testing receiver implementations

Usage: mpremote connect /dev/ttyACM1 run continuous_transmitter.py
"""

import dw1000
import machine
import time

def setup_dw1000():
    """Initialize DW1000 with standard configuration"""
    spi = machine.SPI(0, baudrate=1000000, polarity=0, phase=0, 
                      sck=machine.Pin(18), mosi=machine.Pin(19), miso=machine.Pin(16))
    cs_pin = machine.Pin(17, machine.Pin.OUT)
    reset_pin = machine.Pin(21, machine.Pin.OUT)
    irq_pin = machine.Pin(20, machine.Pin.IN)
    
    dwt = dw1000.DW1000(spi, cs_pin, reset_pin, irq_pin)
    dwt.init()
    
    # Configure for 850K data rate on channel 2 (PolyPoint standard)
    config = {
        'channel': 2,
        'data_rate': dw1000.BR_850K
    }
    dwt.configure(config)
    
    print("DW1000 initialized: 850K data rate, Channel 2")
    return dwt

def continuous_transmit():
    """Run continuous transmission loop"""
    dwt = setup_dw1000()
    
    frame_count = 0
    start_time = time.ticks_ms()
    
    print("Starting continuous transmission...")
    print("Press Ctrl+C to stop")
    
    try:
        while True:
            frame_count += 1
            
            # Create frame with timestamp and counter
            current_time = time.ticks_ms()
            elapsed = time.ticks_diff(current_time, start_time)
            
            # Multiple frame types for variety
            if frame_count % 5 == 0:
                message = f"HEARTBEAT-{frame_count:06d}-{elapsed}ms"
            elif frame_count % 3 == 0:
                message = f"TEST-FRAME-{frame_count:06d}"
            else:
                message = f"DATA-{frame_count:06d}-850K"
            
            frame_data = message.encode('utf-8')
            
            # Transmit frame
            try:
                dwt.tx_frame(frame_data)
                
                # Status check for TX completion (optional)
                status = dwt.get_status()
                tx_complete = (status & 0x0080) != 0
                
                # Print every 10th frame to avoid spam
                if frame_count % 10 == 0:
                    print(f"TX #{frame_count:06d}: {len(frame_data)} bytes, Status: 0x{status:08X}, Complete: {tx_complete}")
                    print(f"  Message: {message}")
                
            except Exception as e:
                print(f"TX ERROR #{frame_count}: {e}")
            
            # Inter-frame delay (adjustable)
            time.sleep_ms(1000)  # 1 second between frames
            
    except KeyboardInterrupt:
        print(f"\nTransmission stopped. Sent {frame_count} frames total.")
        print(f"Total runtime: {time.ticks_diff(time.ticks_ms(), start_time)} ms")
    
    except Exception as e:
        print(f"Unexpected error: {e}")
        print(f"Frames sent before error: {frame_count}")
    
    finally:
        # Cleanup
        try:
            dwt.force_trx_off()
            dwt.deinit()
            print("DW1000 cleaned up")
        except:
            pass

if __name__ == "__main__":
    continuous_transmit()