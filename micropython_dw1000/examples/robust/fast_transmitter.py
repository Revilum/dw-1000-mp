"""
Fast Continuous DW1000 Transmitter
High-frequency transmission for stress testing receivers

Usage: mpremote connect /dev/ttyACM1 run fast_transmitter.py
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
    
    config = {
        'channel': 2,
        'data_rate': dw1000.BR_850K
    }
    dwt.configure(config)
    
    print("DW1000 initialized for fast transmission")
    return dwt

def fast_transmit():
    """Run fast transmission loop - every 200ms"""
    dwt = setup_dw1000()
    
    frame_count = 0
    start_time = time.ticks_ms()
    
    print("Starting FAST continuous transmission (200ms intervals)...")
    print("Press Ctrl+C to stop")
    
    try:
        while True:
            frame_count += 1
            
            # Short, varied messages for fast transmission
            message_type = frame_count % 4
            if message_type == 0:
                message = f"FAST-{frame_count:04d}"
            elif message_type == 1:
                message = f"QUICK-TEST-{frame_count:04d}"
            elif message_type == 2:
                message = f"BURST-{frame_count:04d}-850K"
            else:
                message = f"RAPID-{frame_count:04d}"
            
            frame_data = message.encode('utf-8')
            
            try:
                dwt.tx_frame(frame_data)
                
                # Print every 25th frame
                if frame_count % 25 == 0:
                    elapsed = time.ticks_diff(time.ticks_ms(), start_time)
                    rate = frame_count * 1000 / elapsed if elapsed > 0 else 0
                    print(f"TX #{frame_count:04d}: {message} ({rate:.1f} fps)")
                
            except Exception as e:
                print(f"TX ERROR #{frame_count}: {e}")
            
            # Fast transmission - 200ms between frames
            time.sleep_ms(200)
            
    except KeyboardInterrupt:
        elapsed = time.ticks_diff(time.ticks_ms(), start_time)
        rate = frame_count * 1000 / elapsed if elapsed > 0 else 0
        print(f"\nFast transmission stopped.")
        print(f"Sent {frame_count} frames in {elapsed} ms ({rate:.1f} fps)")
    
    except Exception as e:
        print(f"Unexpected error: {e}")
    
    finally:
        try:
            dwt.force_trx_off()
            dwt.deinit()
            print("DW1000 cleaned up")
        except:
            pass

if __name__ == "__main__":
    fast_transmit()