"""
Continuous DW1000 Receiver
Listens indefinitely and prints all received messages
Perfect for testing with continuous_transmitter.py

Usage: mpremote connect /dev/ttyACM0 run continuous_receiver.py
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

def prepare_receiver(dwt):
    """Setup receiver with proper reset sequence"""
    dwt.force_trx_off()
    time.sleep_ms(1)
    dwt.rx_reset()
    dwt.sync_rx_bufptrs()
    dwt.rx_enable()

def continuous_receive():
    """Run continuous receiver loop"""
    dwt = setup_dw1000()
    prepare_receiver(dwt)
    
    frames_received = 0
    frames_good = 0
    frames_error = 0
    read_errors = 0
    start_time = time.ticks_ms()
    
    print("Starting continuous receiver...")
    print("Listening for messages from transmitter...")
    print("Press Ctrl+C to stop")
    print("-" * 60)
    
    try:
        while True:
            status = dwt.get_status()
            
            # Check for any frame activity
            if status & (0x2000 | 0x0400 | 0x8000):  # Good, Error, or Ready
                frames_received += 1
                
                # Get frame information
                rx_finfo = dwt.get_rx_finfo()
                frame_length = rx_finfo & 0x000003FF
                frame_good = (status & 0x2000) != 0
                frame_error = (status & 0x0400) != 0
                frame_ready = (status & 0x8000) != 0
                
                if frame_good:
                    frames_good += 1
                if frame_error:
                    frames_error += 1
                
                # Calculate elapsed time and rate
                elapsed = time.ticks_diff(time.ticks_ms(), start_time)
                rate = frames_received * 1000 / elapsed if elapsed > 0 else 0
                
                # Print frame info
                print(f"Frame #{frames_received:04d} [{elapsed//1000:3d}s] Status: 0x{status:08X}")
                print(f"  Good: {frame_good}, Error: {frame_error}, Ready: {frame_ready}")
                print(f"  Expected Length: {frame_length} bytes")
                print(f"  Rate: {rate:.1f} fps")
                
                # Try to read the message if frame looks good
                if (frame_good or frame_ready) and frame_length > 0:
                    try:
                        data = dwt.read_rx_data()
                        try:
                            message = data.decode('utf-8')
                        except:
                            message = str(data)  # Fallback to bytes representation
                        print(f"  ✅ MESSAGE: '{message}'")
                        print(f"  📊 Actual Length: {len(data)} bytes")
                        
                    except Exception as e:
                        read_errors += 1
                        print(f"  ❌ Read Error #{read_errors}: {e}")
                
                elif frame_error:
                    print(f"  ⚠️  Frame Error - skipping read")
                
                else:
                    print(f"  ⚠️  No data to read (length={frame_length})")
                
                print("-" * 60)
                
                # Reset receiver for next frame
                prepare_receiver(dwt)
            
            # Short delay to prevent overwhelming the system
            time.sleep_ms(10)
            
    except KeyboardInterrupt:
        elapsed_total = time.ticks_diff(time.ticks_ms(), start_time)
        avg_rate = frames_received * 1000 / elapsed_total if elapsed_total > 0 else 0
        
        print(f"\n{'='*60}")
        print("RECEIVER STATISTICS")
        print(f"{'='*60}")
        print(f"Runtime: {elapsed_total//1000} seconds")
        print(f"Total frames received: {frames_received}")
        print(f"Good frames: {frames_good}")
        print(f"Error frames: {frames_error}")
        print(f"Read errors: {read_errors}")
        print(f"Average rate: {avg_rate:.1f} frames/second")
        print(f"Success rate: {(frames_good/frames_received*100):.1f}%" if frames_received > 0 else "No frames")
        
    except Exception as e:
        print(f"Unexpected error: {e}")
        print(f"Frames received before error: {frames_received}")
    
    finally:
        # Cleanup
        try:
            dwt.force_trx_off()
            dwt.deinit()
            print("DW1000 cleaned up")
        except:
            pass

if __name__ == "__main__":
    continuous_receive()