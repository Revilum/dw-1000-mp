"""
Robust DW1000 Receiver with Timeout and Error Recovery
Implements DW1000 manual recommendations for continuous operation

Based on:
- DW1000 User Manual section 4.1.6 (RX Message timestamp)  
- Section 4.7 (Assessing quality of reception)
- PolyPoint receiver patterns
- Timeout handling and receiver reset procedures
"""

import dw1000
import machine
import time

def setup_dw1000():
    """Initialize DW1000 with robust configuration"""
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
    
    print("DW1000 initialized with robust configuration")
    return dwt

def reset_receiver(dwt):
    """Complete receiver reset sequence (DW1000 manual recommended)"""
    # Force transceiver off first
    dwt.force_trx_off()
    time.sleep_ms(2)  # Allow proper state transition
    
    # Reset RX buffers and sync pointers
    dwt.rx_reset()
    dwt.sync_rx_bufptrs()
    
    # Short delay before re-enabling
    time.sleep_ms(1)
    
    # Re-enable receiver
    dwt.rx_enable()

def robust_receive():
    """Robust receiver with timeout and error recovery"""
    dwt = setup_dw1000()
    
    # Initialize receiver
    reset_receiver(dwt)
    
    frames_received = 0
    frames_good = 0
    frames_error = 0
    timeouts = 0
    resets = 0
    start_time = time.ticks_ms()
    last_activity = start_time
    
    # Timeout configuration (milliseconds)
    ACTIVITY_TIMEOUT = 5000    # Reset if no activity for 5 seconds
    STATUS_CHECK_INTERVAL = 50  # Check status every 50ms
    RESET_INTERVAL = 10000     # Periodic reset every 10 seconds as fallback
    
    last_reset = start_time
    
    print("Robust receiver starting...")
    print("Monitoring with timeout and error recovery...")
    print("Press Ctrl+C to stop")
    print("-" * 60)
    
    try:
        while True:
            current_time = time.ticks_ms()
            
            # Check for activity timeout
            if time.ticks_diff(current_time, last_activity) > ACTIVITY_TIMEOUT:
                print(f"⏰ TIMEOUT: No activity for {ACTIVITY_TIMEOUT}ms - resetting receiver")
                reset_receiver(dwt)
                resets += 1
                timeouts += 1
                last_activity = current_time
                last_reset = current_time
                continue
            
            # Periodic preventive reset (fallback safety measure)
            if time.ticks_diff(current_time, last_reset) > RESET_INTERVAL:
                print(f"🔄 PERIODIC: Preventive receiver reset after {RESET_INTERVAL}ms")
                reset_receiver(dwt)
                resets += 1
                last_reset = current_time
                last_activity = current_time
                continue
            
            # Check receiver status
            status = dwt.get_status()
            
            # Look for frame activity
            if status & (0x2000 | 0x0400 | 0x8000):  # Good, Error, or Ready
                frames_received += 1
                last_activity = current_time
                
                frame_good = (status & 0x2000) != 0
                frame_error = (status & 0x0400) != 0
                frame_ready = (status & 0x8000) != 0
                
                # Get frame length if available
                try:
                    rx_finfo = dwt.get_rx_finfo()
                    frame_length = rx_finfo & 0x000003FF
                except:
                    frame_length = 0
                
                elapsed = time.ticks_diff(current_time, start_time)
                rate = frames_received * 1000 / elapsed if elapsed > 0 else 0
                
                print(f"Frame #{frames_received:04d} [{elapsed//1000:3d}s] Rate: {rate:.1f}fps")
                print(f"  Status: 0x{status:08X} (Good:{frame_good} Err:{frame_error} Rdy:{frame_ready})")
                print(f"  Length: {frame_length} bytes")
                
                # Handle different frame states
                if frame_good and frame_length > 0:
                    try:
                        data = dwt.read_rx_data()
                        frames_good += 1
                        try:
                            message = data.decode('utf-8')
                        except:
                            message = str(data)
                        print(f"  ✅ MESSAGE: '{message}' ({len(data)} bytes)")
                        
                    except Exception as e:
                        print(f"  ❌ READ ERROR: {e}")
                        # Force reset on read errors
                        reset_receiver(dwt)
                        resets += 1
                        continue
                
                elif frame_error:
                    frames_error += 1
                    print(f"  ⚠️ FRAME ERROR - corrupted or invalid frame")
                
                elif frame_ready and frame_length > 0:
                    print(f"  📋 FRAME READY - attempting read...")
                    try:
                        data = dwt.read_rx_data()
                        if data:
                            frames_good += 1
                            print(f"  ✅ RECOVERED: {len(data)} bytes from ready frame")
                    except:
                        print(f"  ❌ READY frame unreadable")
                
                else:
                    print(f"  ❓ UNKNOWN STATE - no readable data")
                
                # Always reset receiver after processing (DW1000 manual recommendation)
                print(f"  🔄 Resetting receiver for next frame...")
                reset_receiver(dwt)
                resets += 1
                
                print("-" * 60)
            
            # Check for error conditions that require reset
            elif status != 0:
                # Some non-frame status - might indicate error state
                print(f"⚠️ NON-FRAME STATUS: 0x{status:08X} - checking for errors")
                
                # Check for specific error conditions
                if status & 0x1000:  # PHR error
                    print("🚨 PHR ERROR detected - resetting")
                    reset_receiver(dwt)
                    resets += 1
                    last_activity = current_time
                elif status & 0x10000:  # Reed Solomon error
                    print("🚨 REED SOLOMON ERROR detected - resetting")
                    reset_receiver(dwt)
                    resets += 1
                    last_activity = current_time
                elif status & 0x20000:  # Frame wait timeout
                    print("🚨 FRAME WAIT TIMEOUT - resetting")
                    reset_receiver(dwt)
                    resets += 1
                    last_activity = current_time
            
            # Brief delay before next check
            time.sleep_ms(STATUS_CHECK_INTERVAL)
            
    except KeyboardInterrupt:
        elapsed_total = time.ticks_diff(time.ticks_ms(), start_time)
        avg_rate = frames_received * 1000 / elapsed_total if elapsed_total > 0 else 0
        
        print(f"\n{'='*60}")
        print("ROBUST RECEIVER STATISTICS")
        print(f"{'='*60}")
        print(f"Runtime: {elapsed_total//1000} seconds")
        print(f"Total frames detected: {frames_received}")
        print(f"Good frames: {frames_good}")
        print(f"Error frames: {frames_error}")
        print(f"Timeouts: {timeouts}")
        print(f"Receiver resets: {resets}")
        print(f"Average rate: {avg_rate:.1f} frames/second")
        if frames_received > 0:
            print(f"Success rate: {(frames_good/frames_received*100):.1f}%")
        else:
            print("No frames received")
    
    except Exception as e:
        print(f"Unexpected error: {e}")
    
    finally:
        # Cleanup
        try:
            dwt.force_trx_off()
            dwt.deinit()
            print("DW1000 cleaned up")
        except:
            pass

if __name__ == "__main__":
    robust_receive()