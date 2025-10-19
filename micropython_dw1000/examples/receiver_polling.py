#!/usr/bin/env python3
"""
Simple UWB Receiver Example - Polling Mode

Continuously receives messages using the DW1000 module with polling.
This example demonstrates basic reception without interrupts.
"""

import machine
import dw1000
import time

# Hardware configuration
SPI_FREQ = 1000000
CS_PIN = 17
RESET_PIN = 21
IRQ_PIN = 20

def main():
    """Main receiver function with polling"""
    print("📡 DW1000 Receiver Starting (Polling Mode)...")
    
    # Setup SPI and pins
    spi = machine.SPI(0, 
                      baudrate=SPI_FREQ,
                      polarity=0, 
                      phase=0,
                      sck=machine.Pin(18),
                      mosi=machine.Pin(19),
                      miso=machine.Pin(16))
    
    cs_pin = machine.Pin(CS_PIN, machine.Pin.OUT)
    reset_pin = machine.Pin(RESET_PIN, machine.Pin.OUT)
    irq_pin = machine.Pin(IRQ_PIN, machine.Pin.IN)
    
    try:
        # Initialize DW1000
        dwt = dw1000.DW1000(spi, cs_pin, reset_pin, irq_pin)
        dwt.init()
        
        # Verify device
        device_id = dwt.read_device_id()
        print(f"✅ Device ID: 0x{device_id:08X}")
        
        # Configure for reliable reception (same as transmitter)
        dwt.configure(channel=2, data_rate=dw1000.BR_850K)  # 850 kbps for best reliability
        print("✅ Device configured for channel 2, 850K data rate")
        
        # Enable receiver
        dwt.rx_enable()
        print("📻 Receiver enabled - polling for frames...")
        
        frame_count = 0
        error_count = 0
        
        while True:
            try:
                # Poll for received frames
                status = dwt.get_status()
                
                # Check if frame received (bit 13 = RXDFR)
                if status & 0x2000:  # Frame received
                    try:
                        # Read frame data
                        rx_data = dwt.read_rx_data()
                        frame_count += 1
                        
                        # Display received message
                        try:
                            message = rx_data.decode('utf-8')
                            print(f"📦 Frame {frame_count}: {message}")
                        except:  # Catch any decode error
                            print(f"📦 Frame {frame_count}: {len(rx_data)} bytes (binary)")
                        
                    except Exception as e:
                        print(f"⚠️  Error reading frame: {e}")
                        error_count += 1
                    
                    # Always reset receiver after frame (good or bad)
                    # Use the robust reset sequence that works reliably
                    dwt.force_trx_off()
                    time.sleep_ms(2)
                    dwt.rx_enable()
                
                # Check for reception errors (bit 10 = RXFCE, bit 12 = LDEERR, etc.)
                elif status & 0x1C00:  # Any RX error bits (10, 11, 12)
                    error_count += 1
                    if error_count % 10 == 1:  # Show occasional error messages
                        print(f"⚠️  RX error detected: status=0x{status:04X}")
                    
                    # Reset receiver on errors - simple but reliable
                    dwt.force_trx_off()
                    time.sleep_ms(5)
                    dwt.rx_enable()
                
                # Small delay to prevent excessive polling
                time.sleep_ms(10)
                
            except Exception as e:
                print(f"❌ Critical error in main loop: {e}")
                error_count += 1
                
                # Simple but reliable recovery
                try:
                    dwt.force_trx_off()
                    time.sleep_ms(50)  # Longer delay for recovery
                    dwt.rx_enable()
                except Exception as recovery_error:
                    print(f"🚨 Recovery failed: {recovery_error}")
                    # Last resort - try to reinitialize
                    try:
                        dwt.init()
                        dwt.configure(channel=2, data_rate=dw1000.BR_850K)
                        dwt.rx_enable()
                        print("✅ Receiver reinitialized")
                    except Exception as init_error:
                        print(f"💥 Complete failure: {init_error}")
                        break  # Exit the loop if we can't recover
                
                time.sleep_ms(100)  # Longer delay after critical errors
            
    except KeyboardInterrupt:
        print(f"\n⏹️  Reception stopped by user")
        print(f"📊 Total frames received: {frame_count}")
        print(f"⚠️  Total errors: {error_count}")
    except Exception as e:
        print(f"❌ Error: {e}")
        print(f"📊 Frames received before error: {frame_count}")
        print(f"⚠️  Total errors: {error_count}")
    finally:
        try:
            dwt.force_trx_off()
            dwt.deinit()
            print("🔌 Device deinitialized")
        except:
            pass

if __name__ == "__main__":
    main()