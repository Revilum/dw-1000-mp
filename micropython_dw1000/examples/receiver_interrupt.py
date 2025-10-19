#!/usr/bin/env python3
"""
Professional UWB Receiver Example - Interrupt + Callbacks

Continuously receives messages using the DW1000 module with hardware interrupts.
This example demonstrates professional event-driven reception with optimal performance.
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
    """Main receiver function with interrupt-driven callbacks"""
    print("🚀 DW1000 Receiver Starting (Interrupt + Callbacks Mode)...")
    
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
    irq_pin = machine.Pin(IRQ_PIN, machine.Pin.IN, machine.Pin.PULL_DOWN)
    
    # Performance counters
    frame_count = 0
    error_count = 0
    start_time = time.ticks_ms()
    
    def frame_received_callback(frame_info):
        """Called when a frame is successfully received"""
        nonlocal frame_count
        frame_count += 1
        
        if frame_info['event'] == 'rx_good':
            data = frame_info['data']
            length = frame_info['length']
            rx_time = frame_info['rx_time_ms']
            
            # The C callback now properly removes CRC, so data is clean
            # Display received message
            try:
                message = data.decode('utf-8')
                print(f"📦 Frame {frame_count}: {message}")
            except:  # Catch any decode error
                print(f"📦 Frame {frame_count}: {length} bytes (binary)")
            
            # Show performance info every 10 frames
            if frame_count % 10 == 0:
                elapsed = time.ticks_diff(time.ticks_ms(), start_time) / 1000
                rate = frame_count / elapsed if elapsed > 0 else 0
                print(f"📊 Performance: {frame_count} frames, {rate:.1f} fps, {error_count} errors")
    
    def frame_error_callback(error_info):
        """Called when a frame reception error occurs"""
        nonlocal error_count
        error_count += 1
        
        if error_count % 10 == 1:  # Show occasional error messages
            print(f"⚠️  Frame error {error_count}: status=0x{error_info['status']:04X}")
        
        # Simple but reliable error recovery
        # The auto_rx feature should handle most recovery automatically
    
    def irq_handler(pin):
        """Hardware interrupt handler - calls C-level processing"""
        dwt.process_events()
    
    try:
        # Initialize DW1000
        dwt = dw1000.DW1000(spi, cs_pin, reset_pin, irq_pin)
        dwt.init(auto_rx=True)  # Enable auto RX for continuous operation
        
        # Verify device
        device_id = dwt.read_device_id()
        print(f"✅ Device ID: 0x{device_id:08X}")
        
        # Configure for reliable reception (same as transmitter)
        dwt.configure(channel=2, data_rate=dw1000.BR_850K)  # 850 kbps for best reliability
        print("✅ Device configured for channel 2, 850K data rate")
        
        # Setup callbacks for event-driven operation
        dwt.set_rx_callback(frame_received_callback)
        dwt.set_error_callback(frame_error_callback)
        dwt.set_auto_rx(True)  # Auto re-enable RX after each frame
        
        # Setup hardware interrupt with hybrid IRQ system
        print("🔧 Setting up hardware interrupt...")
        irq_pin.irq(trigger=machine.Pin.IRQ_RISING, handler=irq_handler)
        dwt.enable_irq()
        
        # Enable receiver
        dwt.rx_enable()
        print("📻 Receiver enabled - interrupt-driven reception active")
        print("⚡ Hardware IRQ → C-level processing → Python callbacks")
        print("🎯 Optimal performance with MicroPython compatibility")
        print("")
        
        # Main loop - just keep the program running
        # All frame processing happens in interrupt-driven callbacks
        try:
            while True:
                time.sleep(1)
                
                # Optional: Show alive indicator every 30 seconds
                if time.ticks_ms() % 30000 < 1000:
                    elapsed = time.ticks_diff(time.ticks_ms(), start_time) / 1000
                    print(f"💚 System alive - {elapsed:.0f}s runtime, {frame_count} frames received")
                    
        except KeyboardInterrupt:
            print(f"\n⏹️  Reception stopped by user")
            
    except Exception as e:
        print(f"❌ Error: {e}")
        import sys
        sys.print_exception(e)
    finally:
        try:
            # Cleanup
            dwt.disable_irq()
            irq_pin.irq(handler=None)
            dwt.force_trx_off()
            dwt.deinit()
            print("🔌 Device deinitialized")
            
            # Final statistics
            elapsed = time.ticks_diff(time.ticks_ms(), start_time) / 1000
            rate = frame_count / elapsed if elapsed > 0 else 0
            print(f"📊 Final Stats: {frame_count} frames, {rate:.1f} fps, {error_count} errors")
            
        except:
            pass

if __name__ == "__main__":
    main()