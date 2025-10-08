"""
Event-Driven UWB Receiver Example

Uses callbacks for efficient frame reception without blocking.
"""

import machine
import dw1000
import time

# Hardware configuration
SPI_FREQ = 1000000
CS_PIN = 17
RESET_PIN = 21
IRQ_PIN = 20

class UWBReceiver:
    """Event-driven UWB receiver using callbacks"""
    
    def __init__(self):
        self.frame_count = 0
        self.dwt = None
        
    def setup_hardware(self):
        """Initialize hardware and DW1000"""
        print("🔧 Setting up hardware...")
        
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
        
        # Initialize DW1000 (auto-init enabled)
        self.dwt = dw1000.DW1000(spi, cs_pin, reset_pin, irq_pin)
        
        # Verify device
        device_id = self.dwt.read_device_id()
        print(f"✅ Device ID: 0x{device_id:08X}")
        
        # Configure for reliable reception
        config = {
            'channel': 2,
            'data_rate': dw1000.BR_850K  # 850 kbps for best reliability
        }
        self.dwt.configure(config)
        print("✅ Device configured")
        
    def on_frame_received(self, frame_info):
        """Callback for received frames"""
        if frame_info['event'] == 'rx_good':
            self.frame_count += 1
            data = frame_info['data']
            length = frame_info['length']
            rx_time = frame_info['rx_time_ms']
            
            try:
                message = data.decode('utf-8')
                print(f"📥 [{self.frame_count:04d}] Received ({length}B): {message}")
                print(f"    RX Time: {rx_time}ms")
            except UnicodeDecodeError:
                print(f"📥 [{self.frame_count:04d}] Binary data ({length}B): {data.hex()}")
                
        elif frame_info['event'] == 'rx_error':
            print(f"❌ Frame error: status=0x{frame_info['status']:08X}")
            
        # Reset receiver for next frame (robust pattern)
        self.reset_receiver()
        
    def on_error(self, error_info):
        """Callback for error events"""
        print(f"⚠️  Error: {error_info}")
        self.reset_receiver()
        
    def on_timeout(self):
        """Callback for timeout events"""
        print("⏰ Timeout occurred")
        self.reset_receiver()
        
    def reset_receiver(self):
        """Reset receiver for next frame (handles blocking issues)"""
        self.dwt.force_trx_off()
        time.sleep_ms(2)
        self.dwt.rx_reset()
        self.dwt.sync_rx_bufptrs()
        time.sleep_ms(1)
        self.dwt.rx_enable()
        
    def start_receiving(self):
        """Start event-driven reception"""
        print("🎯 Setting up callbacks...")
        
        # Register callbacks
        self.dwt.set_rx_callback(self.on_frame_received)
        self.dwt.set_error_callback(self.on_error)
        self.dwt.set_timeout_callback(self.on_timeout)
        
        # Enable auto RX mode for continuous operation
        self.dwt.set_auto_rx(True)
        
        # Initial receiver setup
        self.reset_receiver()
        
        print("👂 Listening for frames... (Ctrl+C to stop)")
        
        # Event processing loop
        try:
            while True:
                self.dwt.process_events()  # Triggers callbacks
                time.sleep_ms(10)  # Don't hog CPU
                
        except KeyboardInterrupt:
            print(f"\n⏹️  Reception stopped by user")
            print(f"📊 Total frames received: {self.frame_count}")
            
    def cleanup(self):
        """Clean up resources"""
        if self.dwt:
            try:
                self.dwt.deinit()
                print("🔌 Device deinitialized")
            except:
                pass

def main():
    """Main receiver function"""
    print("🚀 DW1000 Callback Receiver Starting...")
    
    receiver = UWBReceiver()
    
    try:
        receiver.setup_hardware()
        receiver.start_receiving()
        
    except Exception as e:
        print(f"❌ Error: {e}")
        
    finally:
        receiver.cleanup()

if __name__ == "__main__":
    main()