"""
Simple UWB Transmitter Example

Continuously transmits messages using the DW1000 module.
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
    """Main transmitter function"""
    print("🚀 DW1000 Transmitter Starting...")
    
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
        # Initialize DW1000 (auto-init enabled)
        dwt = dw1000.DW1000(spi, cs_pin, reset_pin, irq_pin)
        
        # Verify device
        device_id = dwt.read_device_id()
        print(f"✅ Device ID: 0x{device_id:08X}")
        
        # Configure for reliable transmission
        config = {
            'channel': 2,
            'data_rate': dw1000.BR_850K  # 850 kbps for best reliability
        }
        dwt.configure(config)
        print("✅ Device configured")
        
        # Transmission loop
        message_count = 0
        
        while True:
            # Create message with counter
            message = f"Hello UWB World! #{message_count:04d}".encode('utf-8')
            
            # Transmit frame
            dwt.tx_frame(message)
            print(f"📡 Sent: {message.decode('utf-8')}")
            
            message_count += 1
            time.sleep(1)  # 1 second between messages
            
    except KeyboardInterrupt:
        print("\n⏹️  Transmission stopped by user")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        try:
            dwt.deinit()
            print("🔌 Device deinitialized")
        except:
            pass

if __name__ == "__main__":
    main()